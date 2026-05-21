#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes


# ============================================================
# 좌표 입력
# ============================================================

POINT_1_X = -0.215
POINT_1_Y = -0.045

POINT_2_X = 3.685
POINT_2_Y = -0.045

MAX_INITIAL_POSE_ERROR = 0.75
MAX_XY_COVARIANCE = 0.8
MAX_YAW_COVARIANCE = 0.8


# ============================================================
# 유틸 함수
# ============================================================

def calc_yaw_from_to(x_from, y_from, x_to, y_to):
    return math.atan2(y_to - y_from, x_to - x_from)


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class TwoPointMission(Node):
    def __init__(self):
        super().__init__("two_point_mission")

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self.last_amcl_pose = None
        self.last_amcl_received_time = 0.0
        self.last_feedback_time = 0.0

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10,
        )

    def amcl_callback(self, msg):
        self.last_amcl_pose = msg
        self.last_amcl_received_time = time.time()

    def make_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def publish_initial_pose(self, x, y, yaw):
        self.last_amcl_pose = None
        self.last_amcl_received_time = 0.0

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # A zero stamp asks TF consumers to use the latest available transform.
        # This avoids multi-machine clock skew turning initial pose into a TF extrapolation.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # RViz 2D Pose Estimate와 비슷한 covariance
        msg.pose.covariance[0] = 0.25      # x
        msg.pose.covariance[7] = 0.25      # y
        msg.pose.covariance[35] = 0.0685   # yaw

        self.get_logger().info(
            f"Publishing initial pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        # AMCL이 확실히 받도록 여러 번 publish
        for _ in range(10):
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.wait_for_localization_stable(
            timeout_sec=10.0,
            expected_x=x,
            expected_y=y,
            max_position_error=MAX_INITIAL_POSE_ERROR,
        )

    def wait_for_localization_stable(
        self,
        timeout_sec=5.0,
        expected_x=None,
        expected_y=None,
        max_position_error=None,
    ):
        start_time = time.time()
        last_log_time = 0.0

        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.last_amcl_pose is None:
                continue

            pose = self.last_amcl_pose.pose.pose
            covariance = self.last_amcl_pose.pose.covariance
            x_cov = covariance[0]
            y_cov = covariance[7]
            yaw_cov = covariance[35]

            position_error = None
            position_ok = True
            if expected_x is not None and expected_y is not None:
                position_error = math.hypot(
                    pose.position.x - expected_x,
                    pose.position.y - expected_y,
                )
                position_ok = (
                    max_position_error is None
                    or position_error <= max_position_error
                )

            covariance_ok = (
                x_cov <= MAX_XY_COVARIANCE
                and y_cov <= MAX_XY_COVARIANCE
                and yaw_cov <= MAX_YAW_COVARIANCE
            )

            if position_ok and covariance_ok:
                self.get_logger().info(
                    "AMCL localization is stable: "
                    f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
                    f"cov=({x_cov:.3f}, {y_cov:.3f}, yaw={yaw_cov:.3f})"
                )
                return True

            now = time.time()
            if now - last_log_time >= 1.0:
                if position_error is None:
                    self.get_logger().warn(
                        "Waiting for AMCL localization to stabilize: "
                        f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
                        f"cov=({x_cov:.3f}, {y_cov:.3f}, yaw={yaw_cov:.3f})"
                    )
                else:
                    self.get_logger().warn(
                        "Waiting for AMCL localization to stabilize: "
                        f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
                        f"initial_pose_error={position_error:.3f} m, "
                        f"cov=({x_cov:.3f}, {y_cov:.3f}, yaw={yaw_cov:.3f})"
                    )
                last_log_time = now

        self.get_logger().error(
            "AMCL localization is not stable enough; refusing to send navigation goal"
        )
        return False

    def call_service(self, client, request, timeout_sec=10.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            return None

        return future.result()

    def get_lifecycle_state(self, node_name):
        client = self.create_client(GetState, f"{node_name}/get_state")
        response = self.call_service(client, GetState.Request(), timeout_sec=2.0)
        self.destroy_client(client)

        if response is None:
            return None

        return response.current_state.id

    def change_lifecycle_state(self, node_name, transition_id):
        client = self.create_client(ChangeState, f"{node_name}/change_state")
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self.call_service(client, request, timeout_sec=10.0)
        self.destroy_client(client)

        return response is not None and response.success

    def activate_lifecycle_node(self, node_name):
        state = self.get_lifecycle_state(node_name)

        if state == 3:  # active
            return True

        if state == 1:  # unconfigured
            self.get_logger().info(f"Configuring {node_name}")
            if not self.change_lifecycle_state(
                node_name,
                Transition.TRANSITION_CONFIGURE,
            ):
                self.get_logger().warn(f"Failed to configure {node_name}")
                return False
            state = self.get_lifecycle_state(node_name)

        if state == 2:  # inactive
            self.get_logger().info(f"Activating {node_name}")
            if not self.change_lifecycle_state(
                node_name,
                Transition.TRANSITION_ACTIVATE,
            ):
                self.get_logger().warn(f"Failed to activate {node_name}")
                return False

        return self.get_lifecycle_state(node_name) == 3

    def ensure_navigation_active(self):
        required_nodes = [
            "/local_costmap/local_costmap",
            "/global_costmap/global_costmap",
            "/controller_server",
            "/planner_server",
            "/smoother_server",
            "/bt_navigator",
            "/behavior_server",
            "/velocity_smoother",
            "/collision_monitor",
        ]

        inactive_nodes = [
            node_name
            for node_name in required_nodes
            if self.get_lifecycle_state(node_name) != 3
        ]

        if not inactive_nodes:
            self.get_logger().info("Nav2 lifecycle nodes are already active")
            return True

        self.get_logger().info(
            "Starting Nav2 lifecycle nodes: " + ", ".join(inactive_nodes)
        )

        manager_client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )
        manager_request = ManageLifecycleNodes.Request()
        manager_request.command = ManageLifecycleNodes.Request.STARTUP
        manager_response = self.call_service(
            manager_client,
            manager_request,
            timeout_sec=20.0,
        )
        self.destroy_client(manager_client)

        if manager_response is not None and manager_response.success:
            start_time = time.time()
            while time.time() - start_time < 10.0:
                if all(
                    self.get_lifecycle_state(node_name) == 3
                    for node_name in required_nodes
                ):
                    self.get_logger().info("Nav2 lifecycle startup complete")
                    return True
                rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().warn(
            "Lifecycle manager startup did not activate every node; trying direct transitions"
        )

        failed_nodes = [
            node_name
            for node_name in required_nodes
            if not self.activate_lifecycle_node(node_name)
        ]

        if failed_nodes:
            self.get_logger().error(
                "Nav2 lifecycle nodes are not active: " + ", ".join(failed_nodes)
            )
            return False

        self.get_logger().info("Nav2 lifecycle nodes are active")
        return True

    def go_to_pose(self, x, y, yaw, name="goal"):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(x, y, yaw)

        self.get_logger().info(
            f"Waiting for /navigate_to_pose action server..."
        )

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/navigate_to_pose action server not available")
            return False

        self.get_logger().info(
            f"Sending {name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{name} was rejected")
            return False

        self.get_logger().info(f"{name} accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        status = result_future.result().status

        # status 4 = succeeded
        if status == 4:
            self.get_logger().info(f"{name} succeeded")
            return True

        self.get_logger().error(
            f"{name} failed with status: {status}, "
            f"error_code: {result.error_code}, error_msg: {result.error_msg}"
        )
        return False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        try:
            now = time.time()
            if now - self.last_feedback_time < 1.0:
                return

            distance = feedback.distance_remaining
            self.get_logger().info(f"남은 거리: {distance:.2f} m")
            self.last_feedback_time = now
        except Exception:
            pass


def main():
    rclpy.init()

    node = TwoPointMission()

    yaw_1_to_2 = calc_yaw_from_to(
        POINT_1_X,
        POINT_1_Y,
        POINT_2_X,
        POINT_2_Y,
    )

    yaw_2_to_1 = calc_yaw_from_to(
        POINT_2_X,
        POINT_2_Y,
        POINT_1_X,
        POINT_1_Y,
    )

    print("=== Two Point Test Mission ===")
    print(f"point_1: x={POINT_1_X:.3f}, y={POINT_1_Y:.3f}")
    print(f"point_2: x={POINT_2_X:.3f}, y={POINT_2_Y:.3f}")
    print(f"yaw 1 -> 2: {yaw_1_to_2:.3f} rad")
    print(f"yaw 2 -> 1: {yaw_2_to_1:.3f} rad")
    print()

    # 1. 처음 pose estimate
    # 실제 로봇을 1번 포인트에 두고, 2번 방향을 바라본다고 AMCL에 알려줌
    if not node.publish_initial_pose(
        POINT_1_X,
        POINT_1_Y,
        yaw_1_to_2,
    ):
        node.destroy_node()
        rclpy.shutdown()
        return

    # 2. RViz Navigation 패널을 수동으로 켰을 때처럼 Nav2 lifecycle을 활성화
    if not node.ensure_navigation_active():
        node.destroy_node()
        rclpy.shutdown()
        return

    if not node.wait_for_localization_stable(timeout_sec=3.0):
        node.destroy_node()
        rclpy.shutdown()
        return

    # 3. 1번 -> 2번
    # 2번에 도착하면 1번을 바라보는 방향으로 정렬
    ok = node.go_to_pose(
        POINT_2_X,
        POINT_2_Y,
        yaw_2_to_1,
        name="point_1_to_point_2",
    )

    if not ok:
        node.destroy_node()
        rclpy.shutdown()
        return

    if not node.wait_for_localization_stable(timeout_sec=3.0):
        node.destroy_node()
        rclpy.shutdown()
        return

    # 4. 2번 -> 1번
    # 1번에 도착하면 다시 2번을 바라보는 방향으로 정렬
    ok = node.go_to_pose(
        POINT_1_X,
        POINT_1_Y,
        yaw_1_to_2,
        name="point_2_to_point_1",
    )

    if ok:
        node.get_logger().info("왕복 테스트 완료")
        node.get_logger().info("최종 자세: 1번 포인트에서 2번 포인트를 바라보는 방향")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
