#!/usr/bin/env python3

import threading
import time

from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Nav2Supervisor(Node):
    """Serializes lifecycle operations for the two Nav2 lifecycle managers."""

    def __init__(self):
        super().__init__(
            "nav2_supervisor",
            automatically_declare_parameters_from_overrides=True,
        )

        self.declare_parameter_if_missing("status_topic", "/nav2_supervisor/status")
        self.declare_parameter_if_missing("service_timeout_sec", 20.0)
        self.declare_parameter_if_missing("active_wait_timeout_sec", 10.0)

        self.callback_group = ReentrantCallbackGroup()
        self.transition_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.current_status = "IDLE"
        self.manager_states = {
            "localization": "unconfigured",
            "navigation": "unconfigured",
        }

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            status_qos,
        )

        self.manager_clients = {
            "localization": self.create_client(
                ManageLifecycleNodes,
                "/lifecycle_manager_localization/manage_nodes",
                callback_group=self.callback_group,
            ),
            "navigation": self.create_client(
                ManageLifecycleNodes,
                "/lifecycle_manager_navigation/manage_nodes",
                callback_group=self.callback_group,
            ),
        }
        self.active_clients = {
            "localization": self.create_client(
                Trigger,
                "/lifecycle_manager_localization/is_active",
                callback_group=self.callback_group,
            ),
            "navigation": self.create_client(
                Trigger,
                "/lifecycle_manager_navigation/is_active",
                callback_group=self.callback_group,
            ),
        }
        self.state_clients = {
            "localization": self.create_client(
                GetState,
                "/map_server/get_state",
                callback_group=self.callback_group,
            ),
            "navigation": self.create_client(
                GetState,
                "/controller_server/get_state",
                callback_group=self.callback_group,
            ),
        }

        self.create_service(
            Trigger,
            "/nav2_supervisor/start_localization",
            self.start_localization_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/nav2_supervisor/start_navigation",
            self.start_navigation_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/nav2_supervisor/pause_navigation",
            self.pause_navigation_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/nav2_supervisor/reset_all",
            self.reset_all_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/nav2_supervisor/status",
            self.status_callback,
            callback_group=self.callback_group,
        )

        self.publish_status("IDLE")
        self.get_logger().info("Nav2 lifecycle supervisor is ready")

    def declare_parameter_if_missing(self, name, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def start_localization_callback(self, request, response):
        del request
        with self.transition_lock:
            success = self.ensure_manager_active("localization")
        return self.fill_response(
            response,
            success,
            "localization active" if success else "failed to start localization",
        )

    def start_navigation_callback(self, request, response):
        del request
        with self.transition_lock:
            success = self.ensure_manager_active("navigation")
        return self.fill_response(
            response,
            success,
            "navigation active" if success else "failed to start navigation",
        )

    def pause_navigation_callback(self, request, response):
        del request
        with self.transition_lock:
            activity = self.get_manager_activity("navigation")
            if activity is None:
                success = False
                message = "navigation lifecycle state unavailable"
                self.publish_status("ERROR")
            elif not activity:
                success = True
                message = "navigation already inactive"
            else:
                self.publish_status("PAUSING_NAVIGATION")
                success = self.send_manager_command(
                    "navigation",
                    ManageLifecycleNodes.Request.PAUSE,
                )
                if success:
                    self.manager_states["navigation"] = "paused"
                message = (
                    "navigation paused" if success else "failed to pause navigation"
                )
                self.publish_status("NAVIGATION_PAUSED" if success else "ERROR")
        return self.fill_response(response, success, message)

    def reset_all_callback(self, request, response):
        del request
        with self.transition_lock:
            self.publish_status("RESETTING")
            navigation_ok = self.reset_manager("navigation")
            localization_ok = self.reset_manager("localization")
            success = navigation_ok and localization_ok
            self.publish_status("IDLE" if success else "ERROR")

        failed = []
        if not navigation_ok:
            failed.append("navigation")
        if not localization_ok:
            failed.append("localization")
        message = "Nav2 reset" if success else "reset failed: " + ", ".join(failed)
        return self.fill_response(response, success, message)

    def status_callback(self, request, response):
        del request
        return self.fill_response(response, True, self.current_status)

    def ensure_manager_active(self, manager):
        if self.manager_is_active(manager):
            self.manager_states[manager] = "active"
            self.publish_status(f"{manager.upper()}_ACTIVE")
            return True

        self.publish_status(f"STARTING_{manager.upper()}")
        lifecycle_state = self.get_manager_state(manager)

        if (
            lifecycle_state == "inactive"
            and self.send_manager_command(
                manager,
                ManageLifecycleNodes.Request.RESUME,
            )
        ):
            if self.wait_for_manager_active(manager):
                self.manager_states[manager] = "active"
                self.publish_status(f"{manager.upper()}_ACTIVE")
                return True

        if lifecycle_state == "inactive":
            self.send_manager_command(manager, ManageLifecycleNodes.Request.CLEANUP)
        elif lifecycle_state != "unconfigured":
            self.send_manager_command(manager, ManageLifecycleNodes.Request.RESET)
        if not self.send_manager_command(manager, ManageLifecycleNodes.Request.STARTUP):
            self.manager_states[manager] = "unknown"
            self.publish_status("ERROR")
            return False

        success = self.wait_for_manager_active(manager)
        self.manager_states[manager] = "active" if success else "unknown"
        self.publish_status(f"{manager.upper()}_ACTIVE" if success else "ERROR")
        return success

    def reset_manager(self, manager):
        client = self.manager_clients[manager]
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            self.get_logger().warn(f"{manager} lifecycle manager is unavailable")
            return False

        lifecycle_state = self.get_manager_state(manager)
        if lifecycle_state == "unconfigured":
            self.manager_states[manager] = "unconfigured"
            return True

        command = (
            ManageLifecycleNodes.Request.CLEANUP
            if lifecycle_state == "inactive"
            else ManageLifecycleNodes.Request.RESET
        )
        if self.send_manager_command(manager, command):
            self.manager_states[manager] = "unconfigured"
            return True
        if self.manager_states[manager] == "unconfigured":
            return True
        if not self.manager_is_active(manager):
            self.get_logger().info(f"{manager} is already inactive")
            self.manager_states[manager] = "unconfigured"
            return True
        return False

    def get_manager_state(self, manager):
        client = self.state_clients[manager]
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return "unknown"

        result = self.call_service(client, GetState.Request(), min(timeout, 5.0))
        if result is None:
            return "unknown"
        states = {
            1: "unconfigured",
            2: "inactive",
            3: "active",
        }
        return states.get(result.current_state.id, "unknown")

    def manager_is_active(self, manager):
        return self.get_manager_activity(manager) is True

    def get_manager_activity(self, manager):
        client = self.active_clients[manager]
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None

        result = self.call_service(client, Trigger.Request(), min(timeout, 5.0))
        if result is None:
            return None
        return bool(result.success)

    def wait_for_manager_active(self, manager):
        timeout = float(self.get_parameter("active_wait_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while not self.shutdown_event.is_set() and time.monotonic() < deadline:
            if self.manager_is_active(manager):
                return True
            time.sleep(0.2)
        return False

    def send_manager_command(self, manager, command):
        client = self.manager_clients[manager]
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"{manager} lifecycle manager is unavailable")
            return False

        request = ManageLifecycleNodes.Request()
        request.command = command
        result = self.call_service(client, request, timeout)
        if result is None or not result.success:
            self.get_logger().warn(
                f"Lifecycle command {command} failed for {manager}"
            )
            return False
        return True

    def call_service(self, client, request, timeout_sec):
        future = client.call_async(request)
        completed = threading.Event()
        result_holder = {}

        def done_callback(done_future):
            try:
                result_holder["result"] = done_future.result()
            except Exception as exc:  # rclpy service exceptions are runtime-specific
                result_holder["exception"] = exc
            completed.set()

        future.add_done_callback(done_callback)
        completed.wait(timeout=max(0.1, float(timeout_sec)))
        if not completed.is_set():
            self.get_logger().error("Lifecycle service call timed out")
            return None
        if "exception" in result_holder:
            self.get_logger().error(
                f"Lifecycle service call failed: {result_holder['exception']}"
            )
            return None
        return result_holder.get("result")

    def fill_response(self, response, success, message):
        response.success = bool(success)
        response.message = message
        return response

    def publish_status(self, status):
        self.current_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {status}")

    def destroy_node(self):
        self.shutdown_event.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = Nav2Supervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
