import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def angular_distance(angle, center):
    """Return the shortest signed angular distance in radians."""
    return math.atan2(math.sin(angle - center), math.cos(angle - center))


def angle_is_in_sector(angle, center, half_angle):
    return abs(angular_distance(angle, center)) <= half_angle + 1.0e-9


class FrontScanFilter(Node):
    def __init__(self):
        super().__init__("front_scan_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_front_filtered")
        self.declare_parameter("front_center_angle_deg", 180.0)
        self.declare_parameter("front_half_angle_deg", 120.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        center_deg = float(self.get_parameter("front_center_angle_deg").value)
        half_angle_deg = float(self.get_parameter("front_half_angle_deg").value)

        if not 0.0 < half_angle_deg <= 180.0:
            raise ValueError("front_half_angle_deg must be in (0, 180]")

        self.front_center_angle = math.radians(center_deg)
        self.front_half_angle = math.radians(half_angle_deg)
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Filtering LaserScan from "
            f"{input_topic} to {output_topic}: "
            f"center={center_deg:.1f} deg, half_angle={half_angle_deg:.1f} deg"
        )

    def scan_callback(self, msg):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = list(msg.ranges)
        filtered.intensities = list(msg.intensities)

        angle = msg.angle_min
        for index in range(len(filtered.ranges)):
            if not angle_is_in_sector(
                angle,
                self.front_center_angle,
                self.front_half_angle,
            ):
                filtered.ranges[index] = math.nan
            angle += msg.angle_increment

        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = FrontScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
