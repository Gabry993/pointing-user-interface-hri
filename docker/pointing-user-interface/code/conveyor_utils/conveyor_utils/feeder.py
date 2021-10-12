from typing import Any

import rclpy
import rclpy.node
from std_msgs.msg import UInt8, Bool
from itertools import cycle


class Feeder(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(Feeder, self).__init__("feeder")

        self.feed = self.declare_parameter("feed", True).value
        self.kind_seq = cycle(self.declare_parameter("kind_sequence", [0, 1, 2]).value)
        self.min_interval = rclpy.duration.Duration(seconds=self.declare_parameter("min_interval", 1.).value)
        self.last_time = self.get_clock().now()

        self.create_subscription(Bool, "/feed", self.feed_cb, 1)
        self.pack_pub = self.create_publisher(UInt8, "/add_package", 1)
        self.create_timer(1/60., self.run_feeder)

        self.get_logger().info("Ready")

    def run_feeder(self):
        if self.feed:
            if self.get_clock().now() - self.last_time > self.min_interval:
                self.pack_pub.publish(UInt8(data=next(self.kind_seq)))
                self.last_time = self.get_clock().now()

    def feed_cb(self, msg):
        self.feed = msg.data


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Feeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
