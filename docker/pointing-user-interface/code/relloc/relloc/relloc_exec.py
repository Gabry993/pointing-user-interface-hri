from typing import Any
import chime
import time

import rclpy
import rclpy.node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import ColorRGBA
from conveyor_msgs.msg import State
from conveyor_utils.utils import LEDs


class RellocExecutor(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(RellocExecutor, self).__init__("relloc_executor")
        path = self.declare_parameter('map_path', '').value
        duration = self.declare_parameter('led_on_period', 3.).value
        self.relloc_leds = self.declare_parameter('relloc_leds', ['led_0', 'led_1']).value
        self.on_duration = rclpy.duration.Duration(seconds=duration)

        qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            State, "state_transition", self.state_cb, qos
        )
        self.state_pub = self.create_publisher(
            State, "state_transition", qos
        )

        self.leds = LEDs.load_file(path).leds
        self.led_pubs = {uid: self.create_publisher(ColorRGBA, "/"+uid+"/color", 1) for uid in self.leds}
        self.on_color = ColorRGBA(**{"r": 1.0, "g": 0.0, "b": 1.0, "a": 1.0})
        self.off_color = ColorRGBA(**{"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0})

        self.target_pub = self.create_publisher(Vector3Stamped, "pointing_target", 100)
        self.state = "ready"

        self.get_logger().info(f"State: {self.state}")
        chime.theme("big-sur")

    def state_cb(self, msg):
        if msg.node_name == self.get_name():
            self.change_to_state(msg.state)
            self.exec()

    def change_to_state(self, new_state):
        self.get_logger().info(f"State transition {self.state} -> {new_state}")
        self.state = new_state

    def publish_target(self, point):
        target_msg = Vector3Stamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.vector.x = point[0]
        target_msg.vector.y = point[1]
        target_msg.vector.z = point[2]
        self.target_pub.publish(target_msg)

    def exec(self):
        if self.state == "start_relloc":
            for led_name in self.relloc_leds:
                self.led_pubs[led_name].publish(self.off_color)
            # wait 2 seconds
            self.change_to_state("waiting 2 sec")
            time.sleep(0.5)
            # turn on led 0 and alert relloc node
            self.change_to_state(f"turning on {self.relloc_leds[0]}")
            self.led_pubs[self.relloc_leds[0]].publish(self.on_color)
            time.sleep(0.1)
            self.led_pubs[self.relloc_leds[0]].publish(self.on_color)

            chime.error()
            target = self.leds[self.relloc_leds[0]]
            time.sleep(1)
            state = State(node_name="relloc_node", state="collect_data")
            self.state_pub.publish(state)
            timer = self.get_clock().now()

            while self.get_clock().now() - timer <= self.on_duration:
                self.publish_target(target)

            # turn off led 0 and wait alert relloc node
            self.change_to_state(f"turning off {self.relloc_leds[0]}")
            self.led_pubs[self.relloc_leds[0]].publish(self.off_color)
            time.sleep(0.1)
            self.led_pubs[self.relloc_leds[0]].publish(self.off_color)
            state = State(node_name="relloc_node", state="wait")
            self.state_pub.publish(state)

            # turn on led 1 and wait half second before alerting relloc node
            self.change_to_state(f"turning on {self.relloc_leds[1]}")
            self.led_pubs[self.relloc_leds[1]].publish(self.on_color)
            time.sleep(0.1)
            self.led_pubs[self.relloc_leds[1]].publish(self.on_color)

            chime.error()
            target = self.leds[self.relloc_leds[1]]
            time.sleep(1)
            state = State(node_name="relloc_node", state="collect_data")
            self.state_pub.publish(state)
            timer = self.get_clock().now()
            while self.get_clock().now() - timer <= self.on_duration:
                self.publish_target(target)

            # turn off led 1 and tell relloc_node to compute relloc
            self.change_to_state("relloc_executed")
            state = State(node_name="relloc_node", state="compute_relloc")
            self.state_pub.publish(state)
            self.led_pubs[self.relloc_leds[1]].publish(self.off_color)
            time.sleep(0.1)
            self.led_pubs[self.relloc_leds[1]].publish(self.off_color)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = RellocExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
