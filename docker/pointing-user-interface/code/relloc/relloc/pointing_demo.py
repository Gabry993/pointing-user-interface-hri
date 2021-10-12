from typing import Any
import chime
try:
    from pynput import keyboard
except ImportError:
    keyboard = None

import rclpy
import rclpy.node
from std_srvs.srv import Empty
from std_msgs.msg import Bool, ColorRGBA
from conveyor_msgs.msg import State, PUISubscription
from conveyor_utils.utils import LEDs
import time


class DemoSupervisor(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(DemoSupervisor, self).__init__("supervisor")
        path = self.declare_parameter('map_path', '').value
        self.sim_imu = self.declare_parameter("sim_imu", True).value
        self.period = 1/self.declare_parameter("update_rate", 10.).value
        self.do_relloc = self.declare_parameter("do_relloc", False).value
        self.use_state_led = self.declare_parameter("use_state_led", False).value
        self.sub_msg = PUISubscription()
        self.sub_msg.user_name = self.declare_parameter("user_name", "").value
        self.sub_msg.pointer_size = self.declare_parameter("pointer_size", 0.2).value
        color = self.declare_parameter("pointer_color", [1., 1., 0.]).value
        self.sub_msg.pointer_color = ColorRGBA(**{"r": color[0], "g": color[1], "b": color[2], "a": 1.0})
        self.sub_msg.pointer_cmap = self.declare_parameter("pointer_cmap", "").value
        self.set_yaw_origin_cli = self.create_client(Empty, "metawear_ros/set_yaw_origin")
        if not self.sim_imu:
            self.set_yaw_origin_cli.wait_for_service()

        qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            State, "state_transition", self.state_cb, qos
        )
        self.state_pub = self.create_publisher(
            State, "state_transition", qos
        )
        self.create_subscription(Bool, 'metawear_ros/button', self.button_cb, qos)
        self.create_subscription(Bool, '/joy/go_on', self.button_cb, 1)

        self.user_subscription_pub = self.create_publisher(PUISubscription, "/PUI_subscription", 10)

        if keyboard:
            self.key_listener = keyboard.Listener(on_press=self.key_cb)
            self.key_listener.start()

        leds = LEDs.load_file(path).leds
        self.get_logger().info(str(leds))
        if self.use_state_led:
            uid = max(list(leds))
            self.led_state_pub = self.create_publisher(ColorRGBA, "/" + uid + "/color", 1)
        self.relloc_color = ColorRGBA(**{"r": 1.0, "g": 1.0, "b": 0.0, "a": 1.0})
        self.wait_color = ColorRGBA(**{"r": 0.0, "g": 0.0, "b": 1.0, "a": 1.0})
        self.point_color = ColorRGBA(**{"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0})
        self.off_color = ColorRGBA(**{"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0})

        for uid in leds:
            pub = self.create_publisher(ColorRGBA, "/" + uid + "/color", 1)
            pub.publish(self.off_color)
            time.sleep(0.1)
            pub.publish(self.off_color)

        self.state = "ready"
        self.current_run = 0
        self.timer = self.create_timer(self.period, self.update)

        self.get_logger().info(f"State: {self.state}")
        if self.use_state_led:
            self.led_state_pub.publish(self.wait_color)
            time.sleep(0.1)
            self.led_state_pub.publish(self.wait_color)

        chime.theme("big-sur")

    def state_cb(self, msg):
        if msg.node_name == self.get_name():
            self.change_to_state(msg.state)

    def change_to_state(self, new_state):
        self.get_logger().info(f"State transition {self.state} -> {new_state}")
        self.state = new_state

    def button_cb(self, msg: Bool) -> None:
        if msg.data is True:
            self.go_on()

    def key_cb(self, key):
        try:
            value = key.name
        except AttributeError:
            value = key.char
        if value == 'n':
            self.go_on()

    def go_on(self):
        if self.state == "ready":
            self.change_to_state("start_relloc")
        elif self.state == "pointing":
            self.change_to_state("ready")
            # state = State(node_name="pui_node", state="ready")
            # self.state_pub.publish(state)
            self.sub_msg.subscribe = False
            self.user_subscription_pub.publish(self.sub_msg)
            if self.use_state_led:
                self.led_state_pub.publish(self.wait_color)
                time.sleep(0.1)
                self.led_state_pub.publish(self.wait_color)

    def update(self):
        if self.state == "start_relloc":
            if self.use_state_led:
                self.led_state_pub.publish(self.relloc_color)
            if self.do_relloc:
                state = State(node_name="relloc_executor", state="start_relloc")
                self.state_pub.publish(state)
                self.change_to_state("waiting_relloc")
                state = State(node_name="single_pui_node", state="reset")
                self.state_pub.publish(state)
            else:
                if not self.sim_imu:
                    self.set_yaw_origin_cli.call_async(Empty.Request())
                    self.get_logger().info("yaw reset")
                state = State(node_name="relloc_node", state="gt_localize")
                self.state_pub.publish(state)
                self.change_to_state("waiting_relloc")
                state = State(node_name="single_pui_node", state="reset")
                self.state_pub.publish(state)
        elif self.state == "relloc_done":
            self.change_to_state("pointing")
            # state = State(node_name="pui_node", state="pointing")
            # self.state_pub.publish(state)
            self.sub_msg.subscribe = True
            self.user_subscription_pub.publish(self.sub_msg)
            if self.use_state_led:
                self.led_state_pub.publish(self.point_color)
                time.sleep(0.1)
                self.led_state_pub.publish(self.point_color)
        elif self.state == "ready":
            if self.use_state_led:
                self.led_state_pub.publish(self.wait_color)
                time.sleep(0.1)
                self.led_state_pub.publish(self.wait_color)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DemoSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.sub_msg.subscribe = False
        node.user_subscription_pub.publish(node.sub_msg)
    node.destroy_node()
    rclpy.shutdown()
