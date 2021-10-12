from typing import Any
import chime
import time
try:
    from pynput import keyboard
except ImportError:
    keyboard = None

import rclpy
import rclpy.node
from std_srvs.srv import Empty
from std_msgs.msg import Int32, Bool, ColorRGBA
from conveyor_utils.utils import LEDs
from conveyor_msgs.msg import State
chime.theme("big-sur")


class ExpSupervisor(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(ExpSupervisor, self).__init__("supervisor")
        path = self.declare_parameter('map_path', '').value
        self.n_runs = self.declare_parameter("n_runs", 5).value
        self.period = 1/self.declare_parameter("update_rate", 10.).value
        self.sim_imu = self.declare_parameter("sim_imu", True).value
        self.do_relloc = self.declare_parameter("do_relloc", False).value

        self.generate_cli = self.create_client(Empty, 'generate_packs')
        self.reset_cli = self.create_client(Empty, 'reset_pack_sim')
        self.set_yaw_origin_cli = self.create_client(Empty, '/metawear_ros/set_yaw_origin')
        self.generate_cli.wait_for_service()
        if not self.sim_imu:
            self.set_yaw_origin_cli.wait_for_service()

        qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.run_n_pub = self.create_publisher(Int32, "/run_number", qos)
        self.create_subscription(
            State, "/state_transition", self.state_cb, qos
        )
        self.state_pub = self.create_publisher(
            State, "/state_transition", qos
        )
        self.create_subscription(Bool, '/metawear_ros/button', self.button_cb, qos)
        self.create_subscription(Bool, '/joy/go_on', self.button_cb, 1)

        leds = LEDs.load_file(path).leds

        for uid in leds:
            pub = self.create_publisher(ColorRGBA, uid + "/color", 1)
            pub.publish(ColorRGBA(**{"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0}))
            time.sleep(0.1)
            pub.publish(ColorRGBA(**{"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0}))

        if keyboard:
            self.key_listener = keyboard.Listener(on_press=self.key_cb)
            self.key_listener.start()

        self.state: str = "ready"
        self.current_run: int = 0
        self.timer = self.create_timer(self.period, self.update)

        self.get_logger().info(f"State: {self.state}")

    def state_cb(self, msg: State) -> None:
        if msg.node_name == self.get_name():
            self.change_to_state(msg.state)

    def change_to_state(self, new_state: str) -> None:
        self.get_logger().info(f"State transition {self.state} -> {new_state}")
        self.state = new_state

    def button_cb(self, msg: Bool) -> None:
        if msg.data:
            self.go_on()

    def key_cb(self, key) -> None:
        try:
            value = key.name
        except AttributeError:
            value = key.char
        if value == 'n':
            self.go_on()

    def go_on(self) -> None:
        if self.state == "ready":
            self.change_to_state("start_relloc")
        elif self.state == "relloc_done":
            self.change_to_state("ready_to_start")
            state = State(node_name="pui_node", state="pointing")
            self.state_pub.publish(state)
        elif self.current_run < self.n_runs and self.state in ["ready_to_start"]:
            self.change_to_state("start")
            self.run_n_pub.publish(Int32(data=self.current_run))
            self.current_run += 1
        elif self.state == "packs_generated":
            self.change_to_state("ready")
            state = State(node_name="pui_node", state="ready")
            self.state_pub.publish(state)
            self.reset_cli.call_async(Empty.Request())
        elif self.current_run >= self.n_runs:
            self.get_logger().info(f"Experiment over ({self.n_runs} runs)")

    def update(self) -> None:
        if self.state == "start_relloc":
            if self.do_relloc:
                state = State(node_name="relloc_executor", state="start_relloc")
                self.state_pub.publish(state)
                self.change_to_state("waiting_relloc")
            else:
                if not self.sim_imu and self.current_run < 1:
                    self.set_yaw_origin_cli.call_async(Empty.Request())
                    self.get_logger().info("yaw reset")
                state = State(node_name="relloc_node", state="gt_localize")
                self.state_pub.publish(state)
                self.change_to_state("waiting_relloc")
        elif self.state == "ready_to_start":
            self.go_on()
        elif self.state == "start":
            chime.warning()
            self.reset_cli.call_async(Empty.Request())
            time.sleep(5)
            self.generate_cli.call_async(Empty.Request())
            chime.warning()
            self.change_to_state("packs_generated")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ExpSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
