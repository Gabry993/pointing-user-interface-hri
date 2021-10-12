from typing import Any, Dict, List

import rclpy
import rclpy.node

from conveyor_msgs.msg import PackageList, PackageUIDList
from std_msgs.msg import Int32
import yaml
from .task_logger import format_nanos


class MultiTaskLogger(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(MultiTaskLogger, self).__init__("multi_task_logger")

        self.period = 1/self.declare_parameter("log_rate", 10.).value
        log_path = self.declare_parameter("log_path", "/home/gabri/Downloads/").value
        conf_path = self.declare_parameter("step_conf", "").value
        with open(conf_path, 'r') as f:
            self.steps = yaml.safe_load(f)['steps']

        ts = format_nanos(self.get_clock().now().nanoseconds, prec='sec')
        exp_name = self.declare_parameter("exp_name", "").value
        self.out_file = log_path + ts + "_" + exp_name + ".txt"
        self.packages: Dict[int, Dict[str, Any]] = {}
        self.selected_uids: List[int] = []

        self.step_n = None

        self.create_subscription(PackageList, 'packages', self.update_packages, 1)
        qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PackageUIDList, 'selected_packages', self.update_selected, qos)

        self.create_subscription(
            Int32, "/step_number", self.step_number_cb, qos
        )
        other_pars = [
            "map_path",
            "led_rate",
            "interface",
            "update_rate",
        ]
        self.other_par_dict = {par: self.declare_parameter(par).value for par in other_pars}
        self.other_par_dict['steps_conf'] = self.steps

        self.log_timer = self.create_timer(self.period, self.log)
        with open(self.out_file, 'a+') as f:
            f.write(f"{self.other_par_dict}\n")
        self.get_logger().info("Ready")

    def step_number_cb(self, msg: Int32) -> None:
        self.step_n = msg.data

    def update_packages(self, msg: PackageList) -> None:
        self.packages = {}
        for package in msg.packages:
            self.packages[package.uid] = {"uid": package.uid,
                                          "position": (package.position.name, package.position.position),
                                          "type": package.type,
                                          "length": package.length,
                                          "selected": True if package.uid in self.selected_uids else False,
                                          }

    def update_selected(self, msg: PackageUIDList) -> None:
        self.selected_uids = []
        for uid in msg.uids:
            self.selected_uids.append(uid)

    def log(self) -> None:
        ts = format_nanos(self.get_clock().now().nanoseconds, prec='nano')
        log_row_dict = {"timestamp": ts, "step_n": self.step_n, "packages": self.packages}
        with open(self.out_file, 'a+') as f:
            f.write(f"{log_row_dict}\n")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = MultiTaskLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
