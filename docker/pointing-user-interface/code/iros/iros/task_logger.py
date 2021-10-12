import datetime
from typing import Any, Dict, List

import rclpy
import rclpy.node

from conveyor_msgs.msg import PackageList, PackageUIDList
from std_msgs.msg import Int32


def format_nanos(nanos: int, prec: str = 'nano') -> str:
    dt = datetime.datetime.fromtimestamp(nanos / 1e9)
    if prec == 'nano':
        return '{}{:03.0f}'.format(dt.strftime('%Y-%m-%dT%H:%M:%S.%f'), nanos % 1e3)
    else:  # seconds
        return dt.strftime('%Y-%m-%dT%H:%M:%S')


class TaskLogger(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(TaskLogger, self).__init__("task_logger")

        self.period = 1/self.declare_parameter("log_rate", 10.).value
        log_path = self.declare_parameter("log_path", "/home/gabri/Downloads/").value
        ts = format_nanos(self.get_clock().now().nanoseconds, prec='sec')
        exp_name = self.declare_parameter("exp_name", "").value
        self.out_file = log_path + ts + "_" + exp_name + ".txt"
        self.packages: Dict[int, Dict[str, Any]] = {}
        self.selected_uids: List[int] = []
        self.run_n = 0
        self.create_subscription(PackageList, 'packages', self.update_packages, 1)
        qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PackageUIDList, 'selected_packages', self.update_selected, qos)

        self.global_state_sub = self.create_subscription(
            Int32, "/run_number", self.run_n_cb, qos
        )
        other_pars = [
            "map_path",
            "belt_speed",
            "package_length",
            "led_rate",
            "n_packs",
            "n_reds",
            "n_runs",
            "interface",
            "update_rate",
        ]
        self.other_par_dict = {par: self.declare_parameter(par).value for par in other_pars}
        self.log_timer = self.create_timer(self.period, self.log)
        with open(self.out_file, 'a+') as f:
            f.write(f"{self.other_par_dict}\n")
        self.get_logger().info("Ready")

    def run_n_cb(self, msg: Int32) -> None:
        self.run_n = msg.data

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
        with open(self.out_file, 'a+') as f:
            f.write(f"{ts}, {self.run_n}, {self.packages}\n")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = TaskLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
