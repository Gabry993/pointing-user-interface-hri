from typing import Any, List, Iterator, Tuple
from itertools import cycle

import rclpy
import rclpy.node
import conveyor_msgs.msg

from .utils import Map, Belt
from dataclasses import dataclass


@dataclass
class Package:
    uid: int
    kind: int
    position: float  # m
    length: float
    belt: Belt

    @property
    def msg(self) -> conveyor_msgs.msg.Package:
        m = conveyor_msgs.msg.Package()
        m.uid = self.uid
        m.type = self.kind
        m.position.position = self.position / self.belt.length
        m.position.name = self.belt.uid
        m.length = self.length
        return m


class DummyTracker(rclpy.node.Node):  # type: ignore

    # type, gap before
    PACKAGES: List[int] = [0, 1]

    def __init__(self) -> None:
        super(DummyTracker, self).__init__("dummy_tracker")
        self.pub = self.create_publisher(conveyor_msgs.msg.PackageList, 'packages', 1)
        path = self.declare_parameter('map_path', '').value
        self.length = self.declare_parameter('package_length', 0.5).value
        self.feeder_period = self.declare_parameter('period', 10.0).value
        self.map_ = Map.load_file(path)
        self.packages: List[Package] = []
        self.speed = 0.25  # m/s
        self.period = 0.1
        self.time = 0.0
        self.uid = 0
        self.timer = self.create_timer(self.period, self.update)
        self.feeder = self.package_feeder(list(self.map_.belts)[0])
        self.package, _ = next(self.feeder)
        self.deadline = 0.0
        self.get_logger().info("Ready")

    def package_feeder(self, belt: Belt) -> Iterator[Tuple[Package, float]]:
        for kind in cycle(self.PACKAGES):
            package = Package(uid=self.uid, kind=kind, position=0.0, belt=belt,
                              length=self.length)
            self.uid += 1
            yield package, self.feeder_period + self.time

    def update(self) -> None:
        self.time += self.period
        if self.deadline < self.time:
            self.packages.append(self.package)
            self.package, self.deadline = next(self.feeder)
        for p in self.packages[:]:
            p.position += self.speed * self.period
            if p.position > p.belt.length:
                belt = self.map_.next_belt(p.belt)
                if not belt:
                    self.packages.remove(p)
                else:
                    p.position -= p.belt.length
                    p.belt = belt
        msg = conveyor_msgs.msg.PackageList(packages=[p.msg for p in self.packages])
        self.pub.publish(msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DummyTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
