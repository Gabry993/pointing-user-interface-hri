from typing import Any, List, Tuple

import rclpy
import rclpy.node
import conveyor_msgs.msg
from conveyor_msgs.srv import AddPack, SetBeltSpeed
from std_srvs.srv import Empty
from std_msgs.msg import UInt8
from .utils import Map, Belt, PositionOnBelt
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


class BeltControl(rclpy.node.Node):  # type: ignore

    # type, gap before
    PACKAGES: List[int] = [0, 1]

    def __init__(self) -> None:
        super(BeltControl, self).__init__("belt_control")
        self.pub = self.create_publisher(conveyor_msgs.msg.PackageList, 'packages', 1)
        path = self.declare_parameter('map_path', '').value
        self.length = self.declare_parameter('package_length', 0.5).value
        self.period = 1/self.declare_parameter('led_rate', 10.).value
        self.speed = self.declare_parameter('belt_speed', 0.25).value  # m/s

        # services
        self.add_pack_srv = self.create_service(AddPack, 'add_pack', self.add_pack_cb)
        self.set_belt_speed_srv = self.create_service(SetBeltSpeed, 'set_belt_speed', self.set_belt_speed_cb)
        self.reset_srv = self.create_service(Empty, 'reset_pack_sim', self.reset_cb)

        # replicates coppelia interface for feeder
        self.create_subscription(conveyor_msgs.msg.BeltCommand, "/belt_cmd", self.belt_cmd_cb, 1)
        self.create_subscription(UInt8, "/add_package", self.add_package_cb, 1)

        self.map_ = Map.load_file(path)
        self.packages: List[Package] = []
        self.time: float = 0
        self.uid: int = 0
        self.timer = self.create_timer(self.period, self.update)
        self.get_logger().info("Ready")

    def belt_cmd_cb(self, msg):
        if msg.name[0] == "*":
            self.speed = msg.velocity[0]

    def add_package_cb(self, msg):
        belt: Belt = list(self.map_.belts)[0]
        uid: int = self.uid
        self.uid += 1
        position: float = 0.0
        if self.valid_uid(uid) and self.check_space(belt, position):
            package = Package(uid=uid, kind=msg.data,
                              position=position*belt.length,
                              belt=belt,
                              length=self.length)
            self.packages.append(package)

    def set_belt_speed_cb(self,
                          request: SetBeltSpeed.Request,
                          response: SetBeltSpeed.Response) -> SetBeltSpeed.Response:
        self.speed = request.speed
        return response

    def reset_cb(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self.packages = []
        return response

    def add_pack_cb(self, request: AddPack.Request, response: AddPack.Response) -> AddPack.Response:
        belt: Belt = list(self.map_.belts)[request.belt]
        if self.valid_uid(request.uid) and self.check_space(belt,
                                                            request.position):
            package = Package(uid=request.uid, kind=request.kind,
                              position=request.position*belt.length,
                              belt=belt,
                              length=self.length)
            self.packages.append(package)
            response.result = True
        else:
            response.result = False
            self.get_logger().debug("pack not added")
        return response

    def valid_uid(self, uid: int) -> bool:
        for package in self.packages:
            if package.uid == uid:
                self.get_logger().debug("Not valid uid")
                return False
        return True

    def check_space(self, belt: Belt, position: float) -> bool:
        intervals: Tuple[PositionOnBelt, PositionOnBelt] = self.map_.interval(belt, position,
                                                                              self.length + self.length/2)
        start, end = intervals
        if start[0] is end[0]:  # only one belt
            for package in self.packages:
                pos = package.position/package.belt.length
                if start[0] is package.belt:
                    if start[1] <= pos <= end[1]:
                        return False
        else:  # two belts
            for package in self.packages:
                pos = package.position/package.belt.length
                if ((start[0] is package.belt and start[1] <= pos)
                   or (end[0] is package.belt and end[1] >= pos)):
                    return False
        return True

    def update(self) -> None:
        self.time += self.period
        for p in self.packages[:]:
            p.position += self.speed * self.period
            if p.position > p.belt.length:
                belt = self.map_.next_belt(p.belt)
                if not belt:
                    self.packages.remove(p)
                else:
                    p.position -= p.belt.length
                    p.belt = belt
            elif p.position < 0:
                belt = self.map_.previous_belt(p.belt)
                if not belt:
                    self.packages.remove(p)
                else:
                    p.position += p.belt.length
                    p.belt = belt
        msg = conveyor_msgs.msg.PackageList(packages=[p.msg for p in self.packages])
        self.pub.publish(msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = BeltControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
