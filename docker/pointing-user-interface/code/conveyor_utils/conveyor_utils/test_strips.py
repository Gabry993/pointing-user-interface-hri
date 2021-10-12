from typing import Any, List
from itertools import groupby
import time

import rclpy
import rclpy.node
import led_strip_msgs.msg
import random

from .utils import Map

import numpy as np


def color() -> List[int]:
    return [random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)]


class TestStripsNode(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(TestStripsNode, self).__init__("test_strips")
        self.pub = self.create_publisher(led_strip_msgs.msg.LedStrips, 'led_strips', 1)
        # self.timer = self.create_timer(0.1, self.update)
        path = self.declare_parameter('map_path', '/Users/Jerome/Dev/My/oneswarm-hri/map.yaml').value
        self.map_ = Map.load_file(path)
        self.p = 0.0
        self.step = 0.02
        self.reset()
        self.begin()
        self.get_logger().info("Ready")

    def update(self) -> None:
        self.pub.publish(self.interval('part_g', self.p))
        self.p += self.step
        if(self.p > 1 or self.p < 0):
            self.step *= -1

    def fill(self, color: List[int]) -> led_strip_msgs.msg.LedStrips:
        msg = led_strip_msgs.msg.LedStrips()
        for strip in self.map_.strips:
            strip_msg = led_strip_msgs.msg.LedStrip(
                id=strip.uid, data=strip.draw(color, [(0, 1)]))
            msg.strips.append(strip_msg)
        return msg

    def interval(self, belt_name: str, position: float) -> led_strip_msgs.msg.LedStrips:
        rmsg = led_strip_msgs.msg.LedStrips()
        rs = self.map_.strips_near(belt_name, position, width=0.5)
        ds = {k: list(v) for k, v in groupby(sorted(rs), key=lambda x: x[0])}
        # for strip in ds.keys():
        #     vs = ds.get(strip, [])
        #     strip_msg = led_strip_msgs.msg.LedStrip(
        #         id=strip.uid, data=strip.draw([255, 0, 0], [r for _, r in vs]))
        #     rmsg.strips.append(strip_msg)
        # return rmsg
        for strip in self.map_.strips:
            vs = ds.get(strip, [])
            strip_msg = led_strip_msgs.msg.LedStrip(
                id=strip.uid, data=strip.draw([255, 0, 0], [r for _, r in vs]))
            rmsg.strips.append(strip_msg)
        return rmsg

    def begin(self) -> None:
        for strip in self.map_.strips:
            data = (np.array(strip.draw([255, 0, 0], [(0, 0.1)])) +
                    np.array(strip.draw([0, 255, 0], [(0.1, 0.2)])) +
                    np.array(strip.draw([0, 255, 0], [(0.8, 0.9)])) +
                    np.array(strip.draw([0, 0, 255], [(0.9, 1.0)])))
            data = list(data)
            strip_msg = led_strip_msgs.msg.LedStrip(
                id=strip.uid, data=data)
            msg = led_strip_msgs.msg.LedStrips(strips=[strip_msg])
            self.pub.publish(msg)
            time.sleep(0.05)

    def random_colors(self) -> led_strip_msgs.msg.LedStrips:
        rmsg = led_strip_msgs.msg.LedStrips()
        for strip in self.map_.strips:
            strip_msg = led_strip_msgs.msg.LedStrip(
                id=strip.uid, data=strip.draw(color(), [(0, 1)]))
            rmsg.strips.append(strip_msg)
        return rmsg

    def reset(self) -> None:
        for strip in self.map_.strips:
            strip_msg = led_strip_msgs.msg.LedStrip(
                id=strip.uid, data=strip.draw([0, 0, 0], [(0, 1)]))
            msg = led_strip_msgs.msg.LedStrips(strips=[strip_msg])
            self.pub.publish(msg)
            time.sleep(0.05)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = TestStripsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
