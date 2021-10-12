from typing import Any

import rclpy
import rclpy.node
import yaml
from std_msgs.msg import UInt8, Bool, Int32
import conveyor_msgs.msg
from itertools import cycle
import numpy as np


class MultiFeeder(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(MultiFeeder, self).__init__("multi_feeder")

        conf_path = self.declare_parameter("step_conf", "").value
        self.feed = self.declare_parameter("feed", True).value

        with open(conf_path, 'r') as f:
            self.steps = yaml.safe_load(f)['steps']

        self.feeders_pub = {}
        for step, step_conf in self.steps.items():
            step_conf['duration'] = rclpy.duration.Duration(seconds=step_conf['duration'])
            for feeder, conf in step_conf['feeders'].items():
                if not conf['random']:
                    conf["kind_sequence"] = cycle(conf["kind_sequence"])
                if feeder not in self.feeders_pub:
                    self.feeders_pub[feeder] = self.create_publisher(UInt8, f"/add_package_{feeder}", 1)
                conf['timer'] = self.get_clock().now()
                duration = np.random.uniform(conf['min_interval'],
                                             conf['max_interval'])
                conf['duration'] = rclpy.duration.Duration(seconds=duration)

        self.steps_list = iter(self.steps.keys())
        self.current_step = list(self.steps.keys())[0]
        self.step_timer = None
        self.create_subscription(Bool, "/feed", self.feed_cb, 1)
        self.speed_pub = self.create_publisher(conveyor_msgs.msg.BeltCommand, "/belt_cmd", 1)
        qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.step_pub = self.create_publisher(Int32, "/step_number", qos)
        self.next_step()
        self.create_timer(1/60., self.run_feeders)
        self.get_logger().info("Ready")

    def next_step(self):
        try:
            self.current_step = next(self.steps_list)
            speed_msg = conveyor_msgs.msg.BeltCommand()
            speed_msg.name = ["*"]
            speed_msg.velocity = [self.steps[self.current_step]['speed']]
            self.speed_pub.publish(speed_msg)
            self.step_pub.publish(Int32(data=int(self.current_step)))
            self.step_timer = self.get_clock().now()
            self.feed = self.steps[self.current_step]['feed']
        except StopIteration:
            self.feed = False

    def run_feeders(self):
        step_conf = self.steps[self.current_step]
        if self.get_clock().now() - self.step_timer < step_conf['duration']:
            if self.feed:
                for feeder, conf in step_conf['feeders'].items():
                    if self.get_clock().now() - conf['timer'] > conf['duration']:
                        if conf['random']:
                            kind = np.random.choice(list(conf['pack_proba'].keys()), p=list(conf['pack_proba'].values()))
                        else:
                            kind = next(conf['kind_sequence'])
                        self.feeders_pub[feeder].publish(UInt8(data=int(kind)))
                        conf['duration'] = rclpy.duration.Duration(seconds=np.random.uniform(conf['min_interval'],
                                                                                             conf['max_interval']))
                        conf['timer'] = self.get_clock().now()
        else:
            self.next_step()

    def feed_cb(self, msg):
        self.feed = msg.data


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = MultiFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
