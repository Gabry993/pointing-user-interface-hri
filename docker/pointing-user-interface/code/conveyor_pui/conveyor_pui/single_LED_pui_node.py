from typing import Any

import rclpy
import rclpy.node

from conveyor_msgs.msg import State, PUISubscription
from geometry_msgs.msg import PoseStamped
import tf2_ros
from relloc.utils import distance_between, do_transform_pose, ray_pose_to_vector
from std_msgs.msg import ColorRGBA
from conveyor_utils.utils import LEDs

import matplotlib
import matplotlib.cm as cm


class SingleLEDPUINode(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(SingleLEDPUINode, self).__init__("single_pui_node")
        path = self.declare_parameter('map_path', '').value
        self.tol = self.declare_parameter("pointing_tol", 0.7).value

        self.leds = LEDs.load_file(path).leds

        self.led_pubs = {uid: self.create_publisher(ColorRGBA, uid+"/color", 1) for uid in self.leds}

        # self.create_subscription(PoseStamped, "/pointing_ray",
        #                          self.pointing_ray_cb, 100)

        self.create_subscription(PUISubscription, "/PUI_subscription", self.pui_subscription_cb, 10)

        self.tf_buff = tf2_ros.Buffer()
        self.tf_ls = tf2_ros.TransformListener(self.tf_buff, node=self)
        qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            State, "/state_transition", self.state_cb, qos
        )
        self.user_name = None
        self.user_ray_sub = None
        self.pointing_cmap = None
        self.timer = self.create_timer(20., self.reset_leds)
        self.state = "ready"
        self.get_logger().info("Ready")

    def state_cb(self, msg):
        if msg.node_name == self.get_name():
            self.change_to_state(msg.state)

    def change_to_state(self, new_state):
        self.get_logger().info(f"State transition {self.state} -> {new_state}")
        self.state = new_state

    def pui_subscription_cb(self, msg: PUISubscription) -> None:
        if msg.subscribe:
            if msg.user_name != self.user_name:
                self.get_logger().warn(f"New user name [{msg.user_name}] subscription")
                if self.user_ray_sub:
                    self.destroy_subscription(self.user_ray_sub)
                self.user_name = msg.user_name
                self.user_ray_sub = self.create_subscription(PoseStamped, f"/{msg.user_name}/pointing_ray",
                                                             self.pointing_ray_cb, 100)
                norm = matplotlib.colors.Normalize(vmin=0.0, vmax=self.tol, clip=True)
                self.pointing_cmap = cm.ScalarMappable(norm=norm, cmap=msg.pointer_cmap)
        else:
            self.user_name = None
            self.pointing_cmap = None
            if self.user_ray_sub:
                self.destroy_subscription(self.user_ray_sub)
            self.get_logger().warn(f"User name [{msg.user_name}] unsubscribed")

    def pointing_ray_cb(self, msg: PoseStamped) -> None:
        ray_frame = msg.header.frame_id
        try:
            transform = self.tf_buff.lookup_transform(
                "world", ray_frame, rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().error(str(e), throttle_duration_sec=5)
            return
        ray = do_transform_pose(msg, transform)
        r_o, r_dir = ray_pose_to_vector(ray)

        for uid, pos in self.leds.items():
            distance = distance_between(r_o, r_dir, pos)
            color = self.pointing_cmap.to_rgba(distance)
            self.led_pubs[uid].publish(
                ColorRGBA(
                    **{"r": color[0], "g": color[1], "b": color[2], "a": 1.0}
                )
            )

    def reset_leds(self):
        if self.state == "reset":
            for uid in self.leds:
                self.led_pubs[uid].publish(
                    ColorRGBA(**{"a": 0.0}))
            self.change_to_state("ready")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = SingleLEDPUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
