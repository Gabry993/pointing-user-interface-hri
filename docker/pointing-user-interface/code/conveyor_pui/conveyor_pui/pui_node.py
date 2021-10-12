from typing import Any, List, Tuple, Optional, Set, Dict

import rclpy
import rclpy.node
import conveyor_msgs.msg
from conveyor_utils.utils import Map, Strip, Range
from conveyor_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros
from relloc.utils import segments, do_transform_pose, ray_pose_to_vector

import numpy as np
from shapely.geometry import Point
from scipy.spatial import KDTree
from scipy.stats import mode
from collections import deque
import chime
chime.theme("big-sur")


class PUINode(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(PUINode, self).__init__("pui_node")
        path = self.declare_parameter('map_path', '').value
        self.update_rate = self.declare_parameter("update_rate", 10).value
        self.period = 1/self.update_rate
        self.time_to_sel = self.declare_parameter("time_for_selection", 0.1).value
        self.pointer_size = self.declare_parameter("pointer_size", 0.2).value
        self.tol = self.declare_parameter("pointing_tol", 0.7).value

        self.map_ = Map.load_file(path, tol=0.1)
        self.pointer: Optional[Tuple[str, float]] = None

        self.create_subscription(conveyor_msgs.msg.PackageList, '/packages',
                                 self.has_updated_packages, 1)
        # self.create_subscription(PoseStamped, "pointing_ray",
        #                          self.pointing_ray_cb, 100)
        self.create_subscription(conveyor_msgs.msg.PUISubscription, "/PUI_subscription", self.pui_subscription_cb, 10)

        self.tf_buff = tf2_ros.Buffer()
        self.tf_ls = tf2_ros.TransformListener(self.tf_buff, node=self)

        qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.selection_pub = self.create_publisher(
            conveyor_msgs.msg.PackageUIDList, '/selected_packages', qos)

        # self.pointer_pub = self.create_publisher(
        #     conveyor_msgs.msg.Pointer, '/pointer', 1)

        self.pointers_pub = self.create_publisher(
            conveyor_msgs.msg.PointerList, '/pointers', 1)

        qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            State, "state_transition", self.state_cb, qos
        )

        self.selected: Set[int] = set()
        self.packages: List[conveyor_msgs.msg.Package] = []
        self.selection_t: Optional[float] = None
        self.last_overlay: Optional[int] = None
        self.smart_overlays = {}
        self.last_overlays = {}
        self.q_size: int = int(self.update_rate * self.time_to_sel)
        self.overlay_q: deque = deque(maxlen=self.q_size)
        self.tree_mapping: Dict[int, str] = {}
        self.tree: KDTree
        self.bins: List[int] = []
        self.step = 0.05
        self.generate_kdtree()

        self.user_pointers = {}
        # self.publish_selection()
        self.create_timer(self.period, self.check_selection_smart)
        self.state = "ready"
        self.get_logger().info(f"State: {self.state}")

    def state_cb(self, msg: State) -> None:
        if msg.node_name == self.get_name():
            self.change_to_state(msg.state)

    def change_to_state(self, new_state):
        self.get_logger().info(f"State transition {self.state} -> {new_state}")
        self.state = new_state

    def generate_kdtree(self) -> None:
        start = 0
        points = []
        for i, belt in enumerate(self.map_.belts):
            self.bins.append(start)
            for segment in segments(belt.centerline):
                length = np.linalg.norm(segment[1]-segment[0])
                if length != 0.:
                    if length > self.step:
                        num = int(length/self.step)
                    else:
                        num = 1
                    pts = np.linspace(segment[0], segment[1], num=num)
                    start += len(pts)
                    points.extend(pts)
            self.tree_mapping[i] = belt.uid
        self.tree = KDTree(np.array(points))

    def pui_subscription_cb(self, msg: conveyor_msgs.msg.PUISubscription) -> None:
        if msg.subscribe:
            if msg.user_name in self.user_pointers:
                self.get_logger().warn(f"User [{msg.user_name}] was already subscribed. Overwriting subscription")
                self.destroy_subscription(self.user_pointers[msg.user_name]['sub'])
                del self.user_pointers[msg.user_name]
            sub = self.create_subscription(PoseStamped, f"/{msg.user_name}/pointing_ray",
                                           self.create_user_cb(msg.user_name), 100)
            self.user_pointers[msg.user_name] = {'msg': conveyor_msgs.msg.Pointer(),
                                                 'pointer': None,
                                                 'sub': sub,
                                                 'size': msg.pointer_size,
                                                 'color': msg.pointer_color,
                                                 'cmap': msg.pointer_cmap,
                                                 'overlay_q': deque(maxlen=self.q_size)}
            self.smart_overlays[msg.user_name] = None
            self.last_overlays[msg.user_name] = None
            self.get_logger().warn(f"New user name [{msg.user_name}] subscription")
        elif msg.user_name in self.user_pointers:
            self.destroy_subscription(self.user_pointers[msg.user_name]['sub'])
            self.user_pointers[msg.user_name]['msg'] = conveyor_msgs.msg.Pointer()
            self.user_pointers[msg.user_name]['msg'].user_name = msg.user_name
            self.publish_pointers()
            del self.user_pointers[msg.user_name]
            del self.smart_overlays[msg.user_name]
            del self.last_overlays[msg.user_name]
            self.get_logger().warn(f"User [{msg.user_name}] unsuscribed")
        else:
            self.get_logger().warn(f"User [{msg.user_name}] can't unsubscribe because it wasn't subscribed")

    def create_user_cb(self, user_name: str):
        def user_pointing_ray_cb(msg: PoseStamped) -> None:
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
            self.update_pointer(ray, user_name)

        return user_pointing_ray_cb

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
        self.update_pointer(ray)

    def publish_selection(self) -> None:
        msg = conveyor_msgs.msg.PackageUIDList(uids=list(self.selected))
        self.selection_pub.publish(msg)

    def has_updated_packages(self, msg: conveyor_msgs.msg.PackageList) -> None:
        self.packages = msg.packages
        self.selected &= set([p.uid for p in self.packages])

    def check_selection_smart(self) -> None:
        for user, d in self.user_pointers.items():
            if d['pointer'] is not None:
                pack_rs: List[Tuple[Strip, Range, int, int]] = []
                for package in self.packages:
                    belt_name = package.position.name
                    position = package.position.position
                    length = package.length
                    pack_rs += [(strip, rs, package.type, package.uid)
                                for (strip, rs) in self.map_.strips_near(belt_name,
                                                                         position,
                                                                         width=length)]
                curr_overlay = self.get_overlay(d['pointer'], d['size'], pack_rs)
            else:
                curr_overlay = None

            self.last_overlays[user] = curr_overlay
            if curr_overlay is None:
                curr_overlay = -1

            d['overlay_q'].append(curr_overlay)

            mode_res = mode(d['overlay_q'], nan_policy='omit')
            mode_overlay = mode_res.mode[0] if mode_res.count[0] / self.q_size >= 0.75 else None
            if mode_overlay == -1:
                mode_overlay = None

            if mode_overlay not in list(self.smart_overlays.values()):
                if mode_overlay is not None:
                    self.toggle_selection(mode_overlay)
                    self.publish_selection()
            self.smart_overlays[user] = mode_overlay

            # if mode_overlay != self.smart_overlays[user]:
            #     self.last_overlay = mode_overlay
            #     if mode_overlay is not None:
            #         self.toggle_selection(mode_overlay)
            #         self.publish_selection()

    def check_selection(self) -> None:
        if self.pointer is not None:
            pack_rs: List[Tuple[Strip, Range, int, int]] = []
            for package in self.packages:
                belt_name = package.position.name
                position = package.position.position
                length = package.length
                pack_rs += [(strip, rs, package.type, package.uid)
                            for (strip, rs) in self.map_.strips_near(belt_name,
                                                                     position,
                                                                     width=length)]
            curr_overlay = self.get_overlay(pack_rs)

            if curr_overlay is not None:
                if curr_overlay != self.last_overlay:
                    self.selection_t = 0.0
                    self.last_overlay = curr_overlay
                elif curr_overlay == self.last_overlay:
                    if self.selection_t is None:
                        return
                    elif self.selection_t > self.time_to_sel:
                        self.toggle_selection(curr_overlay)
                        self.publish_selection()
                        self.selection_t = None
                    else:
                        self.selection_t += self.period
            else:
                self.selection_t = None
                self.last_overlay = None

    def toggle_selection(self, uid: int) -> None:
        if uid in self.selected:
            self.selected.remove(uid)
            chime.error()
        else:
            self.selected.add(uid)
            chime.success()

    @staticmethod
    def overlap(a_range: Tuple[float, float],
                o_range: Tuple[float, float]) -> bool:
        return a_range[1] >= o_range[0] and o_range[1] >= a_range[0]

    def get_overlay(self, pointer, pointer_size, pack_rs: List[Tuple[Strip, Range, int, int]]) -> Optional[int]:
        pointer_rs: List[Tuple[Strip, Range]] = []
        if pointer is not None:
            pointer_rs = [rs for rs in self.map_.strips_near(pointer[0],
                                                             pointer[1],
                                                             width=pointer_size)]
        for pointer_s, pointer_r in pointer_rs:
            for (pack_s, pack_r, _, uid) in pack_rs:
                if pack_s is pointer_s:
                    if PUINode.overlap(pointer_r, pack_r):
                        return uid
        return None

    def update_pointer(self, msg: PoseStamped, user_name: str) -> None:
        r_o, r_dir = ray_pose_to_vector(msg)
        r_e = r_o + 20*r_dir
        num = int(np.linalg.norm(r_e - r_o)/self.step)
        r_pts = np.linspace(r_o, r_e, num=num)
        ds, ids = self.tree.query(r_pts, k=1, distance_upper_bound=self.tol, workers=2)
        min_idx = ids[np.argmin(ds)]
        if min_idx < self.tree.n:
            min_p = self.tree.data[:][min_idx]
            belt_idx = np.searchsorted(np.array(self.bins), min_idx)-1
            min_belt = list(self.map_.belts)[belt_idx]
            position = min_belt.centerline.project(Point(min_p), normalized=True)
            pointer = conveyor_msgs.msg.Pointer()
            pointer.position.name = min_belt.uid
            pointer.position.position = position
            pointer.size = self.user_pointers[user_name]["size"]
            pointer.color = self.user_pointers[user_name]["color"]
            pointer.cmap = self.user_pointers[user_name]["cmap"]
            pointer.distance = np.min(ds)
            last_overlay = self.last_overlays[user_name]
            pointer.overlay.uids = [int(last_overlay)] if last_overlay is not None else []
            pointer.user_name = user_name
            self.user_pointers[user_name]['msg'] = pointer
            self.user_pointers[user_name]['pointer'] = (min_belt.uid, position)
            # self.pointer = (min_belt.uid, position)
            # self.pointer_pub.publish(pointer)
        else:
            self.user_pointers[user_name]['msg'] = conveyor_msgs.msg.Pointer()
            self.user_pointers[user_name]['msg'].user_name = user_name
            self.user_pointers[user_name]['pointer'] = None
            # self.pointer = None
            # self.pointer_pub.publish(conveyor_msgs.msg.Pointer())
            # self.last_overlay = None
        self.publish_pointers()

    def publish_pointers(self):
        pointers_msg = conveyor_msgs.msg.PointerList()
        pointers_msg.pointers = [d['msg'] for d in self.user_pointers.values()]
        self.pointers_pub.publish(pointers_msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = PUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
