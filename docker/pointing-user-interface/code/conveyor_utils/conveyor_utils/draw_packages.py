from itertools import groupby
from typing import List, Any, Tuple, Set, Optional, Dict
from functools import lru_cache

import rclpy
import rclpy.node
import led_strip_msgs.msg
import conveyor_msgs.msg
import numpy as np
import yaml
import matplotlib
import matplotlib.cm as cm

from .utils import Map, Strip, Range

Color = List[int]


class DrawPacketsNode(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(DrawPacketsNode, self).__init__("draw_packages")
        self.selection: Set[int] = set()
        self.overlays: List[int] = []
        self.pub = self.create_publisher(led_strip_msgs.msg.LedStrips, 'led_strips', 1)
        self.create_subscription(conveyor_msgs.msg.PackageList, 'packages',
                                 self.has_updated_packages, 1)
        self.create_subscription(conveyor_msgs.msg.PackageUIDList, 'selected_packages',
                                 self.has_updated_selected_packages, 1)
        # self.create_subscription(conveyor_msgs.msg.Pointer, 'pointer',
        #                          self.has_updated_pointer, 1)
        self.create_subscription(conveyor_msgs.msg.PointerList, 'pointers',
                                 self.has_updated_pointers, 1)
        path = self.declare_parameter('map_path', '').value
        colors: str = self.declare_parameter('colors', "{}").value
        self.colors_for_type = yaml.safe_load(colors)
        self.default_color = self.declare_parameter('color', [0, 0, 0]).value
        colors = self.declare_parameter('selection_colors', "{}").value
        self.colors_for_selected_type = yaml.safe_load(colors)
        self.default_selected_color = self.declare_parameter('selection_color', [255, 255, 0]).value

        # colormap = self.declare_parameter('pointing_cmap', '').value
        self.tol = self.declare_parameter("pointing_tol", 0.7).value
        # if colormap != '':
        #     norm = matplotlib.colors.Normalize(vmin=0.0, vmax=tol, clip=True)
        #     self.pointing_cmap = cm.ScalarMappable(norm=norm, cmap=colormap)
        # else:
        #     self.pointing_cmap = None
        # self.pointer_color = [255, 255, 0]

        self.map_ = Map.load_file(path, tol=0.1)
        self.pointer: Optional[Tuple[str, float]] = None
        self.pointer_size: float = 0.0
        self.strips_data: Dict[Strip, np.ndarray] = {strip: strip.draw(self.default_color, [])
                                                     for strip in self.map_.strips}
        self.user_pointers = {}
        self.create_timer(1/60, self.draw_all)
        self.get_logger().info("Ready")

    def has_updated_selected_packages(self, msg: conveyor_msgs.msg.PackageUIDList) -> None:
        self.selection = set(msg.uids)

    def has_updated_pointers(self, msg: conveyor_msgs.msg.PointerList) -> None:
        self.overlays = []
        for pointer in msg.pointers:
            user_name = pointer.user_name
            self.user_pointers[user_name] = {}
            if not pointer.position.name == '':
                self.user_pointers[user_name]['pointer'] = (pointer.position.name, pointer.position.position)
                self.user_pointers[user_name]['size'] = pointer.size
                self.user_pointers[user_name]['overlay'] = pointer.overlay.uids
                # self.pointer_size = pointer.size
                self.overlays += pointer.overlay.uids
                if pointer.cmap != "":
                    self.user_pointers[user_name]['color'] = self.get_cmap(pointer.cmap).to_rgba(pointer.distance,
                                                                                                 bytes=True)[:3]
                    # self.pointer_color = self.pointing_cmap.to_rgba(pointer.distance, bytes=True)[:3]
                else:
                    self.user_pointers[user_name]['color'] = [int(pointer.color.r*255.),
                                                              int(pointer.color.g * 255.),
                                                              int(pointer.color.b*255.)]
                # self.draw_all()
            else:
                self.user_pointers[user_name]['pointer'] = None
                self.user_pointers[user_name]['size'] = 0.0
                self.user_pointers[user_name]['overlay'] = []
                # self.pointer = None
                # self.pointer_size = 0.0
                # self.overlay = []
                # self.draw_all()

    @lru_cache(maxsize=None)
    def get_cmap(self, cmap):
        norm = matplotlib.colors.Normalize(vmin=0.0, vmax=self.tol, clip=True)
        return cm.ScalarMappable(norm=norm, cmap=cmap)

    def draw_all(self) -> None:
        # rs: List[Tuple[Strip, Range]] = []
        strips_data = {strip: data.copy() for strip, data in self.strips_data.items()}
        for user, d in self.user_pointers.items():
            if d['pointer']:
                rs = [r for r in self.map_.strips_near(d['pointer'][0],
                                                       d['pointer'][1],
                                                       width=d['size'])]
                for (strip, _range) in rs:
                    strip_data = strip.draw(d['color'], [_range])
                    mask = np.any(strip_data > 0, axis=1)
                    strips_data[strip][mask, :] = 0
                    strips_data[strip] += strip_data

        rmsg = led_strip_msgs.msg.LedStrips()
        for strip, data in strips_data.items():
            data = np.clip(data.flatten(), 0, 255).tolist()
            strip_msg = led_strip_msgs.msg.LedStrip(id=strip.uid, data=data)
            rmsg.strips.append(strip_msg)
        self.pub.publish(rmsg)

    def has_updated_packages(self, msg: conveyor_msgs.msg.PackageList) -> None:
        rs: List[Tuple[Strip, Range, int, int]] = []
        for package in msg.packages:
            belt_name = package.position.name
            position = package.position.position
            length = package.length
            rs += [(strip, r, package.type, package.uid)
                   for (strip, r) in self.map_.strips_near(belt_name, position, width=length)]
        ds = {k: list(v) for k, v in groupby(sorted(rs), key=lambda x: x[0])}
        for strip in self.map_.strips:
            vs = ds.get(strip, [])
            if not vs:
                self.strips_data[strip] = strip.draw(self.default_color, [])
            else:
                ty = [strip.draw(self.color_for(t, uid in self.selection), [r])
                      for _, r, t, uid in vs]
                for i, ((_, _, t, uid), tx) in enumerate(zip(vs, ty)):
                    marg_color = self.get_margin_color(t, uid in self.selection, uid in self.overlays)
                    bs = np.where(tx != [0, 0, 0])[0]
                    if bs.size != 0:
                        s = bs[0]
                        e = bs[-1]
                        if s > 1:
                            tx[s-2] = marg_color
                        if e < len(tx)-3:
                            tx[e+2] = marg_color
                    ty[i] = np.array(tx)
                self.strips_data[strip] = sum(ty)
        # self.draw_all()

    def get_margin_color(self, kind: int, selected: bool, pointed: bool) -> Color:
        if selected and pointed:
            return [255, 255, 255]
        elif selected and not pointed:
            return [150, 150, 150]
        elif not selected and pointed:
            return [20, 20, 20]
        else:
            return [0, 0, 0]

    def color_for(self, kind: int, selected: bool) -> Color:
        if not selected:
            return self.colors_for_type.get(kind, self.default_color)
        return self.colors_for_selected_type.get(kind, self.default_selected_color)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DrawPacketsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
