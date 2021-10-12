# type: ignore

import rclpy.qos

from rqt_gui_py.plugin import Plugin

from python_qt_binding.QtCore import Qt, QPoint
from python_qt_binding.QtGui import QPainter, QPen, QColor
from python_qt_binding.QtWidgets import QWidget

from conveyor_utils.utils import Map
import conveyor_msgs.msg

from typing import Tuple


def orient(x: float, y: float, orientation: int) -> Tuple[float, float]:
    if orientation == 0:
        return (x, y)
    if orientation == 1:
        return (y, -x)
    if orientation == 2:
        return (-x, -y)
    if orientation == 3:
        return (-y, x)


def orient_bb(minx: float, miny: float, maxx: float, maxy: float, orientation: int
              ) -> Tuple[float, float, float, float]:
    if orientation == 0:
        return (minx, miny, maxx, maxy)
    if orientation == 1:
        return (-maxy, minx, -miny, maxx)
    if orientation == 2:
        return (-maxx, -maxy, -minx, -miny)
    if orientation == 3:
        return (miny, -maxx, maxy, -minx)


class ConveyorWidget(QWidget):

    def __init__(self, context=None):
        super(ConveyorWidget, self).__init__()
        self._node = context.node
        self.orientation = self._node.declare_parameter('orientation', 0).value
        path = self._node.declare_parameter('map_path', '').value
        self.map_ = Map.load_file(path, tol=0.1)
        (self.minx, self.miny, self.maxx, self.maxy) = orient_bb(
            *self.map_.bounding_box, self.orientation)
        self.margin = self._node.declare_parameter('margin', 50).value
        self.radius = self._node.declare_parameter('package_radius', 0.3).value
        self.tol = (self._node.declare_parameter('click_tolerance', 0.5).value + 1) * self.radius
        self._node.create_subscription(conveyor_msgs.msg.PackageList, 'packages',
                                       self.has_updated_packages, 1)
        qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self._node.create_publisher(
            conveyor_msgs.msg.PackageUIDList, 'selected_packages', qos)
        self.selected = set()
        self.positions = {}
        self.has_changed_size()
        self.publish_selection()

    def publish_selection(self):
        msg = conveyor_msgs.msg.PackageUIDList(uids=list(self.selected))
        self.pub.publish(msg)

    def has_changed_size(self):
        size = self.size()
        w, h = size.width(), size.height()
        self.scale = min((w - 2 * self.margin) / (self.maxx - self.minx),
                         (h - 2 * self.margin) / (self.maxy - self.miny))
        self.mx = self.margin - self.scale * self.minx
        self.my = self.margin + self.scale * self.maxy

    def resizeEvent(self, event):
        # size = event.size()
        # self._node.get_logger().info(f"resized to on ({size.width()} x {size.height()})")
        self.has_changed_size()

    def mousePressEvent(self, event):
        p = event.localPos()
        x, y = self.world(p.x(), p.y())
        # self._node.get_logger().info(f"clicked on ({p.x()}, {p.y()} ~ ({x}, {y})")
        for uid, (px, py, _) in self.positions.items():
            if abs(px - x) < self.tol and abs(py - y) < self.tol:
                # self._node.get_logger().info(f"-> on uid {uid}")
                if uid in self.selected:
                    self.selected.remove(uid)
                else:
                    self.selected.add(uid)
                self.publish_selection()
                return

    def canvas(self, x: float, y: float):
        x, y = orient(x, y, (4 - self.orientation) % 4)
        x, y = (self.mx + self.scale * x, self.my - self.scale * y)
        return x, y

    def world(self, x: float, y: float):
        x, y = (x - self.mx) / self.scale, (y - self.my) / (-self.scale)
        x, y = orient(x, y, self.orientation)
        return x, y

    def has_updated_packages(self, msg):
        self.positions = {p.uid: self.map_.point_from_msg(p.position) for p in msg.packages}
        self.selected &= set(self.positions)
        self.update()

    def paintEvent(self, event):
        qp = QPainter()
        qp.begin(self)
        self.draw_map(event, qp)
        self.draw_packages(event, qp)
        qp.end()

    def draw_map(self, event, qp):
        for belt in self.map_.belts:
            qp.setPen(QPen(QColor(150, 150, 150), belt.width * self.scale, Qt.SolidLine))
            ps = [QPoint(*self.canvas(x, y)) for x, y, _ in belt.centerline.coords]
            qp.drawPolyline(*ps)

    def draw_packages(self, event, qp):
        r = self.scale * self.radius
        qp.setPen(QPen(QColor(0, 0, 0), 1, Qt.SolidLine))
        for uid, (x, y, _) in self.positions.items():
            if uid in self.selected:
                qp.setBrush(QColor(200, 200, 0))
            else:
                qp.setBrush(QColor(50, 50, 0))
            # self._node.get_logger().info(f'{uid} {(x, y)} <-> {self.canvas(x, y)}')
            p = QPoint(*self.canvas(x, y))
            qp.drawEllipse(p, r, r)


class GUI(Plugin):

    def __init__(self, context):
        super(GUI, self).__init__(context)
        self.setObjectName('GUI')
        self._context = context
        self._widget = ConveyorWidget(context=context)
        self._context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        ...

    def restore_settings(self, plugin_settings, instance_settings):
        ...

    def trigger_configuration(self):
        ...

    def shutdown_plugin(self):
        ...
