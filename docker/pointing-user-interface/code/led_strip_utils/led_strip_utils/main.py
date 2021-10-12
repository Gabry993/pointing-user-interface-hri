import itertools
from typing import List, Tuple, Any, Optional

import rclpy
import rclpy.node
import rclpy.duration
import rclpy.task
import rclpy.timer
import led_strip_msgs.msg
import led_strip_msgs.srv
import yaml
import rcl_interfaces.msg


WHITE = [255, 255, 255]
BLACK = [0, 0, 0]
GREY = [127, 127, 127]
RED = [255, 0, 0]
GREEN = [0, 255, 0]
BLUE = [0, 0, 255]
YELLOW = [255, 255, 0]
MAGENTA = [255, 0, 255]
CYAN = [0, 255, 255]

Strip = Tuple[int, int, int]

COLORS = [WHITE, GREY, BLACK, RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA]


def pattern(strips: List[Strip], offset: int) -> led_strip_msgs.msg.LedStrips:
    msg = led_strip_msgs.msg.LedStrips()
    ls = [size for _, size, direction in strips]
    total_size = sum(ls)
    n = len(COLORS)
    size = total_size // n
    data: List[List[int]] = sum(([c] * size for c in COLORS), [])
    data += [BLACK] * (total_size - len(data))
    offset = (offset % total_size)
    data = data[offset:] + data[:offset]
    bs = zip(itertools.accumulate(ls, initial=0), itertools.accumulate(ls))   # type: ignore
    buffers = [data[i: j] for i, j in bs]
    msg.strips = [
        led_strip_msgs.msg.LedStrip(id=i, color_order=0, data=sum(ds[::direction], []))
        for (i, _, direction), ds in zip(strips, buffers)
    ]
    return msg


class LedStripClient(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(LedStripClient, self).__init__("led_strip_client")
        self.pub = self.create_publisher(led_strip_msgs.msg.LedStrips, 'led_strips', 1)
        path = self.declare_parameter('map_path', '').value
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.strips: List[Strip] = [
            (channel, s['pixels'], s['direction']) for channel, s in data.get('strips', {}).items()]
        self.get_logger().info(f"Loaded strips {self.strips} from {path}")
        self.offset = 0
        self.brightness = self.declare_parameter('brightness.value', 0.1).value
        self.brightness_step = self.declare_parameter('brightess.step', 0).value
        self.brightness_period = self.declare_parameter('brightess.period', 0).value
        self.period = self.declare_parameter('period', 0.1).value
        self.offset_step = self.declare_parameter('step', 1).value
        self.set_brightness_future: Optional[rclpy.task.Future] = None
        self.set_brightness = self.create_client(
            led_strip_msgs.srv.SetBrightness, 'set_brightness')
        self.publish()
        self.get_logger().info("Waiting for the service")
        if self.brightness >= 0:
            self.set_brightness.wait_for_service()
            self.set_brightness_timeout = None
            if self.brightness_period:
                self.get_logger().info(
                    f"Start brightness timer with period {self.brightness_period}"
                    f" and step {self.brightness_step}")
                self.btimer = self.create_timer(2.0, self.change_brightness)
            else:
                req = led_strip_msgs.srv.SetBrightness.Request(
                    brightness=self.brightness, channel_index_mask=0xFF)
                self.set_brightness.call_async(req)
        self.timer: Optional[rclpy.timer.Timer]
        if self.period:
            self.get_logger().info(
                f"Start timer with period {self.period} and step {self.offset_step}")
            self.timer = self.create_timer(self.period, self.publish)
        else:
            self.timer = None
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info("Ready")

    def parameter_callback(self, params: Any) -> rcl_interfaces.msg.SetParametersResult:
        for param in params:
            if param.name == 'period':
                self.period = param.value
                if not self.timer and self.period:
                    self.timer = self.create_timer(self.period, self.publish)
                elif self.timer:
                    if not self.period:
                        self.destroy_timer(self.timer)
                        self.timer = None
                    else:
                        old_period_ns = self.timer.timer_period_ns
                        period_ns = round(1e9 * self.period)
                        if period_ns != old_period_ns:
                            self.timer.timer_period_ns = period_ns
                            self.timer.reset()
            if param.name == 'step':
                self.offset_step = param.value
        return rcl_interfaces.msg.SetParametersResult(successful=True)

    def publish(self) -> None:
        self.offset += self.offset_step
        self.pub.publish(pattern(self.strips, self.offset))

    def change_brightness(self) -> None:
        if self.set_brightness_future:
            if self.set_brightness_future.done():
                response = self.set_brightness_future.result()
                self.get_logger().debug(f"Got response {response}")
            else:
                if self.get_clock().now() < self.set_brightness_timeout:
                    self.get_logger().info(
                        "Waiting ... {self.get_clock().now()} < {self.set_brightness_timeout}")
                    return
                else:
                    self.get_logger().warn("Timed out")
                    self.set_brightness.remove_pending_request(self.set_brightness_future)
                    self.set_brightness_future = None
        self.get_logger().debug("change_brightness")
        self.brightness += self.brightness_step
        if self.brightness > 1:
            self.brightness_step *= -1
            self.brightness = 1.0
        if self.brightness < 0:
            self.brightness_step *= -1
            self.brightness = 0.0
        req = led_strip_msgs.srv.SetBrightness.Request(
            brightness=self.brightness, channel_index_mask=0xFF)
        self.set_brightness_timeout = self.get_clock().now() + rclpy.duration.Duration(seconds=1)
        self.set_brightness_future = self.set_brightness.call_async(req)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = LedStripClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
