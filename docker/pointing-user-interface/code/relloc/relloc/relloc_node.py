from typing import Any
import collections
import threading

from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from conveyor_msgs.msg import State
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
import rclpy.node
import numpy as np
import PyKDL as kdl
from relloc.relloclib import estimate_pose_no_constraints
from relloc.utils import frame_to_tf
import tf2_ros
import time


class RellocNode(rclpy.node.Node):
    def __init__(self) -> None:
        super(RellocNode, self).__init__("relloc_node")
        self.lock = threading.RLock()
        self.publish_rate = self.declare_parameter("publish_rate", 50.0).value
        led_on_period = self.declare_parameter('led_on_period', 3.).value
        self.timewindow = led_on_period * 2
        self.sync_freq = self.declare_parameter("sync_freq", 50.0).value
        self.user_tf_prefix = self.declare_parameter("user_tf_prefix", "").value
        self.sample_size = int(self.sync_freq * self.timewindow)

        self.pointing_q = collections.deque(maxlen=self.sample_size)

        self.pointing_ray_sub = Subscriber(self, PoseStamped, "pointing_ray")
        self.pointing_target_sub = Subscriber(self, Vector3Stamped, "pointing_target")

        self.synchronizer = ApproximateTimeSynchronizer([self.pointing_ray_sub, self.pointing_target_sub],
                                                        queue_size=100, slop=0.1)
        self.synchronizer.registerCallback(self.topic_sync_cb)

        qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            State, "state_transition", self.state_cb, qos
        )
        self.state_pub = self.create_publisher(
            State, "state_transition", qos
        )

        self.state = "ready"
        self.on_timer = None

        self.transform = None
        self.tf_transform = None
        self.tf_br = tf2_ros.TransformBroadcaster(node=self)
        self.imu_yaw = None
        self.theta = None
        timer_period = 1 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_tf)
        self.get_logger().info(f"State: {self.state}")

    def state_cb(self, msg):
        if msg.node_name == self.get_name():
            self.change_to_state(msg.state)
            self.run()

    def change_to_state(self, new_state):
        self.get_logger().info(f"State transition {self.state} -> {new_state}")
        self.state = new_state

    def topic_sync_cb(self, ray_msg, target_msg):
        ray_origin = np.array([ray_msg.pose.position.x, ray_msg.pose.position.y, ray_msg.pose.position.z])
        ray_dir = ray_msg.pose.orientation
        target = np.array([target_msg.vector.x, target_msg.vector.y, target_msg.vector.z])
        rotation = kdl.Rotation.Quaternion(ray_dir.x, ray_dir.y, ray_dir.z, ray_dir.w)
        if self.state == 'collect_data':
            dir_vect = rotation * kdl.Vector(x=1., y=0., z=0.)
            dir_vect = np.array([dir_vect.x(), dir_vect.y(), dir_vect.z()])
            sample = np.concatenate((ray_origin, dir_vect, target), axis=None)
            with self.lock:
                self.pointing_q.append(sample)
        _, _, yaw = rotation.GetRPY()
        self.imu_yaw = kdl.Rotation.RPY(0, 0, yaw)

    # def set_gt_relloc(self):
    #     tr, theta = np.array(self.gt_trans), 0
    #     rotation = kdl.Rotation.RPY(0, 0, theta)
    #     pos = kdl.Vector(*tr)
    #     self.transform = kdl.Frame(R=rotation, V=pos).Inverse()   # active transform
    #     self.tf_transform = frame_to_tf(self.transform)
    #     self.publish_tf()

    def compute_relloc(self):
        with self.lock:
            if len(list(self.pointing_q)) < self.sample_size:
                return False
            samples = list(self.pointing_q)
            self.pointing_q.clear()
            samples = np.array(samples)
        qc = samples[:, 0:3]
        qv = samples[:, 3:6]
        p = samples[:, 6:]

        res, maxerr = estimate_pose_no_constraints(p.T, qc.T, qv.T, np.array([0, 0, 0, 0]))
        tr, theta = np.array(res.x[0:3]), res.x[3]
        rotation = kdl.Rotation.RPY(0, 0, theta)
        pos = kdl.Vector(*tr)
        self.transform = kdl.Frame(R=rotation, V=pos).Inverse()   # passive transform must be inverted to get active
        self.tf_transform = frame_to_tf(self.transform)
        self.publish_tf()
        self.get_logger().info(f"{self.transform}")
        return True

    def publish_tf(self):
        if self.tf_transform is not None:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = self.user_tf_prefix + "human_footprint"
            t.transform = self.tf_transform
            self.tf_br.sendTransform(t)

    def run(self):
        if self.state == "gt_localize":
            self.change_to_state("computing_relloc")
            self.change_to_state("localized")
            state = State(node_name="supervisor", state="relloc_done")
            self.get_logger().info("Waiting 2 second before going on")
            self.state_pub.publish(state)
            time.sleep(2)
            #state = State(node_name="pui", state="relloc_done")
            #self.state_pub.publish(state)
        elif self.state == "compute_relloc":
            self.change_to_state("computing_relloc")
            if self.compute_relloc():
                self.change_to_state("localized")
                state = State(node_name="supervisor", state="relloc_done")
                self.get_logger().info("Waiting 2 second before going on")
                time.sleep(2)
                self.state_pub.publish(state)
                #state = State(node_name="pui", state="pointing")
                #self.state_pub.publish(state)
            else:
                self.change_to_state("error_no_relloc_data")
        elif self.state == "localized":
            self.publish_tf()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = RellocNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
