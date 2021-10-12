from typing import Tuple
import PyKDL as kdl
from geometry_msgs.msg import Pose, Transform, PoseStamped
import numpy as np
from shapely.geometry import LineString, Point


def vector_to_np(v):
    return np.array([v.x(), v.y(), v.z()])


def frame_to_tf(f):
    tf = Transform()
    (
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z,
        tf.rotation.w,
    ) = f.M.GetQuaternion()
    tf.translation.x = f.p[0]
    tf.translation.y = f.p[1]
    tf.translation.z = f.p[2]
    return tf


def transform_to_kdl(t):
    return kdl.Frame(
        kdl.Rotation.Quaternion(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ),
        kdl.Vector(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ),
    )


def tf_from_pose(p):
    """
    :param p: input pose
    :type p: :class:`geometry_msgs.msg.Pose`
    :return: New :class:`PyKDL.Frame` object
    Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
    """
    return kdl.Frame(
        kdl.Rotation.Quaternion(
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        ),
        kdl.Vector(p.position.x, p.position.y, p.position.z),
    )


def tf_to_pose(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`
    Return a ROS Pose message for the Frame f.
    """
    p = Pose()
    (
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w,
    ) = f.M.GetQuaternion()
    p.position.x = f.p[0]
    p.position.y = f.p[1]
    p.position.z = f.p[2]
    return p


def do_transform_pose(pose, transform):
    f = transform_to_kdl(transform) * kdl.Frame(kdl.Rotation.Quaternion(pose.pose.orientation.x,
                                                                        pose.pose.orientation.y,
                                                                        pose.pose.orientation.z,
                                                                        pose.pose.orientation.w),
                                                kdl.Vector(pose.pose.position.x,
                                                           pose.pose.position.y,
                                                           pose.pose.position.z))
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x,
     res.pose.orientation.y,
     res.pose.orientation.z,
     res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res


def distance_between(p1, v1, p2):
    """ Returns distance between a point and a line:
        - p1 is a point on the line
        - v1 is the direction vector of the line
        - p2 is the point for which we want the distance
    https://onlinemschool.com/math/library/analytic_geometry/p_line/
    """
    return np.linalg.norm(np.cross(p1-p2, v1))/np.linalg.norm(v1)


def distance_between_two_lines(v1, v2, p1, p2):
    """
    Returns distance between 2 lines and the 2 closest point on each line:
        - p1, p2 are points on each line
        - v1, v2 are the direction vectors of each line
    https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
    https://math.stackexchange.com/questions/1414285/location-of-shortest-distance-between-two-skew-lines-in-3d
    """
    # Find the unit vector perpendicular to both lines
    n = np.cross(v1, v2)
    n /= np.linalg.norm(n)
    # Calculate distance
    d = np.dot(n, p1 - p2)
    n2 = np.cross(v2, n)
    n1 = np.cross(v1, n)
    c1 = p1 + v1*np.dot(p2-p1, n2)/np.dot(v1, n2)
    c2 = p2 + v2*np.dot(p1-p2, n1)/np.dot(v2, n1)
    return d, c1, c2


def ray_pose_to_vector(msg: PoseStamped) -> Tuple[np.ndarray, np.ndarray]:
    point = np.array([msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z])
    ori = msg.pose.orientation
    rotation = kdl.Rotation.Quaternion(ori.x, ori.y, ori.z, ori.w)
    dir_vect = vector_to_np(rotation * kdl.Vector(x=1., y=0., z=0.))
    return point, dir_vect


def line_to_vector(line: LineString) -> Tuple[np.ndarray, np.ndarray]:
    start, end = np.array(line.coords[0]), np.array(line.coords[-1])
    dir_vector = end - start
    return start, dir_vector


def ray_pose_to_line(msg: PoseStamped, length: float = 300.) -> LineString:
    ray_start = np.array([msg.pose.position.x,
                          msg.pose.position.y,
                          msg.pose.position.z])
    ori = msg.pose.orientation
    rotation = kdl.Rotation.Quaternion(ori.x, ori.y, ori.z, ori.w)
    dir_vect = rotation * kdl.Vector(x=1., y=0., z=0.)
    ray_end = ray_start + vector_to_np(dir_vect)*length
    return LineString([ray_start, ray_end])


def ray_plane_intersection(msg, epsilon=1e-6):
    planeNormal = np.array([0, 0, 1])
    planePoint = np.array([0., 0., 1.0])
    rayPoint = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z])
    ori = msg.pose.orientation
    rotation = kdl.Rotation.Quaternion(ori.x, ori.y, ori.z, ori.w)
    dir_vect = rotation * kdl.Vector(x=1., y=0., z=0.)
    rayDirection = vector_to_np(dir_vect)
    ndotu = planeNormal.dot(rayDirection)
    if abs(ndotu) < epsilon:
        return None

    w = rayPoint - planePoint
    si = -planeNormal.dot(w) / ndotu
    Psi = w + si * rayDirection + planePoint
    return Psi


def segments(curve):
    return list(map(np.array, zip(curve.coords[:-1], curve.coords[1:])))
