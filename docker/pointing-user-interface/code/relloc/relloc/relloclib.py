#!/usr/bin/env python

import numpy as np
import scipy.optimize
from multiprocessing import Pool
from functools import partial
# https://pypi.python.org/pypi/transforms3d
import transforms3d


def make_transform(tx, ty, tz, rotz):
    ''' Creates a 4x4 rigid transform matrix with
    translation: tx, ty, tz
    rotation: rotz radians around z axis
    '''
    rot = transforms3d.axangles.axangle2mat([0, 0, 1], rotz)
    return transforms3d.affines.compose([tx, ty, tz], rot, [1, 1, 1])


def transform_points(points, tf):
    ''' Input matrix of N points (one per column) 3xN
    Outputs points in the same format '''
    points_h = np.vstack((points, np.ones((1, points.shape[1]))))
    tpoints = np.matmul(tf, points_h)
    return tpoints[0:3, :] / tpoints[3, :]


def angle_between(v1, v2):
    ''' Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    '''
    v1_u = transforms3d.utils.normalized_vector(v1)
    v2_u = transforms3d.utils.normalized_vector(v2)

    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def angle_between_vectorized(v1, v2):
    '''
    Vectorized version of angle_between: v1 and v2 are 2D matrices, one vector per row.
    Returns 1d array, one element per row
    '''
    v1_u = v1 / np.linalg.norm(v1, axis=1, keepdims=True)
    v2_u = v2 / np.linalg.norm(v2, axis=1, keepdims=True)
    dot = np.einsum("ij,ij->i", v1_u, v2_u)  # dot product for each row
    return np.arccos(np.clip(dot, -1.0, 1.0))


def distance_between_vectorized(p1, v1, p2):
    """
    Vectorized version of distance_between: p1, v1 and p2 are 2D matrices, one vector/point per row.
    Returns 1d array, one element per row
    '''
    Returns distance between a point and a line:
        - p1 is a point on the line
        - v1 is the direction vector of the line
        - p2 is the point for which we want the distance
    https://onlinemschool.com/math/library/analytic_geometry/p_line/
    """
    return np.linalg.norm(np.cross(p1-p2, v1, axis=1), axis=1)/np.linalg.norm(v1, axis=1)


def error_func(p, qc, qv, tx, ty, tz, rotz, errorf='angle'):
    ''' Transform points p using tx, ty, tz, rotz.
    For each transformed point tp, compute the angle between:
    - the direction joining qc and tp
    - the direction qv '''
    tf = make_transform(tx, ty, tz, rotz)
    tp = transform_points(p, tf)
    if errorf == 'angle':
        return list(angle_between_vectorized(qv.T, (tp - qc).T))
    else:
        return list(distance_between_vectorized(qc.T, qv.T, tp.T))


def estimate_pose(p, qc, qv, x0):
    ''' Given points in robot frame (p) and rays in human frame (qc, qv), find
    transformation parameters from human frame to robot frame that minimize the
    residual, using starting x0 as the initial solution '''

    def f(x):
        return np.mean(error_func(p, qc, qv, *x)) + max(0.0, np.linalg.norm(x[:3]) - 7.0)

    res = scipy.optimize.minimize(f, x0)
    maxerr = np.max(error_func(p, qc, qv, *res.x)) + max(0.0, np.linalg.norm(res.x[:3]) - 7.0)
    disterr = np.mean(error_func(p, qc, qv, *res.x, errorf='distance'))
    return res, maxerr, disterr


def estimate_pose_no_constraints(p, qc, qv, x0):
    ''' Given points in robot frame (p) and rays in human frame (qc, qv), find
    transformation parameters from human frame to robot frame that minimize the
    residual, using starting x0 as the initial solution '''

    def f(x):
        x[2] = 0
        return np.mean(error_func(p, qc, qv, *x))

    res = scipy.optimize.minimize(f, x0)

    maxerr = np.max(error_func(p, qc, qv, *res.x))

    return res, maxerr


def get_sample(angle_res, distance_res, p, qc, qv):
    sample = {}
    angle_errors = error_func(p, qc, qv, *angle_res.x, errorf='angle')
    distance_errors = error_func(p, qc, qv, *distance_res.x, errorf='distance')
    for error, err_col in zip(['angle', 'distance'], [angle_errors, distance_errors]):
        sample[error + '_error'] = np.mean(err_col)
        sample[error + '_std'] = np.std(err_col)
        sample[error + '_errors'] = err_col

    tf = make_transform(*angle_res.x)
    tp = transform_points(p, tf)
    robot_rays = transforms3d.utils.normalized_vector(tp-qc)
    human_rays = transforms3d.utils.normalized_vector(qv)

    for name, rays in zip(['human', 'robot'], [human_rays, robot_rays]):
        x, y, z = rays
        sample[name + '_pitch'] = np.unwrap(np.arctan2(z, np.linalg.norm([x, y])))
        # sample['data'][name + '_pitch'] = np.unwrap(np.arcsin(-y))
        sample[name + '_yaw'] = np.unwrap(np.arctan2(y, x))
        # sample['data'][name + '_pitch_rate'] = np.sign(np.diff(sample['data'][name + '_pitch'], prepend=0))
        # sample['data'][name + '_yaw_rate'] = np.sign(np.diff(sample['data'][name + '_yaw'], prepend=0))
        sample[name + '_pitch_rate'] = np.diff(sample[name + '_pitch'], prepend=0.0001)
        sample[name + '_yaw_rate'] = np.diff(sample[name + '_yaw'], prepend=0.0001)

        sample[name + '_pitch_mean'] = np.mean(sample[name + '_pitch_rate'])
        sample[name + '_pitch_std'] = np.std(sample[name + '_pitch_rate'])
        sample[name + '_yaw_mean'] = np.mean(sample[name + '_yaw_rate'])
        sample[name + '_yaw_std'] = np.std(sample[name + '_yaw_rate'])

    sample['pitch_ray_corr'] = np.corrcoef(sample['human_pitch_rate'], sample['robot_pitch_rate'])[0, 1]
    sample['yaw_ray_corr'] = np.corrcoef(sample['human_yaw_rate'], sample['robot_yaw_rate'])[0, 1]
    return sample


def angle_f(x, p, qc, qv):
    return np.mean(error_func(p, qc, qv, *x, errorf='angle'))


def distance_f(x, p, qc, qv):
    return np.mean(error_func(p, qc, qv, *x, errorf='distance'))


def estimate_pose_no_constraints_parallel(p, qc, qv, x0):
    ''' Given points in robot frame (p) and rays in human frame (qc, qv), find
    transformation parameters from human frame to robot frame that minimize the
    residual, using starting x0 as the initial solution '''

    p_angle = partial(angle_f, p=p, qc=qc, qv=qv)
    p_distance = partial(distance_f, p=p, qc=qc, qv=qv)

    pools = Pool(2)
    optimize = partial(scipy.optimize.minimize, x0=x0)
    angle_res, distance_res = pools.map(optimize, [p_angle, p_distance])
    pools.close()
    # pools.join()
    sample = get_sample(angle_res, distance_res, p, qc, qv)

    maxerr = np.max(sample['angle_error'])

    return angle_res, distance_res, sample, maxerr
