#!/usr/bin/env python3
import math
import time
from copy import deepcopy
from sys import path

import numpy as np
import rospy
import tf
from casadi import *
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from hybrid_mpc_local_planner.srv import (MPCTrajPlanner,
                                          MPCTrajPlannerRequest,
                                          MPCTrajPlannerResponse)
from mpc_planner_core import KinMPCPathFollower
from pid import *

x_curr = 0.0
y_curr = 0.0
psi_curr = 0.0
listener = None
control_pub = None
mpc_ref_path_preprocessed_pub = None
traj_horizon = 15
traj_dt = 0.1
angle_threshold = 0.872  # 1.3 threshold of goal yaw and current yaw at the start point
pub_repeat = 2
idx_of_reference = 5
sliding_window_len = 2
state_est_feq = 10.0
preprocess_v_target = 0.5
min_v = 0.0
max_v = 2.0
min_w = -0.6
max_w = 0.6


def state_est_callback(msg):
    global state_est_freq
    loop_rate = rospy.Rate(state_est_feq)
    loop_rate.sleep()
    global x_curr, y_curr, psi_curr
    global listener
    listener.waitForTransform('/map', '/base_link',
                              rospy.Time(0), rospy.Duration(3))
    trans, rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    x_curr = trans[0]
    y_curr = trans[1]
    (roll, pitch, yaw) = euler_from_quaternion(rot)
    if yaw < 0:
        yaw = yaw + 2 * math.pi
    psi_curr = yaw
    rospy.logdebug("--------------current robot pose is---------------")
    rospy.logdebug(x_curr, y_curr, psi_curr)


def get_ref_path(trajectory, x_init, y_init, v_target=None):
    if type(trajectory) == 'list':
        return None, None, False
    elif len(trajectory[0]) > 1:
        traj = deepcopy(trajectory)
        xy_traj = traj[:, 1:3]  # full xy astar trajectory
        xy_query = np.array([[x_init, y_init]])  # vehicle's current pose
        # find the index of the closest point on the trajectory to the initial vehicle pose
        diff_dists = np.sum((xy_traj-xy_query)**2, axis=1)
        closest_traj_idx = np.argmin(diff_dists)
        return _waypoints_using_vtarget(closest_traj_idx, v_target, traj)


def _waypoints_using_vtarget(idx, v_target, traj):
    global traj_dt, traj_horizon
    # s0, cumulative dist corresponding to closest point
    start_dist = traj[idx, 3]
    dists_to_fit = [x * traj_dt * v_target +
                    start_dist for x in range(0, traj_horizon + 1)]
    x_interp = np.interp(dists_to_fit, traj[:, 3], traj[:, 1])
    y_interp = np.interp(dists_to_fit, traj[:, 3], traj[:, 2])
    stop_cmd = False
    if x_interp[0] == traj[-1, 1] and y_interp[0] == traj[-1, 2]:
        stop_cmd = True
    return x_interp, y_interp, stop_cmd


def handle_mpc_planning(req):
    global x_curr, y_curr, psi_curr, control_pub, preprocess_v_target
    global mpc_ref_path_preprocessed_pub, idx_of_reference, sliding_window_len
    T1 = time.time()

    tms = []
    Xs = []
    Ys = []
    cdists = []

    if len(req.ref_path.poses) > 1:
        prev_x = req.ref_path.poses[0].pose.position.x
        prev_y = req.ref_path.poses[0].pose.position.y
        prev_dist = 0
        for i in range(0, len(req.ref_path.poses)):
            tms.append(req.ref_path.poses[i].header.stamp.to_sec(
            ) + 1e-9 * req.ref_path.poses[i].header.stamp.to_nsec())
            Xs.append(req.ref_path.poses[i].pose.position.x)
            Ys.append(req.ref_path.poses[i].pose.position.y)
            cumulate_dist = math.sqrt((req.ref_path.poses[i].pose.position.x - prev_x) ** 2 + (
                req.ref_path.poses[i].pose.position.y - prev_y) ** 2) + prev_dist
            cdists.append(cumulate_dist)
            # update prev
            prev_x = Xs[i]
            prev_y = Ys[i]
            prev_dist = cdists[i]
        trajectory = [[]]
        trajectory = np.column_stack((tms, Xs, Ys, cdists))

        # preprocess the reference path
        T3 = time.time()
        # preprocess the reference path
        x_ref, y_ref, stop_cmd = get_ref_path(
            trajectory, x_curr, y_curr, preprocess_v_target)

        # adjust the robot to an appropriate orientation before starting mpc tracking
        if not (need_to_adjust_orientation(x_curr, y_curr, psi_curr, idx_of_reference, x_ref, y_ref, sliding_window_len)):
            # mpc path following
            kmpc = KinMPCPathFollower(
                V_MIN=min_v, V_MAX=max_v, ANGVEL_MIN=min_w, ANGVEL_MAX=max_w)
            kmpc.update_initial_condition(x_curr, y_curr, psi_curr)
            kmpc.update_reference(x_ref[1:], y_ref[1:])
            kmpc.update_previous_input(0., 0.)
            p_opts = {'expand': True}
            s_opts = {'max_cpu_time': 0.1, 'print_level': 0}
            kmpc.opti.solver('ipopt', p_opts, s_opts)

            rospy.Time.now()
            # use warm start from previous solution
            u_ws = None
            z_ws = None
            sl_ws = None
            is_opt, solve_time, u_opt, z_opt, sl_opt, z_ref = kmpc.solve(z_dv_warm_start=z_ws, u_dv_warm_start=u_ws,
                                                                         sl_dv_warm_start=sl_ws)
            # visualize the mpc trajectory
            show_mpc_traj(z_opt)
            T2 = time.time()
            return MPCTrajPlannerResponse(
                linear_acc=u_opt[0, 0],
                steer_angle=u_opt[0, 1]
            )

        else:
            # rotate in place to adjust orientation
            angular_vel = adjust_orientation(
                x_curr, y_curr, psi_curr, idx_of_reference, x_ref, y_ref)
            rospy.logdebug(
                "------------Rotate in place-----------------, %f", angular_vel)
            return MPCTrajPlannerResponse(
                linear_acc=0.0,
                steer_angle=angular_vel
            )

    else:
        rospy.logdebug(
            "the reference path received from hybrid astar is empty")
        return MPCTrajPlannerResponse(
            linear_acc=0.0,
            steer_angle=0.0
        )


def need_to_adjust_orientation(x_curr, y_curr, psi_curr, ref_idx, x_ref, y_ref, window_size):
    # get the yaw angle of the line connecting (x_curr, y_curr) and (x_ref[ref_idx], y_ref[ref_idx])
    ref_yaw_tan = 0
    count = 0
    for i in range(ref_idx-window_size, ref_idx+window_size, 1):
        ref_yaw_tan = ref_yaw_tan + (y_ref[i]-y_curr)/(x_ref[i]-x_curr)
        count = count+1
    ref_yaw_tan = ref_yaw_tan/count
    ref_yaw = math.atan(ref_yaw_tan)
    if ref_yaw < 0:
        # fourth quarter
        if (y_ref[ref_idx]-y_curr) < 0:
            ref_yaw = ref_yaw + 2*np.pi
        # second quarter
        else:
            ref_yaw = ref_yaw + np.pi
    else:
        # third quarter
        if (y_ref[ref_idx]-y_curr) < 0:
            ref_yaw = ref_yaw + np.pi
    rospy.logdebug("current astar ref path yaw angle is: %f", ref_yaw)
    orientation_diff = abs(ref_yaw - psi_curr)
    if orientation_diff > np.pi:
        orientation_diff = 2*np.pi - orientation_diff

    global angle_threshold
    if orientation_diff > angle_threshold:
        rospy.logdebug("angle diff is: %f", orientation_diff)
        return True
    else:
        return False


yaw_pid = PID(2.0, 0, 0, 1, 1, 3.5)


def adjust_orientation(x_curr, y_curr, curr_yaw, ref_idx, x_ref, y_ref):
    # get the yaw angle of the line connecting (x_curr, y_curr) and (x_ref[ref_idx], y_ref[ref_idx])
    ref_yaw = math.atan((y_ref[ref_idx]-y_curr)/(x_ref[ref_idx]-x_curr))
    if ref_yaw < 0:
        # fourth quarter
        if (y_ref[ref_idx]-y_curr) < 0:
            ref_yaw = ref_yaw + 2*np.pi
        # second quarter
        else:
            ref_yaw = ref_yaw + np.pi
    else:
        # third quarter
        if (y_ref[ref_idx]-y_curr) < 0:
            ref_yaw = ref_yaw + np.pi
    delta_yaw = ref_yaw-curr_yaw
    delta_yaw = angle_limit_pi(delta_yaw)
    ref_yaw = curr_yaw+delta_yaw
    steer_vel = yaw_pid.calcu_output(curr_yaw, ref_yaw)
    return steer_vel


def show_mpc_traj(z_opt):
    mpc_traj = Path()
    mpc_traj.header.frame_id = "/map"
    mpc_traj.header.stamp = rospy.Time.now()
    mpc_traj_pub = rospy.Publisher("mpc_traj", Path, queue_size=1)
    for i in range(z_opt.shape[0]):
        temp_pose = PoseStamped()
        temp_pose.header.frame_id = "/map"
        temp_pose.header.stamp = rospy.Time.now()
        temp_pose.pose.position.x = z_opt[i, 0]
        temp_pose.pose.position.y = z_opt[i, 1]
        temp_pose.pose.position.z = 0.0
        temp_q = Quaternion()
        temp_q = quaternion_from_euler(0, 0, z_opt[i, 2])
        temp_pose.pose.orientation.x = temp_q[0]
        temp_pose.pose.orientation.y = temp_q[1]
        temp_pose.pose.orientation.z = temp_q[2]
        temp_pose.pose.orientation.w = temp_q[3]
        mpc_traj.poses.append(temp_pose)
    mpc_traj_pub.publish(mpc_traj)


def mpc_planner_server():
    rospy.init_node('mpc_planner_server', log_level=rospy.ERROR)
    global listener, control_pub, pub_repeat, mpc_ref_path_preprocessed_pub, min_v, max_v, min_w, max_w, preprocess_v_target
    listener = tf.TransformListener()
    mpc_ref_path_preprocessed_pub = rospy.Publisher(
        "mpc_ref_path_preprocessed", Path, queue_size=1)
    control_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    s = rospy.Service('mpc_traj_planner', MPCTrajPlanner, handle_mpc_planning)
    sub_state = rospy.Subscriber(
        "/MirKinova/LIDAR/laser_scan", LaserScan, state_est_callback, queue_size=10)
    rospy.loginfo("Ready to do a MPC trajectory planning")
    rospy.spin()


if __name__ == "__main__":
    mpc_planner_server()
