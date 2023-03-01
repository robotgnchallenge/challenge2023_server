# -- coding: utf-8 --
# !/usr/bin/env python
import sys
import math
import logging
import threading

import rospy
import numpy as np
import moveit_commander
import roslib
from select import select
from transforms3d.quaternions import mat2quat
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as Rot
from webots_ros.srv import node_get_pose, supervisor_get_from_def
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

from constants import KEY_TIMEOUT, ARM_POSE_INIT, move_bindings, speed_bindings, obj_idx_colour

roslib.load_manifest('teleop_twist_keyboard')

TwistMsg = Twist
stamped = False
twist_frame = ''
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


class PublishThread(threading.Thread):

    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/diff_drive_controller/cmd_vel',
                                         TwistMsg,
                                         queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # For new data to publish)
        if not math.isclose(rate, 0.0):
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections(
        ) == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(
                    self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception(
                "Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

    def move(self, x, y, z, th, speed, turn, times):
        count = times
        while (count > 0):
            self.update(x, y, z, th, speed, turn)
            rospy.sleep(0.5)
            count -= 1
        self.update(0, 0, 0, 0, self.speed, self.turn)
        rospy.sleep(0.5)

    def set_speed_turn_val(self, speed, turn):
        speed_val = None
        turn_val = None

        if (speed != None): speed_val = speed
        else: speed_val = self.speed
        if (turn != None): turn_val = turn
        else: turn_val = self.turn

        return speed_val, turn_val

    def front(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(1, 0, 0, 0, speed_val, turn_val, times)

    def left_front(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(1, 0, 0, 1, speed_val, turn_val, times)

    def right_front(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(1, 0, 0, -1, speed_val, turn_val, times)

    def rear(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(-1, 0, 0, 0, speed_val, turn_val, times)

    def left_rear(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(-1, 0, 0, -1, speed_val, turn_val, times)

    def right_rear(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(-1, 0, 0, 1, speed_val, turn_val, times)

    def turn_left(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(0, 0, 0, 1, speed_val, turn_val, times)

    def turn_right(self, times=1, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(0, 0, 0, -1, speed_val, turn_val, times)

    def update_speed(self, factor, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(self.x, self.y, self.z, self.th, speed_val * factor,
                      turn_val, 1)

    def update_turn(self, factor, speed=None, turn=None):
        speed_val, turn_val = self.set_speed_turn_val(speed, turn)
        self.movement(self.x, self.y, self.z, self.th, speed_val,
                      turn_val * factor, 1)


def get_key(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def set_teleop_thread():
    speed = rospy.get_param("~speed", 1.0)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0

    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)

    return pub_thread, [x, y, z, th, speed, turn]


def robot_move_front(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.front(times=step, speed=speed, turn=turn)
    return f"[FRONT-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_move_left_front(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.left_front(times=step, speed=speed, turn=turn)
    return f"[LEFT FRONT-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_move_right_front(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.right_front(times=step, speed=speed, turn=turn)
    return f"[RIGHT FRONT-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_move_rear(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.rear(times=step, speed=speed, turn=turn)
    return f"[REAR-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_move_left_rear(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.left_rear(times=step, speed=speed, turn=turn)
    return f"[LEFT REAR-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_move_right_rear(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.right_rear(times=step, speed=speed, turn=turn)
    return f"[RIGHT FRONT-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_turn_left(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.turn_left(times=step, speed=speed, turn=turn)
    return f"[TURN LEFT-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_turn_right(pub_thread, step=1, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    pub_thread.turn_right(times=step, speed=speed, turn=turn)
    return f"[TURN RIGHT-{step}]" + vels(pub_thread.speed, pub_thread.turn)


def robot_speed_update(pub_thread, factor=None, speed=None, turn=None):
    if (speed == None): speed = pub_thread.speed
    if (turn == None): turn = pub_thread.turn
    speed = speed * factor
    turn = turn * factor
    pub_thread.update_speed(factor=factor, speed=speed, turn=turn)
    return f"[SPEED UPDATE]" + vels(pub_thread.speed, pub_thread.turn)


def robot_move_keyboard(speed=0.5, turn=1.0):
    pub_thread, x, y, z, th, status = set_teleop_thread()

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed, turn))

        while (1):
            key = get_key(termios.tcgetattr(sys.stdin), KEY_TIMEOUT)
            if key in move_bindings.keys():
                x = move_bindings[key][0]
                y = move_bindings[key][1]
                z = move_bindings[key][2]
                th = move_bindings[key][3]
            elif key in speed_bindings.keys():
                speed = speed * speed_bindings[key][0]
                turn = turn * speed_bindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif (key == 'r'):
                break
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        logging.warning(e)

    finally:
        pub_thread.stop()


def robot_get_pose(robot_name='/MirKinova/supervisor'):
    rospy.wait_for_service(robot_name + '/get_from_def')
    def_service = rospy.ServiceProxy(robot_name + '/get_from_def',
                                     supervisor_get_from_def)
    origin_def = def_service("Origin", 0)
    robot_def = def_service("MobileArm", 0)

    rospy.wait_for_service(robot_name + '/node/get_pose')
    pose_service = rospy.ServiceProxy(robot_name + '/node/get_pose',
                                      node_get_pose)
    robot_pose = pose_service(origin_def.node, robot_def.node)

    trans = robot_pose.pose.translation
    rot = robot_pose.pose.rotation
    robot_trans = [trans.x, trans.y, trans.z]
    robot_rot = [rot.x, rot.y, rot.z, rot.w]

    return robot_trans, robot_rot


def init_movegroup(group_name="arm"):
    move_group = moveit_commander.MoveGroupCommander(group_name)

    return move_group


def robot_get_eef_pose(move_group):
    end_effector_link = move_group.get_end_effector_link()
    eef_pose = move_group.get_current_pose(end_effector_link).pose
    print(f"[EEF_POSE] {eef_pose}")

    return eef_pose


def robot_get_joint_value(move_group):
    joint_val = move_group.get_current_joint_values()
    print(f"[JOINT_VALUE] {joint_val}")

    return joint_val


def robot_set_joint_value(move_group, target_pose=ARM_POSE_INIT):
    try:
        msg = move_group.set_joint_value_target(target_pose)
        print(msg)
    except Exception as e:
        logging.warning(e)
        return False

    try:
        move_group.go(wait=True)
        joint_value_move = np.array(robot_get_joint_value(move_group))
        target_value = np.array(target_pose)
        joint_error = abs(np.sum(joint_value_move - target_value))
        if (joint_error < 0.01):
            return True
        else:
            return False

    except Exception as e:
        logging.warning(e)
        return False

    return True


def robot_set_pose(move_group,
                   target_pose,
                   tolerance_tarns=0.01,
                   tolerance_rot=0.5):
    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(tolerance_tarns)
    move_group.set_goal_orientation_tolerance(tolerance_rot)

    move_group.set_pose_target(target_pose)
    plan_suc, plan1, time, err_code = move_group.plan()

    if (plan_suc):
        move_group.go(wait=True)
        return True
    else:
        print(
            "[SET_POSE_ERROR]Can't find a path to targe pose, try to replanning or allow bigger tolerance"
        )
        return False


def segmap_to_img(seg_map):
    seg_img = np.zeros((seg_map.shape[0], seg_map.shape[1], 3))
    for i in range(seg_map.shape[0]):
        for j in range(seg_map.shape[1]):
            seg_img[i][j] = obj_idx_colour[seg_map[i][j]]

    return seg_img


def get_sim_cam_intrinsic(width, height):
    cx = width / 2
    cy = height / 2
    fx = width / (2 * math.tan(1 / 2))
    fy = fx

    return cx, cy, fx, fy
