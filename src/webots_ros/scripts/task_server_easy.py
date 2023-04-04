import sys
import os
import time
import math
import copy
import struct
import random
import socketserver
import pickle
import multiprocessing
from sys import argv
from pathlib import Path

sys.path.append('../')

import tf
import cv2
import rospy
import actionlib
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from webots_ros.srv import node_get_pose, supervisor_get_from_def
from webots_ros.srv import set_int, get_bool, save_image
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

from grasp_quality import force_closure
from controller import get_sim_cam_intrinsic, robot_get_pose, init_movegroup, robot_get_eef_pose, robot_get_joint_value, robot_set_joint_value, robot_set_pose
from math_utils import pose_4x4_to_pos_quat

from config.socket_config import socket_conf

bridge = CvBridge()

ADDR, HEADER, FORMAT, DISCONNECT_MSG = socket_conf()

client_set = set()
scene = -1
current_state = 0
obj_list = set()
scene_set = set()
target_obj_idx = -1
target_loc = ""
goal_loc = ""
target_obj = ""
grasp_pose_global = None
home_dir = os.getenv("HOME")

msg_server = " "
goal_index = -1
py_file, eval_inst = argv

score_e = 0
score_g = 0
task_time = 0
nav_time = 0


class TaskSocketServer(socketserver.StreamRequestHandler):

    def set_status(self):
        global current_state
        self.task_state = [
            "INITIAL", "START_NAV", "START_GSP", "START_NAV_GOAL",
            "TASK_FINISH"
        ]
        self.room_area = ["Bedroom", "Living room", "Parlor", "Kitchen"]
        self.grasp_obj = [
            'Crackers', 'Sugar', 'Can', 'Mustard', 'Spam', 'Banana', 'Bowl',
            'Mug', 'Drill', 'Scissor', 'Strawberry', 'Apple', 'Lemon', 'Peach',
            'Pear', 'Orange', 'Plum', 'Screwdriver', 'Ball', 'Toy'
        ]

        self.obj_map = {
            "Crackers": "",
            "Sugar": "",
            "Can": "",
            "Mustard": "",
            "Spam": "",
            "Banana": "",
            "Bowl": "",
            "Mug": "",
            "Drill": "",
            "Scissor": "",
            "Strawberry": "",
            "Apple": "",
            "Lemon": "",
            "Peach": "",
            "Pear": "",
            "Orange": "",
            "Plum": "",
            "Screwdriver": "",
            "Ball": "",
            "Toy": ""
        }

        self.seg_colour_map = {
            (205, 133, 63): 101,
            (192, 192, 192): 102,
            (255, 165, 0): 103,
            (50, 205, 50): 104,
            (70, 130, 180): 105,
            (186, 85, 211): 106,
            (220, 20, 60): 107,
            (255, 215, 0): 108,
            (178, 34, 34): 109,
            (255, 250, 240): 110,
            (255, 255, 0): 111,
            (75, 0, 130): 0,
            (0, 128, 128): 1,
            (173, 255, 47): 2,
            (255, 250, 205): 3,
            (233, 150, 122): 4,
            (218, 165, 32): 5,
            (143, 188, 143): 6,
            (255, 0, 0): 7,
            (95, 158, 160): 8,
            (173, 216, 230): 9,
            (255, 182, 193): 11,
            (255, 69, 0): 12,
            (240, 230, 140): 13,
            (255, 240, 245): 14,
            (127, 255, 0): 15,
            (255, 140, 0): 16,
            (221, 160, 221): 17,
            (85, 107, 47): 19,
            (0, 255, 255): 21,
            (188, 255, 0): 25,
            (128, 42, 42): 100,
            (0, 0, 0): -1
        }

        self.table_area_list = np.load('../worlds/scene_info/table_info.npy')

        self.obj_list = set()
        self.scene = set()
        self.grasp_obj_idx = [
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 19, 21,
            25
        ]

        self.obj_pose = np.zeros((len(self.grasp_obj), 7))
        self.state = self.task_state[current_state]
        self.robot_sup_name = '/MirKinova/supervisor'
        self.robot_name = '/MirKinova'
        self.client_state = "Initial"
        self.gripper_config = [0.1, 0.02, 0.07]
        self.data_root = os.path.join(home_dir, "dataset/graspnet")

        self.msg_instruction = "Instruction"
        self.msg_nav_succ = "Navigation succeeded"
        self.msg_nav_fail = "Navigation failed"
        self.msg_gsp_succ = "Grasped succeeded"
        self.msg_gsp_fail = "Grasped failed"

        self.livingroom_area = [-6, 0, -6, 0.38]
        self.kitchen_area = [-6, 0, 0.94, 5.8]
        self.parlor_area = [0, 5.8, 0.55, 5.83]
        self.bedroom_area = [0, 5.8, -6, 0.38]

        self.place_area = self.table_area_list
        self.seg_enable_hand = False
        self.seg_enable_head = False

        self.sig_started = False
        self.sig_end = False
        self.arm_radius = 1.5

        self.instruction = ""

        rospy.init_node('grasp_tf', anonymous=True)
        self.listener = tf.TransformListener(True, rospy.Duration(15.0))
        rospy.set_param('/webots_grasp', -1)
        rospy.set_param('/webots_place', -1)

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

    def go_to(self, pose: PoseStamped) -> bool:
        print(f"[{self.client_address}] Start moving to targe pose ...")
        self.move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose.pose.position.x
        goal.target_pose.pose.position.y = pose.pose.position.y
        goal.target_pose.pose.orientation.z = pose.pose.orientation.z
        goal.target_pose.pose.orientation.w = pose.pose.orientation.w
        self.move_base_client.send_goal(goal)

        finished_within_time = self.move_base_client.wait_for_result(
            rospy.Duration(60))
        if not finished_within_time:
            self.move_base_client.cancel_goal()
            print(f"[{self.client_address}]" +
                  "Timedout occurrend, go to pose failed.")
        else:
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                print(f"[{self.client_address}]" +
                      "Go to target pose successfully.")

        return finished_within_time

    def get_obj_area(self, trans):
        global scene_set
        src_area = -1

        scene = scene_set.pop()
        scene_set.add(scene)
        area = self.place_area[scene]
        for i in range(area.shape[0]):
            if (trans[0] >= area[i][0][0] and trans[0] <= area[i][0][1]
                    and trans[1] >= area[i][1][0]
                    and trans[1] <= area[i][1][1]):
                src_area = i
                break

        if (src_area != -1):
            src_area = self.room_area[src_area]
        else:
            src_area = "Other"
        return src_area

    def obj_pose_handler(self, msg):
        global obj_list
        global scene_set
        obj_info = msg.split(" ")
        obj_src = obj_info[1].split('[')[1].split(']')[0]
        obj_src_pos = self.grasp_obj.index(obj_src)
        obj_src_idx = self.grasp_obj_idx[obj_src_pos]

        obj_pose_info = "".join(obj_info[3:])[1:-1]
        obj_src_pose = list(obj_pose_info.split(','))
        obj_src_pose = np.array([float(p) for p in obj_src_pose]).reshape(4, 4)

        (obj_trans, obj_rot) = pose_4x4_to_pos_quat(obj_src_pose)

        obj_pose = np.concatenate((obj_trans, obj_rot), axis=0)

        scene_set.add(int(obj_info[2].split('[')[1].split(']')[0]))
        src_area = self.get_obj_area(obj_trans)
        self.obj_map[obj_src] = src_area

        self.obj_list.add(obj_src_idx)
        obj_list.add(obj_src_idx)

        return obj_pose, obj_src_idx

    def get_robot_pose(self):
        rospy.wait_for_service(self.robot_sup_name + '/get_from_def')
        def_service = rospy.ServiceProxy(self.robot_sup_name + '/get_from_def',
                                         supervisor_get_from_def)
        origin_def = def_service("Origin", 0)
        robot_def = def_service("MobileArm", 0)

        rospy.wait_for_service(self.robot_sup_name + '/node/get_pose')
        pose_service = rospy.ServiceProxy(
            self.robot_sup_name + '/node/get_pose', node_get_pose)
        robot_pose = pose_service(origin_def.node, robot_def.node)

        trans = robot_pose.pose.translation
        robot_trans = [trans.x, trans.y, trans.z]

        return robot_trans

    def rectangle_distance(self, pose_range, robot_pose):
        center_x = (pose_range[0][0] + pose_range[0][1]) / 2
        center_y = (pose_range[1][0] + pose_range[1][1]) / 2
        w = abs(pose_range[0][1] - pose_range[0][0])
        h = abs(pose_range[1][0] - pose_range[1][1])

        dx = max(abs(robot_pose[0] - center_x) - w / 2, 0)
        dy = max(abs(robot_pose[1] - center_y) - h / 2, 0)

        return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

    def verify_workspace(self, robot_pose, goal_index):
        global scene_set
        scene = scene_set.pop()
        scene_set.add(scene)
        dist = self.rectangle_distance(self.place_area[scene][goal_index],
                                       robot_pose[0:2])
        if (dist < self.arm_radius):
            return True
        else:
            return False

    def check_robot_area(self, trans):
        robot_area = ""
        if (self.livingroom_area[0] <= trans[0] <= self.livingroom_area[1]
                and self.livingroom_area[2] <= trans[1] <=
                self.livingroom_area[3]):
            robot_area = self.room_area[1]
        elif (self.kitchen_area[0] <= trans[0] <= self.kitchen_area[1]
              and self.kitchen_area[2] <= trans[1] <= self.kitchen_area[3]):
            robot_area = self.room_area[3]
        elif (self.parlor_area[0] <= trans[0] <= self.parlor_area[1]
              and self.parlor_area[2] <= trans[1] <= self.parlor_area[3]):
            robot_area = self.room_area[2]
        elif (self.bedroom_area[0] <= trans[0] <= self.bedroom_area[1]
              and self.bedroom_area[2] <= trans[1] <= self.bedroom_area[3]):
            robot_area = self.room_area[0]
        else:
            robot_area = ""
        return robot_area

    def check_nav_goal(self, target_loc):
        trans = self.get_robot_pose()
        robot_area = self.check_robot_area(trans)

        if (target_loc == robot_area):
            return True, robot_area
        else:
            return False, robot_area

    def check_nav_result(self, obj_pose):
        trans_robot = np.array(self.get_robot_pose())
        trans_obj = obj_pose[0:3]

        dist_obj = np.linalg.norm(trans_robot - trans_obj)

        if (dist_obj < self.arm_radius):
            return True
        else:
            return False

    def generate_inst(self):
        global obj_list
        obj_list = list(self.obj_list)
        obj_list = list(obj_list)
        obj_index = random.choice(obj_list)
        obj_src_idx = self.grasp_obj_idx.index(obj_index)

        target_obj = self.grasp_obj[obj_src_idx]
        target_loc = self.obj_map[target_obj]
        target_idx = self.grasp_obj_idx[obj_src_idx]

        goal_opt = copy.deepcopy(self.room_area)
        if (target_loc in goal_opt):
            goal_opt.remove(target_loc)
        goal_loc = random.choice(goal_opt)
        goal_index = self.room_area.index(goal_loc)

        inst = "Go to the {}, pick the {} and place in {}".format(
            target_loc, target_obj, goal_loc)

        return inst, target_loc, target_obj, target_idx, goal_loc, goal_index

    def generate_rand_position(self, region):
        rand_pose = [0.0, 0.0, region[2][0]]
        rand_pose[0] = random.uniform(region[0][0], region[0][1])
        rand_pose[1] = random.uniform(region[1][0], region[1][1])
        return rand_pose

    def subscribe_depth_ros(self, topic):
        front_img = rospy.wait_for_message(topic, Image)
        depth = bridge.imgmsg_to_cv2(front_img, "32FC1")

        return depth

    def subscribe_laser_ros(self, topic):
        laser = rospy.wait_for_message(topic, LaserScan)
        laser_time = str(rospy.Time.now())
        ranges = np.array(laser.ranges)

        return ranges, laser_time

    def subscribe_pc_ros(self, cam_mode, rgb_topic, pc_topic):
        segmap, rgb, depth, pc_full = None, None, None, []

        rgb_time = ''
        pc_time = ''

        rgb_im = rospy.wait_for_message(rgb_topic, Image)
        rgb_time = str(rospy.Time.now())
        rgb = bridge.imgmsg_to_cv2(rgb_im, rgb_im.encoding)

        pc_sim = rospy.wait_for_message(pc_topic, PointCloud2)
        pc_time = str(rospy.Time.now())
        pc = pc2.read_points(pc_sim,
                             field_names=("x", "y", "z"),
                             skip_nans=True)

        if (cam_mode == "Sim"):
            width = 1280
            height = 720

            cx, cy, fx, fy = get_sim_cam_intrinsic(width, height)

            pc_sim = rospy.wait_for_message(pc_topic, PointCloud2)
            pc = pc2.read_points(pc_sim,
                                 field_names=("x", "y", "z"),
                                 skip_nans=True)

            for p in pc:
                pc_full.append([p[0], p[1], p[2]])

        cam_K = np.zeros((3, 3))
        cam_K[0, 0] = fx
        cam_K[0, 2] = cx
        cam_K[1, 1] = fy
        cam_K[1, 2] = cy

        pc_full = np.array(pc_full)

        sample_pts = 20000
        if (len(pc_full) != 0):
            choice = np.random.choice(
                len(pc_full),
                sample_pts,
                replace=False if len(pc_full) >= sample_pts else True)
            pc_full = pc_full[choice]
        else:
            pc_full = pc_full

        return segmap, rgb, depth, cam_K, pc_full, rgb_time, pc_time

    def movebase_to_pose(self, msg):
        inst = msg.split('[CONTROL]')[-1]
        inst_ele = inst.split(' ')

        if ('--trans' in inst_ele):
            idx = inst_ele.index('--trans')
            trans = "".join(inst_ele[idx + 1:])
            trans = trans.split('[')[1].split(']')[0].split(',')
            trans = [float(ele) for ele in trans]
        if ('--rot' in inst_ele):
            idx = inst_ele.index('--rot')
            rot = "".join(inst_ele[idx + 1:])
            rot = rot.split('[')[1].split(']')[0].split(',')
            rot = [float(ele) for ele in rot]

        tf_pose = PoseStamped()
        tf_pose.header.frame_id = "map"
        tf_pose.header.stamp = rospy.Time.now()
        tf_pose.pose.position.x = trans[0]
        tf_pose.pose.position.y = trans[1]
        tf_pose.pose.position.z = trans[2]
        tf_pose.pose.orientation.x = rot[0]
        tf_pose.pose.orientation.y = rot[1]
        tf_pose.pose.orientation.z = rot[2]
        tf_pose.pose.orientation.w = rot[3]

        while not self.go_to(tf_pose):
            continue

        state_info = "[CONTROL] Finish move mobile to target pose"
        self.request.sendall(state_info.encode(FORMAT))

        return

    def tf_handler(self, msg):
        inst = msg.split('[TF]')[-1]
        inst_ele = inst.split(' ')

        pose = None
        source = ''
        target = ''
        trans = [0, 0, 0]
        rot = [0, 0, 1, 0]
        frame = ''

        if ('--source' in inst_ele):
            idx = inst_ele.index('--source')
            source = inst_ele[idx + 1]
        if ('--target' in inst_ele):
            idx = inst_ele.index('--target')
            target = inst_ele[idx + 1]
        if ('--trans' in inst_ele):
            idx = inst_ele.index('--trans')
            trans = "".join(inst_ele[idx + 1:])
            trans = trans.split('[')[1].split(']')[0].split(',')
            trans = [float(ele) for ele in trans]
        if ('--rot' in inst_ele):
            idx = inst_ele.index('--rot')
            rot = "".join(inst_ele[idx + 1:])
            rot = rot.split('[')[1].split(']')[0].split(',')
            rot = [float(ele) for ele in rot]
        if ('--frame' in inst_ele):
            idx = inst_ele.index('--frame')
            frame = inst_ele[idx + 1]
        if ('--time' in inst_ele):
            idx = inst_ele.index('--time')
            time = "".join(inst_ele[idx + 1:])
            time = time.split('[')[1].split(']')[0].split(',')
            time = [int(ele) for ele in time]
            ros_time = rospy.Time(time[0], time[1])
        else:
            ros_time = rospy.Time(0)

        if (not source.startswith('/')):
            source = '/' + source
        if (not target.startswith('/')):
            target = '/' + target

        if (inst_ele[0] == "LOOKUP_TRANSFORM"):
            self.listener.waitForTransform(source, target, ros_time,
                                           rospy.Duration(4.0))
            (trans,
             rot) = self.listener.lookupTransform(source, target, ros_time)

            pose_list = trans + rot
            pose = np.array(pose_list)
        elif (inst_ele[0] == "TRANSFORM_POSE"):
            tf_pose = PoseStamped()
            tf_pose.header.frame_id = frame
            tf_pose.pose.position.x = trans[0]
            tf_pose.pose.position.y = trans[1]
            tf_pose.pose.position.z = trans[2]
            tf_pose.pose.orientation.x = rot[0]
            tf_pose.pose.orientation.y = rot[1]
            tf_pose.pose.orientation.z = rot[2]
            tf_pose.pose.orientation.w = rot[3]

            self.listener.waitForTransform(source, frame, ros_time,
                                           rospy.Duration(4.0))
            pose_transform = self.listener.transformPose(source, tf_pose)
            trans = [
                pose_transform.pose.position.x, pose_transform.pose.position.y,
                pose_transform.pose.position.z
            ]
            rot = [
                pose_transform.pose.orientation.x,
                pose_transform.pose.orientation.y,
                pose_transform.pose.orientation.z,
                pose_transform.pose.orientation.w
            ]
            pose = trans + rot
            pose = np.array(pose)
        else:
            pose = np.array([-1])
        return pose

    def enable_seg(self, cam_frame):
        try:
            rospy.wait_for_service(os.path.join(self.robot_name, cam_frame,
                                                'recognition_enable'),
                                   timeout=5)
        except Exception:
            self.ros_info_exception_handler()
        def_service = rospy.ServiceProxy(
            os.path.join(self.robot_name, cam_frame, 'recognition_enable'),
            set_int)
        def_service(10)
        try:
            rospy.wait_for_service(os.path.join(
                self.robot_name, cam_frame, 'recognition_enable_segmentation'),
                                   timeout=5)
        except Exception:
            self.ros_info_exception_handler()
        def_service = rospy.ServiceProxy(
            os.path.join(self.robot_name, cam_frame,
                         'recognition_enable_segmentation'), get_bool)
        def_service(10)

        if (cam_frame == 'kinect_color'):
            self.seg_enable_head = True
        else:
            self.seg_enable_hand = True

    def save_segmap(self, cam_frame):
        try:
            rospy.wait_for_service(os.path.join(
                self.robot_name, cam_frame,
                'recognition_save_segmentation_image'),
                                   timeout=5)
        except Exception:
            self.ros_info_exception_handler()
        def_service = rospy.ServiceProxy(
            os.path.join(self.robot_name, cam_frame,
                         'recognition_save_segmentation_image'), save_image)
        def_service(
            os.path.join(Path().absolute(), 'segmap_' + cam_frame + '.png'),
            100)
        seg_time = str(rospy.Time.now())

        seg_img = cv2.imread(
            os.path.join(Path().absolute(), 'segmap_' + cam_frame + '.png'))
        if (seg_img.all() is None):
            return -1 * np.ones((640, 480)), seg_time
        seg_map = np.zeros((seg_img.shape[0], seg_img.shape[1]))
        for i in range(seg_img.shape[0]):
            for j in range(seg_img.shape[1]):
                cur_pixel = seg_img[i][j]
                cur_pixel[0], cur_pixel[2] = cur_pixel[2], cur_pixel[0]
                cur_pixel = tuple(cur_pixel)
                seg_label = self.seg_colour_map[cur_pixel]
                seg_map[i][j] = seg_label
        return seg_map, seg_time

    def sensor_info_handler(self, msg):
        rgb_flag = False
        depth_flag = False
        seg_flag = False
        laser_flag = False
        pc_flag = False
        hand_flag = False
        head_flag = False
        info_count = 0

        sensor_info = {}

        if (msg == "REQUEST_SENSOR_INFO"):
            rgb_flag = True
            depth_flag = True
            seg_flag = True
            laser_flag = True
            pc_flag = True
            hand_flag = True
            head_flag = True

        inst_ele = msg.split(' ')
        if ('--rgb' in inst_ele):
            rgb_flag = True
            info_count += 1
        if ('--depth' in inst_ele):
            depth_flag = True
            info_count += 1
        if ('--seg' in inst_ele):
            seg_flag = True
            info_count += 1
        if ('--laser' in inst_ele):
            laser_flag = True
            info_count += 1
        if ('--pc' in inst_ele):
            pc_flag = True
            info_count += 1
        if ('--hand' in inst_ele):
            hand_flag = True
        if ('--head' in inst_ele):
            head_flag = True
        if (hand_flag is True and head_flag is True):
            info_count *= 2

        if (pc_flag or rgb_flag):
            if (pc_flag):
                if (hand_flag):
                    _, rgb, _, cam_K, pc_full, rgb_time, pc_time = self.subscribe_pc_ros(
                        "Sim", "/MirKinova/hand_camera/image",
                        "/MirKinova/hand_depth/points")
                    if (rgb_flag):
                        sensor_info.update({
                            'pc_full_hand': {
                                'data': pc_full,
                                'time': pc_time
                            },
                            'rgb_hand': {
                                'data': rgb,
                                'time': rgb_time
                            }
                        })
                    else:
                        sensor_info.update({
                            'pc_full_hand': {
                                'data': pc_full,
                                'time': pc_time
                            }
                        })
                if (head_flag):
                    _, kinect_rgb, _, kinect_cam_K, kinect_pc_full, kinect_rgb_time, kinect_pc_time = self.subscribe_pc_ros(
                        "Sim", "/MirKinova/kinect_color/image",
                        "/MirKinova/kinect_range/points")
                    if (rgb_flag):
                        sensor_info.update({
                            'pc_full_head': {
                                'data': kinect_pc_full,
                                'time': kinect_pc_time
                            },
                            'rgb_head': {
                                'data': kinect_rgb,
                                'time': kinect_rgb_time
                            }
                        })
                    else:
                        sensor_info.update({
                            'pc_full_head': {
                                'data': kinect_pc_full,
                                'time': kinect_pc_time
                            }
                        })
            else:
                if (hand_flag):
                    rgb_im_hand = rospy.wait_for_message(
                        "/MirKinova/hand_camera/image", Image)
                    rgb_time = str(rospy.Time.now())
                    rgb_hand = bridge.imgmsg_to_cv2(rgb_im_hand,
                                                    rgb_im_hand.encoding)
                    sensor_info.update(
                        {'rgb_hand': {
                            'data': rgb_hand,
                            'time': rgb_time
                        }})
                if (head_flag):
                    rgb_im_head = rospy.wait_for_message(
                        "/MirKinova/kinect_color/image", Image)
                    rgb_time = str(rospy.Time.now())
                    rgb_head = bridge.imgmsg_to_cv2(rgb_im_head,
                                                    rgb_im_head.encoding)
                    sensor_info.update(
                        {'rgb_head': {
                            'data': rgb_head,
                            'time': rgb_time
                        }})
        if (depth_flag):
            if (hand_flag):
                depth_im_hand = rospy.wait_for_message(
                    "/MirKinova/hand_depth/range_image", Image)
                depth_time = str(rospy.Time.now())
                depth_hand = bridge.imgmsg_to_cv2(depth_im_hand,
                                                  depth_im_hand.encoding)
                sensor_info.update(
                    {'depth_hand': {
                        'data': depth_hand,
                        'time': depth_time
                    }})
            if (head_flag):
                depth_im_head = rospy.wait_for_message(
                    "/MirKinova/kinect_range/range_image", Image)
                depth_time = str(rospy.Time.now())
                depth_head = bridge.imgmsg_to_cv2(depth_im_head,
                                                  depth_im_head.encoding)
                sensor_info.update(
                    {'depth_head': {
                        'data': depth_head,
                        'time': depth_time
                    }})
        if (seg_flag):
            if (hand_flag):
                if (self.seg_enable_hand is False):
                    self.enable_seg("hand_camera")
                seg_hand, seg_time = self.save_segmap("hand_camera")
                sensor_info.update(
                    {'seg_hand': {
                        'data': seg_hand,
                        'time': seg_time
                    }})
            if (head_flag):
                if (self.seg_enable_head is False):
                    self.enable_seg("kinect_color")
                seg_head, seg_time = self.save_segmap("kinect_color")
                sensor_info.update(
                    {'seg_head': {
                        'data': seg_head,
                        'time': seg_time
                    }})
        if (laser_flag):
            laser_ranges, laser_time = self.subscribe_laser_ros(
                "/MirKinova/LIDAR/laser_scan")
            sensor_info.update(
                {'laser': {
                    'data': laser_ranges,
                    'time': laser_time
                }})

        data = pickle.dumps(sensor_info)
        size = sys.getsizeof(data)
        header = struct.pack("i", size)
        self.request.sendall(header)
        self.request.sendall(data)

    def map_handler(self, msg):
        map_flag = False
        global_flag = False
        local_flag = False

        map_info = {}

        if (msg == "REQUEST_MAP"):
            map_flag = True
            global_flag = True
            local_flag = True

        inst_ele = msg.split(' ')
        if ('--map' in inst_ele):
            map_flag = True
        if ('--global' in inst_ele):
            global_flag = True
        if ('--local' in inst_ele):
            local_flag = True

        if (map_flag):
            map_msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=5)
            ros_time = rospy.Time.now()
            map_info.update({'map': {'data': map_msg, 'time': ros_time}})
        if (global_flag):
            global_msg = rospy.wait_for_message(
                "/move_base/global_costmap/costmap", OccupancyGrid, timeout=5)
            ros_time = rospy.Time.now()
            map_info.update(
                {'global_map': {
                    'data': global_msg,
                    'time': ros_time
                }})
        if (local_flag):
            local_msg = rospy.wait_for_message(
                "/move_base/local_costmap/costmap", OccupancyGrid, timeout=5)
            ros_time = rospy.Time.now()
            map_info.update(
                {'local_map': {
                    'data': local_msg,
                    'time': ros_time
                }})

        data = pickle.dumps(map_info)
        size = sys.getsizeof(data)
        header = struct.pack("i", size)
        self.request.sendall(header)
        self.request.sendall(data)

    def moveit_control_handler(self, msg):
        inst = msg.split('[MOVEIT]')[-1]
        inst_ele = inst.split(' ')
        pose = None
        if ('--pose' in inst_ele):
            idx = inst_ele.index('--pose')
            pose = "".join(inst_ele[idx + 1:])
            pose = pose.split('[')[1].split(']')[0].split(',')
            pose = [float(ele) for ele in pose]
        if ('--tolerance_tarns' in inst_ele):
            idx = inst_ele.index('--tolerance_tarns')
            tolerance_tarns = float(inst_ele[idx + 1])
        if ('--tolerance_rot' in inst_ele):
            idx = inst_ele.index('--tolerance_rot')
            tolerance_rot = float(inst_ele[idx + 1])

        robot_info = ""
        get_info = False
        if (inst_ele[0] == "GET_ARM_JOINT_VALUE"):
            get_info = True
            group = init_movegroup("arm")
            robot_info = robot_get_joint_value(group)
        elif (inst_ele[0] == "GET_GRIPPER_JOINT_VALUE"):
            get_info = True
            group = init_movegroup("gripper")
            robot_info = robot_get_joint_value(group)
        elif (inst_ele[0] == "GET_ROBOT_POSE"):
            get_info = True
            trans, rot = robot_get_pose()
            robot_info = trans + rot
        elif (inst_ele[0] == "GET_EEF_POSE"):
            get_info = True
            group = init_movegroup("arm")
            pose = robot_get_eef_pose(group)
            trans = [pose.position.x, pose.position.y, pose.position.z]
            rot = [
                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w
            ]
            robot_info = trans + rot
        elif (inst_ele[0] == "SET_ARM_JOINT_VALUE"):
            group = init_movegroup("arm")
            robot_info = robot_set_joint_value(group, pose)
            if (robot_info is True):
                robot_info = "[SET_ARM_JOINT_VALUE] SUCCESS"
            else:
                robot_info = "[SET_ARM_JOINT_VALUE] FAILURE"
        elif (inst_ele[0] == "SET_GRIPPER_JOINT_VALUE"):
            group = init_movegroup("gripper")
            robot_info = robot_set_joint_value(group, pose)
            if (robot_info is True):
                robot_info = "[SET_GRIPPER_JOINT_VALUE] SUCCESS"
            else:
                robot_info = "[SET_GRIPPER_JOINT_VALUE] FAILURE"
        elif (inst_ele[0] == "SET_EEF_POSE"):
            group = init_movegroup("arm")
            robot_info = robot_set_pose(group, pose, tolerance_tarns,
                                        tolerance_rot)
            if (robot_info is True):
                robot_info = "[SET_EEF_POSE] SUCCESS"
            else:
                robot_info = "[SET_EEF_POSE] FAILURE"
        elif (inst_ele[0] == "SET_PANTILT_JOINT_VALUE"):
            group = init_movegroup("pantilt")
            robot_info = robot_set_joint_value(group, pose)
            if (robot_info is True):
                robot_info = "[SET_PANTILT_JOINT_VALUE] SUCCESS"
            else:
                robot_info = "[SET_PANTILT_JOINT_VALUE] FAILURE"
        elif (inst_ele[0] == "SET_ARMBASE_JOINT_VALUE"):
            group = init_movegroup("armbase")
            robot_info = robot_set_joint_value(group, pose)
            if (robot_info is True):
                robot_info = "[SET_PANTILT_JOINT_VALUE] SUCCESS"
            else:
                robot_info = "[SET_PANTILT_JOINT_VALUE] FAILURE"

        if (get_info is True):
            robot_info = np.array(robot_info)
            data = pickle.dumps(robot_info)
            size = sys.getsizeof(data)
            header = struct.pack("i", size)
            self.request.sendall(header)
            self.request.sendall(data)
        else:
            self.request.sendall(robot_info.encode(FORMAT))

    def ros_info_exception_handler(self):
        data = pickle.dumps(np.array([-1]))
        size = sys.getsizeof(data)
        header = struct.pack("i", size)
        self.request.sendall(header)
        self.request.sendall(data)

    def handle(self):
        global current_state
        global scene_set
        global target_obj_idx
        global target_loc
        global goal_loc
        global target_obj
        global grasp_pose_global
        global msg_server
        global goal_index
        global score_e
        global score_g
        global task_time
        global nav_time

        print(f"[{self.client_address}] NEW CONNECTION connected.")
        connected = True
        self.state = self.task_state[current_state]

        while (connected):
            header = self.request.recv(4)

            try:
                size = struct.unpack('i', header)
                data = self.request.recv(size[0])
            except Exception:
                pass

            try:
                msg = data.decode(FORMAT)
                if (str.isnumeric(msg)):
                    grasp_pose = []
            except Exception:
                data = pickle.loads(data)
                grasp_pose.append(data)
                grasp_pose_global = grasp_pose
                state_info = f"[{self.client_address}][Grasp] Server recieved grasp data."
                self.request.sendall(state_info.encode(FORMAT))
            if (msg.startswith("[WEBOTS_INFO]")):
                client_set.add(self)
                obj_src_pose, obj_src_idx = self.obj_pose_handler(msg)
                obj_idx = self.grasp_obj_idx.index(obj_src_idx)
                self.obj_pose[obj_idx] = np.array(obj_src_pose)
            elif (msg == "REQUEST_CAMERA_INFO"):
                _, rgb, _, cam_K, pc_full, _, _ = self.subscribe_pc_ros(
                    "Sim", "/MirKinova/hand_camera/image",
                    "/MirKinova/hand_depth/points")
                camera_info = {'rgb': rgb, 'cam_K': cam_K, 'pc_full': pc_full}
                data = pickle.dumps(camera_info)
                size = sys.getsizeof(data)
                header = struct.pack("i", size)
                self.request.sendall(header)
                self.request.sendall(data)
            elif (msg == "REQUEST_FRONT_DEPTH"):
                front_depth = self.subscribe_depth_ros(
                    "/MirKinova/depth_camera/range_image")
                depth_info = {'depth': front_depth}
                data = pickle.dumps(depth_info)
                size = sys.getsizeof(data)
                header = struct.pack("i", size)
                self.request.sendall(header)
                self.request.sendall(data)
            elif (msg == "REQUEST_LASER"):
                laser_ranges = self.subscribe_laser_ros(
                    "/MirKinova/LIDAR/laser_scan")
                ranges_info = {'laser': laser_ranges}
                data = pickle.dumps(ranges_info)
                size = sys.getsizeof(data)
                header = struct.pack("i", size)
                self.request.sendall(header)
                self.request.sendall(data)
            elif (msg == "REQUEST_SEGMAP_HEAD"
                  or msg == "REQUEST_SEGMAP_HAND"):
                cam_type = msg.split('_')[-1]
                cam_frame = ""
                if (cam_type == "HEAD"):
                    cam_frame = "kinect_color"
                    if (self.seg_enable_head is False):
                        self.enable_seg(cam_frame)
                else:
                    cam_frame = "hand_camera"
                    if (self.seg_enable_hand is False):
                        self.enable_seg(cam_frame)

                try:
                    rospy.wait_for_service(os.path.join(
                        self.robot_name, cam_frame,
                        'recognition_save_segmentation_image'),
                                           timeout=5)
                except Exception:
                    self.ros_info_exception_handler()
                def_service = rospy.ServiceProxy(
                    os.path.join(self.robot_name, cam_frame,
                                 'recognition_save_segmentation_image'),
                    save_image)
                def_service(os.path.join(Path().absolute(), 'segmap.png'), 100)

                seg_img = cv2.imread(
                    os.path.join(Path().absolute(), 'segmap.png'))
                seg_map = np.zeros((seg_img.shape[0], seg_img.shape[1]))

                for i in range(seg_img.shape[0]):
                    for j in range(seg_img.shape[1]):
                        cur_pixel = seg_img[i][j]
                        cur_pixel[0], cur_pixel[2] = cur_pixel[2], cur_pixel[0]
                        cur_pixel = tuple(cur_pixel)
                        seg_label = self.seg_colour_map[cur_pixel]
                        seg_map[i][j] = seg_label

                data = pickle.dumps(seg_map)
                size = sys.getsizeof(data)
                header = struct.pack("i", size)
                self.request.sendall(header)
                self.request.sendall(data)
            elif (msg.startswith("REQUEST_SENSOR_INFO")):
                self.sensor_info_handler(msg)
            elif (msg.startswith("REQUEST_MAP")):
                self.map_handler(msg)
            elif (msg.startswith("[CONTROL]")):
                inst = msg.split('[CONTROL]')[-1]
                inst_ele = inst.split(' ')
                if (inst_ele[0] == "START_KEYBOARD_CONTROL"):
                    state_info = f"[{self.client_address}][INIT_SUCCESS] KEYBOARD_CONTROL"
                    self.request.sendall(state_info.encode(FORMAT))
                elif (inst_ele[0] == "MOVE_TO_TARGET_POSE"):
                    self.movebase_to_pose(msg)
            elif (msg.startswith("[MOVEIT]")):
                self.moveit_control_handler(msg)
            elif (msg.startswith("[TF]")):
                tf_pose = self.tf_handler(msg)
                data = pickle.dumps(tf_pose)
                size = sys.getsizeof(data)
                header = struct.pack("i", size)
                self.request.sendall(header)
                self.request.sendall(data)
            elif (msg == "REQUEST_FOR_INST_RANDOM"):
                print(f"[{self.client_address}]" + msg)
                if (self.state == "INITIAL"):
                    msg_server, target_loc, target_obj, target_obj_idx, goal_loc, goal_index = self.generate_inst(
                    )
                    current_state += 1
                    self.state = self.task_state[current_state]
                else:
                    state_info = f"[{self.client_address}][TASK_FAILED] Current task state is not match, expected state is START_NAV but got {self.state}"
                    self.request.sendall(state_info.encode(FORMAT))
                    msg_server = DISCONNECT_MSG
                    break
                self.request.sendall(msg_server.encode(FORMAT))
            elif (msg == "REQUEST_FOR_INST"):
                print(f"[{self.client_address}]" + msg)
                if (self.state == "INITIAL"):
                    task_time = time.time()
                    msg_server = eval_inst

                    target_loc = [
                        ele for ele in self.room_area
                        if (ele in msg_server.split(',')[0])
                    ]
                    target_loc = target_loc[0]

                    target_obj = msg_server.split('pick the')[1].split(
                        'and place')[0].replace(" ", "")
                    target_obj_idx_pos = self.grasp_obj.index(target_obj)
                    target_obj_idx = self.grasp_obj_idx[target_obj_idx_pos]

                    goal_loc = [
                        ele for ele in self.room_area
                        if (ele in msg_server.split(',')[-1])
                    ]
                    goal_loc = goal_loc[0]
                    goal_index = self.room_area.index(goal_loc)

                    current_state += 1
                    self.state = self.task_state[current_state]
                else:
                    state_info = f"[{self.client_address}][TASK_FAILED] Current task state is not match, expected state is START_NAV but got {self.state}"
                    self.request.sendall(state_info.encode(FORMAT))
                    msg_server = DISCONNECT_MSG
                    break
                self.request.sendall(msg_server.encode(FORMAT))
            elif (msg == "REQUEST_TO_CHECK_NAV_RESULT"):
                print(f"[{self.client_address}]" + msg)
                if (self.state == "START_NAV"):
                    obj_idx = self.grasp_obj_idx.index(target_obj_idx)
                    obj_pose = self.obj_pose[obj_idx]
                    nav_result = self.check_nav_result(obj_pose)
                    if (nav_result is True):
                        task_time = time.time() - task_time
                        current_state += 1
                        self.state = self.task_state[current_state]
                        score_e += (1 - task_time / 600) * 40
                        state_info = f"[{self.client_address}][NAV_SUCCESS] Successfully navigate to the target obj. Time of object search is {task_time} . Now score is {score_e}"
                        self.request.sendall(state_info.encode(FORMAT))
                    else:
                        state_info = f"[{self.client_address}][TASK_FAILED] Robot is too far to the object. Got score {score_e}."
                        self.request.sendall(state_info.encode(FORMAT))
                        save_eval_result(str(score_e), str(score_g))
                        self.server.shutdown()
                        os._exit(0)
                    print(state_info)
                else:
                    state_info = f"[{self.client_address}][TASK_FAILED] Current task state is not match, expected state is START_NAV but got {self.state}"
                    self.request.sendall(state_info.encode(FORMAT))
                    self.request.sendall(DISCONNECT_MSG.encode(FORMAT))
                    save_eval_result(str(score_e), str(score_g))
                    self.server.shutdown()
                    os._exit(0)
            elif (msg == "REQUEST_TO_CHECK_GSP_RESULT"):
                if (self.state == "START_GSP"):
                    print(f"[{self.client_address}]" + msg)
                    grasp_pose_global = np.array(grasp_pose_global)
                    if (grasp_pose_global.shape[0] > 10):
                        grasp_pose_global = grasp_pose_global[0:10]
                    _, rgb, _, cam_K, pc_full, _, _ = self.subscribe_pc_ros(
                        "Sim", "/MirKinova/hand_camera/image",
                        "/MirKinova/hand_depth/points")
                    obj_idx = self.grasp_obj_idx.index(target_obj_idx)
                    score_grasp = force_closure(self.gripper_config,
                                                grasp_pose_global,
                                                self.obj_pose[obj_idx],
                                                target_obj_idx, self.data_root, pc_full)
                    state_info = f"[{self.client_address}][GRASP] Get {grasp_pose_global.shape[0]} Grasp pose and get result {score_grasp}"
                    print(state_info)
                    if (score_grasp != 0):
                        current_state += 1
                        self.state = self.task_state[current_state]
                        score_e += score_grasp
                        score_g = 2 * score_grasp
                        state_info = f"[{self.client_address}][GSP_SUCCESS] Successfully grasp {target_obj} and get grasp result {score_grasp}"
                        self.request.sendall(state_info.encode(FORMAT))
                        rospy.set_param('/webots_grasp', target_obj)
                        nav_time = time.time()
                    else:
                        state_info = f"[{self.client_address}][TASK_FAILED] Failed to grasp {target_obj}. Got score {score_e}."
                        self.request.sendall(state_info.encode(FORMAT))
                        save_eval_result(str(score_e), str(score_g))
                        self.server.shutdown()
                        os._exit(0)
                else:
                    state_info = f"[{self.client_address}][TASK_FAILED] Current task state is not match,expected state is START_GSP but got {self.state}"
                    self.request.sendall(state_info.encode(FORMAT))
                    self.request.sendall(DISCONNECT_MSG.encode(FORMAT))
                    save_eval_result(str(score_e), str(score_g))
                    self.server.shutdown()
                    os._exit(0)
            elif (msg == "REQUEST_TO_PLACE"):
                if (self.state == "START_NAV_GOAL"):
                    current_state += 1
                    self.state = self.task_state[current_state]
                    state_info = f"[{self.client_address}][PLACE] ..."

                    robot_trans = self.get_robot_pose()
                    place_valid = self.verify_workspace(
                        robot_trans, goal_index)

                    if (place_valid is False):
                        state_info = f"[{self.client_address}][TASK_FAILED] Robot is too far to place the object. Got score {score_e}."
                        self.request.sendall(state_info.encode(FORMAT))
                        save_eval_result(str(score_e), str(score_g))
                        self.server.shutdown()
                        os._exit(0)
                    else:
                        nav_time = time.time() - nav_time
                        score_e += (1 - nav_time / 600) * 10
                        scene = scene_set.pop()
                        scene_set.add(scene)
                        goal_pose = self.generate_rand_position(
                            self.place_area[scene][goal_index])
                        print(f"[PLACE OBJECT] Place object in {goal_pose}")
                        state_info = f"[{self.client_address}][OBJ_NAME] {goal_pose}"
                        rospy.set_param('/webots_place', f"{goal_pose}")

                    state_info = f"[{self.client_address}][TASK_FINISHED] Got score {score_e}."
                    self.request.sendall(state_info.encode(FORMAT))
                    save_eval_result(str(score_e), str(score_g))
                    self.server.shutdown()
                    os._exit(0)
                else:
                    state_info = f"[{self.client_address}][TASK_FAILED] Current task state is not match, expected state is START_NAV_GOAL but got {self.state}"
                    self.request.sendall(state_info.encode(FORMAT))
                    self.request.sendall(DISCONNECT_MSG.encode(FORMAT))
                    save_eval_result(str(score_e), str(score_g))
                    self.server.shutdown()
                    os._exit(0)
            elif (msg == DISCONNECT_MSG):
                if (self in client_set):
                    client_set.remove(self)
                save_eval_result(str(score_e), str(score_g))
                self.server.shutdown()
                os._exit(0)


def save_eval_result(score, score_grasp):
    dict = {'score_e': score, 'score_g': score_grasp}
    pickle.dump(dict, open('../eval_res.p', 'ab'))


def main():
    server = socketserver.ThreadingTCPServer(ADDR, TaskSocketServer)
    TaskSocketServer.set_status(TaskSocketServer)
    server.serve_forever()


if __name__ == '__main__':
    print("[STARTING] Task server (Easy) is starting ...")
    p = multiprocessing.Process(target=main)
    p.start()
    p.join(700)
    if p.is_alive():
        print("Time out")
        save_eval_result(str(score_e), str(score_g))
        p.kill()
    exit()
