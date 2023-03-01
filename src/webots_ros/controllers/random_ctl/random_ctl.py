import sys
import math
import random
import struct

sys.path.append('../..')

import rospy
import socket
import numpy as np
from controller import Supervisor

from config.socket_config import socket_conf

supervisor = Supervisor()

finger_left_node = supervisor.getFromDef("fingertip_left")
finger_right_node = supervisor.getFromDef("fingertip_right")

ADDR, HEADER, FORMAT, DISCONNECT_MSG = socket_conf()

obj_name = sys.argv[2]
obj_size = float(sys.argv[1])

obj_size = obj_size / 2

eef_link = "link_6"


def get_gripper_vec():
    """Get object Attach Offset

    Returns:
        list[3]: object offset in x, y, z axiz.
    """
    eef_link_node = supervisor.getFromDef(eef_link)
    link_trans = eef_link_node.getPosition()
    cam_node = supervisor.getFromDef("CAMERA")
    eef_trans = cam_node.getPosition()
    link_trans = np.array(link_trans)
    eef_trans = np.array(eef_trans)
    approach_vec = eef_trans - link_trans

    offset_param = 0.1

    sum_square = math.sqrt(
        pow(approach_vec[0], 2) + pow(approach_vec[1], 2) +
        pow(approach_vec[2], 2))
    center_offset = (offset_param / sum_square) * approach_vec
    return center_offset


class TaskClient():
    """Task Client for communicate with task server
    """

    def __init__(self) -> None:
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(ADDR)
        self.client.setblocking(False)

    def send(self, msg_content):
        msg = msg_content.encode(FORMAT)
        msg_length = len(msg)
        header = struct.pack("i", msg_length)
        self.client.sendall(header)
        self.client.sendall(msg)

    def recv_msg(self):
        while (supervisor.step(1) != -1):
            try:
                task_server_msg = self.client.recv(2048)
                if (task_server_msg):
                    return task_server_msg.decode(FORMAT)
                else:
                    pass
            except Exception:
                pass

    def check_valid_msg(self, msg):
        if (msg == DISCONNECT_MSG):
            self.send(DISCONNECT_MSG)
        else:
            return msg

    def close(self):
        self.client.close()


task_state_client = TaskClient()

rospy.init_node('webots_controller', anonymous=True)

if (obj_name.isupper()):
    obj_name = obj_name[0] + obj_name[1:].lower()

obj_node = supervisor.getFromDef(obj_name)
robot_node = supervisor.getFromDef("eef")
trans_field = obj_node.getField("translation")
obj_pose = obj_node.getPose()

world_node = supervisor.getFromDef("WORLD")
scene_field = world_node.getField("title")
scene = scene_field.getSFString()

print("[WEBOTS_INFO][" + obj_name + "] waiting for grasp")

task_state_client.send(" ".join(
    ["[WEBOTS_INFO] [" + obj_name + "] [" + scene + "]",
     str(obj_pose)]))

print("=============" + obj_name + ": wait_for_call===============")

while (supervisor.step(1) != -1):
    try:
        target_obj = rospy.get_param('/webots_grasp')
        if (target_obj != -1):
            break
    except Exception:
        pass

if (target_obj == obj_name):
    print(obj_name + ": attach to grippper")
    while (supervisor.step(1) != -1):
        offset = get_gripper_vec()
        left_trans = finger_left_node.getPose()
        left_trans = [left_trans[3], left_trans[7], left_trans[11]]
        right_trans = finger_right_node.getPose()
        right_trans = [right_trans[3], right_trans[7], right_trans[11]]
        target_pose = [(left_trans[0] + right_trans[0]) / 2 + offset[0],
                       (left_trans[1] + right_trans[1]) / 2 + offset[1],
                       left_trans[2] + offset[2]]
        trans_field.setSFVec3f(target_pose)
        obj_node.resetPhysics()
        try:
            plc_pose = rospy.get_param('/webots_place')
        except Exception:
            pass
        if (plc_pose != -1):
            pose = plc_pose.split(',')
            pose[0] = pose[0][1:]
            pose[2] = pose[2][:-1]
            x = float(pose[0])
            y = float(pose[1])
            z = float(pose[2]) + obj_size
            trans_field.setSFVec3f([x, y, z])
            obj_node.resetPhysics()
            print(obj_name + ": place to ", x, ' ', y, ' ', z)
            task_state_client.close()
            break
else:
    task_state_client.close()
