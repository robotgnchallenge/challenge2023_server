import sys
import os
import math
import copy
import pickle

import open3d as o3d
import numpy as np
import rospy
import tf
from tqdm import tqdm
from scipy.spatial.transform import Rotation as Rot
from webots_ros.srv import supervisor_get_from_def, node_get_pose
from geometry_msgs.msg import Pose, TransformStamped
from graspnetAPI.utils.collision_detector import ModelFreeCollisionDetector
from graspnetAPI import GraspGroup
from graspnetAPI.utils.eval_utils import get_grasp_score, transform_points
from graspnetAPI.utils.config import get_config
from transforms3d.quaternions import quat2mat
from graspnetAPI.utils.rotation import matrix_to_dexnet_params
from graspnetAPI.utils.dexnet.grasping.grasp import ParallelJawPtGrasp3D
from graspnetAPI.utils.dexnet.grasping.grasp_quality_config import GraspQualityConfigFactory

from math_utils import pose_4x4_to_pos_quat

collision_param = {
    'voxel_size': 0.01,
    'approach_dist': 0.02,
    'collision_thresh': 0.01
}


def get_tf_transform(pose, parent, child):
    tf_stamp = TransformStamped()
    tf_stamp.header.frame_id = parent
    tf_stamp.child_frame_id = child
    tf_stamp.transform.translation.x = pose.position.x
    tf_stamp.transform.translation.y = pose.position.y
    tf_stamp.transform.translation.z = pose.position.z
    tf_stamp.transform.rotation.w = pose.orientation.w
    tf_stamp.transform.rotation.x = pose.orientation.x
    tf_stamp.transform.rotation.y = pose.orientation.y
    tf_stamp.transform.rotation.z = pose.orientation.z
    return tf_stamp


def vis_grasps(gg, cloud):
    gg.sort_by_score()
    gg = gg[:50]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])


def read_model(data_root, obj_idx):
    model_dir = os.path.join(data_root, 'models')

    dexmodel_list = []
    model_list = []
    normal_list = []
    o3d_model_list = []

    dex_cache_path = os.path.join(data_root, 'dex_models',
                                  '%03d.pkl' % obj_idx)
    pcd = o3d.io.read_point_cloud(
        os.path.join(model_dir, '%03d' % obj_idx, 'nontextured.ply'))
    points = np.array(pcd.points)
    normals = np.array(pcd.normals)
    model_list.append(points)
    normal_list.append(normals)
    o3d_model_list.append(pcd)

    with open(dex_cache_path, 'rb') as f:
        dexmodel = pickle.load(f)

    dexmodel_list.append(dexmodel)

    config = get_config()

    return model_list, dexmodel_list, normal_list, config, o3d_model_list


def set_geometry_pose(pose):
    '''
    Set pose of ros geometry msg
    input: pose (7,) -> [x,y,z,rx,ry,rz,rw]
    '''
    obj_pose = Pose()
    obj_pose.position.x = pose[0]
    obj_pose.position.y = pose[1]
    obj_pose.position.z = pose[2]
    obj_pose.orientation.w = pose[6]
    obj_pose.orientation.x = pose[3]
    obj_pose.orientation.y = pose[4]
    obj_pose.orientation.z = pose[5]

    return obj_pose


def get_obj_center(pc):
    return np.sum(pc, axis=0) / pc.shape[0]


def set_o3d_pc(pc, offset):
    p = copy.deepcopy(pc.reshape(-1, 3))
    p[0][2] += offset
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(p)
    result.paint_uniform_color([0, 0, 1])

    return result, p


def suction_quality(dexgrasp_list, model_list, normal_list):
    points = model_list[0]
    np_normals = normal_list[0]
    result = []

    for i in tqdm(range(len(dexgrasp_list[1]))):
        grasp = dexgrasp_list[1][i]

        gripper_width = grasp.max_grasp_width_ / 2
        gripper_depth = grasp.max_grasp_depth
        gripper_axis = grasp.axis
        gripper_center = grasp.center
        gripper_angle = grasp.approach_angle_
        gripper_angle = gripper_angle % (math.pi * 2)

        if (gripper_angle > 0 and gripper_angle <= math.pi):
            sin = math.sin(math.radians(90) - gripper_angle)
            cos = math.cos(math.radians(90) - gripper_angle)
        else:
            sin = math.sin(gripper_angle - math.radians(90))
            cos = math.cos(gripper_angle - math.radians(90))

        right_point = gripper_center - (gripper_width / (math.sqrt(
            pow(gripper_axis[0], 2) + pow(gripper_axis[1], 2) +
            pow(gripper_axis[2], 2))) * gripper_axis)
        left_point = gripper_center + (gripper_width / (math.sqrt(
            pow(gripper_axis[0], 2) + pow(gripper_axis[1], 2) +
            pow(gripper_axis[2], 2))) * gripper_axis)
        right_point = np.array([right_point])
        left_point = np.array([left_point])

        left_top = copy.deepcopy(left_point)
        left_top[0][2] += gripper_depth

        right_top = copy.deepcopy(right_point)
        right_top[0][2] += gripper_depth

        vec_1 = np.array([
            left_top[0][0] - left_point[0][0],
            left_top[0][1] - left_point[0][1],
            left_top[0][2] - left_point[0][2]
        ])
        vec_2 = np.array([
            right_top[0][0] - right_point[0][0],
            right_top[0][1] - right_point[0][1],
            right_top[0][2] - right_point[0][2]
        ])

        R_y = [[cos, 0, sin], [0, 1, 0], [-1 * sin, 0, cos]]
        vec_1 = np.dot(R_y, vec_1)
        vec_2 = np.dot(R_y, vec_2)

        left_top = left_point + (float(gripper_depth) / (math.sqrt(
            pow(vec_1[0], 2) + pow(vec_1[1], 2) + pow(vec_1[2], 2))) * vec_1)
        left_top = np.array([left_top])

        right_top = right_point + (float(gripper_depth) / (math.sqrt(
            pow(vec_2[0], 2) + pow(vec_2[1], 2) + pow(vec_2[2], 2))) * vec_2)
        right_top = np.array([right_top])

        pc_offset = 0.01

        b_1, b_pc_1 = set_o3d_pc(left_top, pc_offset)
        b_2, b_pc_2 = set_o3d_pc(left_point, pc_offset)
        b_3, b_pc_3 = set_o3d_pc(right_point, pc_offset)
        b_4, b_pc_4 = set_o3d_pc(right_top, pc_offset)

        b_5, b_pc_5 = set_o3d_pc(left_top, -pc_offset)
        b_6, b_pc_6 = set_o3d_pc(left_point, -pc_offset)
        b_7, b_pc_7 = set_o3d_pc(right_point, -pc_offset)
        b_8, b_pc_8 = set_o3d_pc(right_top, -pc_offset)

        p_1 = np.squeeze(b_pc_5)
        p_4 = np.squeeze(b_pc_6)
        p_2 = np.squeeze(b_pc_8)
        p_5 = np.squeeze(b_pc_1)

        u = np.cross((p_1 - p_4), (p_1 - p_5))
        v = np.cross((p_1 - p_2), (p_1 - p_5))
        w = np.cross((p_1 - p_2), (p_1 - p_4))

        u_range = [min(u.dot(p_2), u.dot(p_1)), max(u.dot(p_2), u.dot(p_1))]
        v_range = [min(v.dot(p_1), v.dot(p_4)), max(v.dot(p_1), v.dot(p_4))]
        w_range = [min(w.dot(p_5), w.dot(p_1)), max(w.dot(p_5), w.dot(p_1))]

        pt_idx = []
        for j in range(points.shape[0]):
            vec_u = u.dot(points[j])
            vec_v = v.dot(points[j])
            vec_w = w.dot(points[j])

            if (u_range[0] <= vec_u <= u_range[1]
                    and v_range[0] <= vec_v <= v_range[1]
                    and w_range[0] <= vec_w <= w_range[1]):
                pt_idx.append(j)

        grasp_end = gripper_center + (float(gripper_depth) / (math.sqrt(
            pow(vec_1[0], 2) + pow(vec_1[1], 2) + pow(vec_1[2], 2))) * vec_1)
        grasp_vec = grasp_end - gripper_center

        vec_angles = []

        target_np_normals = np_normals[pt_idx]

        for j in range(target_np_normals.shape[0]):
            vec_1 = target_np_normals[j]
            vec_2 = grasp_vec

            vec_prod = vec_1.dot(vec_2)
            if ((np.linalg.norm(vec_1) * np.linalg.norm(vec_2)) != 0):
                angle = vec_prod / (np.linalg.norm(vec_1) *
                                    np.linalg.norm(vec_2))
                angle = np.arccos(np.clip(angle, -1, 1))
                angle = math.degrees(angle)
                if (angle > 90):
                    angle = 180 - angle
                vec_angles.append(angle)

        vec_angles = np.array(vec_angles)

        valid_pt = 0
        for j in range(vec_angles.shape[0]):
            if (vec_angles[j] < 30):
                valid_pt += 1

        result.append(valid_pt)

    return result


def grasp_quality(config, dexmodel_list, grasp_list, obj_pose_mat):
    grasps = grasp_list
    grasp_points = grasps[:, 13:16]
    grasp_poses = grasps[:, 4:13].reshape([-1, 3, 3])
    grasp_depths = grasps[:, 3]
    grasp_widths = grasps[:, 1]

    force_closure_quality_config = dict()
    fc_list = np.array([1.8, 1.6, 1.2, 1.0, 0.8, 0.6, 0.4, 0.2])
    for value_fc in fc_list:
        value_fc = round(value_fc, 2)
        config['metrics']['force_closure']['friction_coef'] = value_fc
        force_closure_quality_config[
            value_fc] = GraspQualityConfigFactory.create_config(
                config['metrics']['force_closure'])

    dexgrasp_list = list()
    dexgrasp_list.append(list())
    dexgrasps = list()
    for grasp_id, _ in enumerate(grasps):
        grasp_point = grasp_points[grasp_id]
        rot = grasp_poses[grasp_id]
        width = grasp_widths[grasp_id]
        depth = grasp_depths[grasp_id]
        center = np.array([depth, 0, 0]).reshape([3, 1])
        center = np.dot(grasp_poses[grasp_id], center).reshape([3])
        center = (center + grasp_point).reshape([1, 3])
        center = transform_points(center,
                                  np.linalg.inv(obj_pose_mat)).reshape([3])
        rot = np.dot(obj_pose_mat[:3, :3].T, rot)
        binormal, approach_angle = matrix_to_dexnet_params(rot)
        grasp = ParallelJawPtGrasp3D(
            ParallelJawPtGrasp3D.configuration_from_params(
                center, binormal, width, approach_angle), depth)
        dexgrasps.append(grasp)
    dexgrasp_list.append(dexgrasps)

    i = 0
    grasp_result = []
    for grasp in dexgrasp_list[1]:
        score = get_grasp_score(grasp, dexmodel_list[0], fc_list,
                                force_closure_quality_config)
        if (score != -1):
            grasp_result.append((i, score))
        else:
            grasp_result.append((i, 2))
        i += 1
    return grasp_result, dexgrasp_list


def get_camera_pose(robot_name):
    rospy.wait_for_service(robot_name + '/supervisor/get_from_def')

    def_service = rospy.ServiceProxy(robot_name + '/supervisor/get_from_def',
                                     supervisor_get_from_def)
    origin_def = def_service("Origin", 0)
    camera_def = def_service("CAMERA", 0)

    rospy.wait_for_service(robot_name + '/supervisor/node/get_pose')
    pose_service = rospy.ServiceProxy(robot_name + '/supervisor/node/get_pose',
                                      node_get_pose)

    cam_pose = pose_service(origin_def.node, camera_def.node)
    cam_trans = [
        cam_pose.pose.translation.x, cam_pose.pose.translation.y,
        cam_pose.pose.translation.z
    ]
    cam_rot = [
        cam_pose.pose.rotation.x, cam_pose.pose.rotation.y,
        cam_pose.pose.rotation.z, cam_pose.pose.rotation.w
    ]

    return cam_trans, cam_rot


def preprocess_grasp_pose(gripper_config, grasp_poses, target_obj_pose,
                          obj_idx, data_root, pc_full):
    grasp_info = {}
    tf_trans = tf.Transformer(True, rospy.Duration(10.0))

    offset = []
    rot_mat = []
    grasp_list = []

    model_list, dexmodel_list, normal_list, config, o3d_model_list = read_model(
        data_root, obj_idx)

    obj_pose = set_geometry_pose(target_obj_pose)

    pred_grasps_cam = grasp_poses

    gsp_trans = np.zeros((pred_grasps_cam.shape[0], 3))
    gsp_quat = np.zeros((pred_grasps_cam.shape[0], 4))
    gsp_trans_tran = np.zeros((pred_grasps_cam.shape[0], 3))
    gsp_quat_tran = np.zeros((pred_grasps_cam.shape[0], 4))

    for i in range(pred_grasps_cam.shape[0]):
        gsp_trans[i], gsp_quat[i] = pose_4x4_to_pos_quat(pred_grasps_cam[i])

        r = Rot.from_matrix(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        r = Rot.from_euler('zyx', [90, 0, 90], degrees=True)
        T = r.as_matrix()

        arr = np.matmul(pred_grasps_cam[i][:3, :3], T)
        tran_rotmat = np.eye(4)
        tran_rotmat[:3, :3] = arr
        tran_rotmat[:3, 3] = gsp_trans[i]
        rot_mat.append(tran_rotmat)

        gsp_trans_tran[i], gsp_quat_tran[i] = pose_4x4_to_pos_quat(tran_rotmat)

        ori_pt = np.array(
            [gsp_trans_tran[i][0], gsp_trans_tran[i][1], gsp_trans_tran[i][2]])
        axis_pt = np.array([
            gsp_trans_tran[i][0] + tran_rotmat[0, 0],
            gsp_trans_tran[i][1] + tran_rotmat[1, 0],
            gsp_trans_tran[i][2] + tran_rotmat[2, 0]
        ])
        x_vector = axis_pt - ori_pt
        sum_square = math.sqrt(
            pow(x_vector[0], 2) + pow(x_vector[1], 2) + pow(x_vector[2], 2))
        center_offset = (gripper_config[i][2] / sum_square) * x_vector
        offset.append(center_offset)

        gsp_pose = set_geometry_pose([
            gsp_trans[i][0], gsp_trans[i][1], gsp_trans[i][2],
            gsp_quat_tran[i][0], gsp_quat_tran[i][1], gsp_quat_tran[i][2],
            gsp_quat_tran[i][3]
        ])

        cam_trans, cam_rot = get_camera_pose('/MirKinova')

        cam_pose = set_geometry_pose([
            cam_trans[0], cam_trans[1], cam_trans[2], cam_rot[0], cam_rot[1],
            cam_rot[2], cam_rot[3]
        ])

        cam_tf_stamp = get_tf_transform(cam_pose, '/world',
                                        '/MirKinova/hand_camera')
        tf_trans.setTransform(cam_tf_stamp)

        gsp_tf_stamp = get_tf_transform(gsp_pose, '/MirKinova/hand_camera',
                                        '/gsp_' + str(i))
        tf_trans.setTransform(gsp_tf_stamp)

    obj_tf_stamp = get_tf_transform(obj_pose, '/world', '/obj_pose')
    tf_trans.setTransform(obj_tf_stamp)
    (obj_trans, obj_rot) = tf_trans.lookupTransform('/MirKinova/hand_camera',
                                                    '/obj_pose', rospy.Time(0))

    trans3d_quat = [obj_rot[3]] + obj_rot[0:3]
    obj_mat = quat2mat(trans3d_quat)

    grasp_list_obj_frame = []
    offset_obj = []

    for i in range(pred_grasps_cam.shape[0]):
        (gsp_trans,
         gsp_rot) = tf_trans.lookupTransform('/MirKinova/hand_camera',
                                             '/gsp_' + str(i), rospy.Time(0))
        grasp_list.append(
            np.array([
                1, gripper_config[i][0], gripper_config[i][1], gripper_config[i][2],
                rot_mat[i][0][0], rot_mat[i][0][1], rot_mat[i][0][2],
                rot_mat[i][1][0], rot_mat[i][1][1], rot_mat[i][1][2],
                rot_mat[i][2][0], rot_mat[i][2][1], rot_mat[i][2][2],
                gsp_trans[0] + offset[i][0], gsp_trans[1] + offset[i][1],
                gsp_trans[2] + offset[i][2], 3
            ]))

        (gsp_trans_obj,
         gsp_rot_obj) = tf_trans.lookupTransform('/obj_pose', '/gsp_' + str(i),
                                                 rospy.Time(0))
        gsp_rot_obj = [gsp_rot_obj[3]] + gsp_rot_obj[0:3]
        gsp_rot_obj = quat2mat(gsp_rot_obj)

        ori_pt = np.array(
            [gsp_trans_obj[0], gsp_trans_obj[1], gsp_trans_obj[2]])
        axis_pt = np.array([
            gsp_trans_obj[0] + gsp_rot_obj[0, 0],
            gsp_trans_obj[1] + gsp_rot_obj[1, 0],
            gsp_trans_obj[2] + gsp_rot_obj[2, 0]
        ])
        x_vector = axis_pt - ori_pt
        sum_square = math.sqrt(
            pow(x_vector[0], 2) + pow(x_vector[1], 2) + pow(x_vector[2], 2))
        center_offset = (0.07 / sum_square) * x_vector
        offset_obj.append(center_offset)

        grasp_list_obj_frame.append(
            np.array([
                1, gripper_config[i][0], gripper_config[i][1], gripper_config[i][2],
                gsp_rot_obj[0][0], gsp_rot_obj[0][1], gsp_rot_obj[0][2],
                gsp_rot_obj[1][0], gsp_rot_obj[1][1], gsp_rot_obj[1][2],
                gsp_rot_obj[2][0], gsp_rot_obj[2][1], gsp_rot_obj[2][2],
                gsp_trans_obj[0] + center_offset[0],
                gsp_trans_obj[1] + center_offset[1],
                gsp_trans_obj[2] + center_offset[2], 3
            ]))

    grasp_list = np.array(grasp_list)
    grasp_list_obj_frame = np.array(grasp_list_obj_frame)

    obj_pose_mat = np.eye(4)
    obj_pose_mat[:3, :3] = obj_mat
    obj_pose_mat[:3, 3] = obj_trans

    grasp_info['config'] = config
    grasp_info['dexmodel_list'] = dexmodel_list
    grasp_info['model_list'] = model_list
    grasp_info['normal_list'] = normal_list
    grasp_info['o3d_model_list'] = o3d_model_list
    grasp_info['grasp_list'] = grasp_list
    grasp_info['obj_pose_mat'] = obj_pose_mat
    grasp_info['grasp_list_obj_frame'] = grasp_list_obj_frame

    """
    #Grasp Visualize
    gg = GraspGroup(grasp_list_obj_frame)
    gg_pc = GraspGroup(grasp_list)
    cloud = o3d_model_list[0]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_full)
    o3d.io.write_point_cloud("./pc.ply",pcd)

    pcd_load = o3d.io.read_point_cloud("./pc.ply")
    vis_grasps(gg, cloud)
    vis_grasps(gg_pc, pcd_load)
    """

    return grasp_info


def collision_detection(gg_obj_frame, cloud):
    cloud = cloud[0]
    gg = GraspGroup(gg_obj_frame)
    mfcdetector = ModelFreeCollisionDetector(
        cloud.points, voxel_size=collision_param['voxel_size'])
    collision_mask = mfcdetector.detect(
        gg,
        approach_dist=collision_param['approach_dist'],
        collision_thresh=collision_param['collision_thresh'])
    gg = gg[~collision_mask]
    collision_mask = np.array(collision_mask)
    collision_idx = np.where(collision_mask == True)[0]
    collision = np.ones((collision_mask.shape[0], ))
    collision[collision_idx] = 0
    return collision


def force_closure(gripper_config, grasp_poses, target_obj_pose, obj_idx,
                  data_root, pc_full):
    pc_info = preprocess_grasp_pose(gripper_config, grasp_poses,
                                    target_obj_pose, obj_idx, data_root, pc_full)

    score_fc, dexgrasp_list = grasp_quality(pc_info['config'],
                                            pc_info['dexmodel_list'],
                                            pc_info['grasp_list'],
                                            pc_info['obj_pose_mat'])

    score_normal = suction_quality(dexgrasp_list, pc_info['model_list'],
                                   pc_info['normal_list'])

    collision_map = collision_detection(pc_info['grasp_list_obj_frame'],
                                        pc_info['o3d_model_list'])

    score = 0
    max_pt = 500
    alpha = 14
    beta = 20

    for i in range(len(score_fc)):
        score += collision_map[i] * (alpha * (min(
            (score_normal[i] / max_pt), 1)) + beta * ((2 - score_fc[i][1])))
    score = score / len(score_fc)

    return score
