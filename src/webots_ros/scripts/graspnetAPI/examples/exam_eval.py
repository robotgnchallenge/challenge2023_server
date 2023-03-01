__author__ = 'mhgou'
__version__ = '1.0'

# GraspNetAPI example for evaluate grasps for a scene.
# change the graspnet_root path
import numpy as np
from graspnetAPI import GraspNetEval

####################################################################
graspnet_root = '/home/cheang/dataset/graspnet' # ROOT PATH FOR GRASPNET
dump_folder = '/home/cheang/code/graspnet-baseline/logs/dump_rs' # ROOT PATH FOR DUMP
####################################################################

sceneId = 100
camera = 'realsense'    
#ge_k = GraspNetEval(root = graspnet_root, camera = 'kinect', split = 'test_seen')
ge_r = GraspNetEval(root = graspnet_root, camera = 'realsense', split = 'test_seen')


# eval a single scene
#print('Evaluating scene:{}, camera:{}'.format(sceneId, camera))
#acc = ge_k.eval_scene(scene_id = sceneId, dump_folder = dump_folder)
acc = ge_r.eval_scene(scene_id = sceneId, dump_folder = dump_folder)
np_acc = np.array(acc)
print('mean accuracy:{}'.format(np.mean(np_acc)))


# # eval all data for kinect
# print('Evaluating kinect')
# res, ap = ge_k.eval_all(dump_folder, proc = 24)

# # eval 'seen' split for realsense
# print('Evaluating realsense')
# res, ap = ge_r.eval_seen(dump_folder, proc = 24)
