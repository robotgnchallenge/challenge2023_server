import os
import time
import pickle
import subprocess

import psutil
from tqdm import tqdm

home_dir = os.getenv("HOME")
eval_mode = 'easy'
rviz = "true"
webots_ros_home = "~/server_ws/src/webots_ros"
webots_home = "~/webots"
map_path_prefix = os.path.join(home_dir, "server_ws/src/webots_ros/config/map")
env_name = "robotmani"
conda_path = "~/anaconda3/etc/profile.d/conda.sh"

if __name__ == '__main__':
    inst_file = '../worlds/scene_info/validation_inst.p'
    result_path = '../eval_res.p'

    with open(inst_file, 'rb') as f:
        inst_dict = pickle.load(f)

    for key, value in tqdm(inst_dict.items()):
        scene_name = "_".join([eval_mode, key])
        inst = '\"\'' + inst_dict[key] + '\'\"'
        scene = scene_name.split('_')[1]
        map_path = os.path.join(map_path_prefix, scene + '.yaml')
        print("Task instruction:", inst)

        process_server = subprocess.run(" ".join([
            'bash ../eval_run.sh', rviz, webots_home, webots_home, map_path,
            env_name, scene_name, conda_path, inst, eval_mode
        ]),
                                        shell=True)

        process_client = subprocess.run("sudo bash ~/client_run.sh",
                                        shell=True)

        time.sleep(60)
        while (True):
            process_exist = "roscore" in (p.name()
                                          for p in psutil.process_iter())
            if (process_exist):
                time.sleep(10)
            else:
                res = ''
                with open(result_path, 'rb') as f:
                    while True:
                        try:
                            res = pickle.load(f)
                        except EOFError:
                            break
                print("Finish processing ", key, ", Got result :")
                print(res)
                print("=========================================")
                break

        time.sleep(60)

    result_list = []
    with open(result_path, 'rb') as f:
        while True:
            try:
                res = pickle.load(f)
                result_list.append(res)
            except EOFError:
                break

    score_keys = result_list[0].keys()
    result_dict = {}
    for key in score_keys:
        result_dict[key] = 0
    for i in range(len(result_list)):
        for key in score_keys:
            result_dict[key] += float(result_list[i][key])
    for key in score_keys:
        print(key, ':', result_dict[key] / len(result_list))
    print("=========================================")