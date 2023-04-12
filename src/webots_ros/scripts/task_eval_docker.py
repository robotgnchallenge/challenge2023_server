import os
import time
import glob
import psutil
import pickle
import subprocess
from tqdm import tqdm

eval_mode = 'easy'
rviz = "true"
webots_ros_home = "/root/server_ws/src/webots_ros"
webots_home = "/root/webots"
map_path_prefix = "/root/server_ws/src/webots_ros/config/map"
env_name = "robotmani"
conda_path = "/root/miniconda3/etc/profile.d/conda.sh"

if __name__ == '__main__':
    inst_file = '../worlds/scene_info/validation_inst.p'
    result_path = '../eval_res.p'

    if(os.path.exists('../eval_res_docker')):
        pass
    else:
        os.mkdir('../eval_res_docker')

    with open(inst_file, 'rb') as f:
        inst_dict = pickle.load(f)

    for key, value in tqdm(inst_dict.items()):
        scene_name = "_".join([eval_mode, key])
        inst = '\"\'' + inst_dict[key] + '\'\"'
        scene = scene_name.split('_')[1]
        map_path = os.path.join(map_path_prefix, scene + '.yaml')
        print("Task instruction:", inst)
        
        process_server = subprocess.run(" ".join([
            'sudo sh ../evaluation_docker.sh', rviz, webots_home, webots_home, map_path,
            env_name, scene_name, conda_path, inst, eval_mode
        ]),
                                        shell=True)
        
        time.sleep(60)
        process_client = subprocess.run("sudo bash ~/client_run.sh",
                                        shell=True)

        time.sleep(60)
        while (True):
            process_exist = "roscore" in (p.name()
                                          for p in psutil.process_iter())
            if (process_exist):
                time.sleep(10)
            else:
                print("Finish Processing")
                id = os.popen("sudo docker ps -lq").read()
                id = id.split('\n')[0]
                try: 
                    os.system("sudo docker cp " +str(id) +":/root/server_ws/src/webots_ros/eval_res.p ../eval_res_docker")
                    os.system(f"mv ../eval_res_docker/eval_res.p ../eval_res_docker/eval_res_{scene_name.split('.')[0]}.p")
                    os.system("sudo killall -e terminator")

                    res = ''
                    with open(f"../eval_res_docker/eval_res_{scene_name.split('.')[0]}.p", 'rb') as f:
                        while True:
                            try:
                                res = pickle.load(f)
                            except EOFError:
                                break
                except Exception:
                    res = {'score_e':0,'score':0, 'score_g': 0, 'score_o' : 0}
                    pickle.dump(dict, open(f"../eval_res_docker/eval_res_{scene_name.split('.')[0]}.p", 'wb'))

                print("Finish processing ", key, ", Got result :")
                print(res)
                print("=========================================")
                break
        time.sleep(60)
    
    # Read evaluation results
    res_folder = '../eval_res_docker'
    res_files = glob.glob(os.path.join(res_folder,'*'))
    res_dict = {}

    for res_file in res_files:
        with open(res_file, 'rb') as f:
            res = pickle.load(f)
            keys = res.keys()
            for key in keys:
                if(key in res_dict):
                    res_dict[key] += float(res[key])
                else:
                    res_dict[key] = float(res[key])
    
    for key in res_dict.keys():
        res_dict[key] /= len(res_files)
    
    print("Finish processing all of the data, Got result :")
    print(res_dict)
    print("=========================================")

    os.system(f"rm -rf {os.path.join(res_folder,'*')}")
                                         

    