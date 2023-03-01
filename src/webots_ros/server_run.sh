#!/bin/bash
rviz="true"
webots_ros_home="$HOME/server_ws/src/webots_ros"
webots_home="~/webots"
map_path="$webots_ros_home/config/map/3.yaml"
env_name="robotmani"
scene_name="easy_3_3.wbt"
conda_path="~/anaconda3/etc/profile.d/conda.sh"
inst="'Go to the Parlor, pick the Mug and place in Bedroom'"

terminator --new-tab -e "source ~/server_ws/devel/setup.bash;
export ROS_MASTER_URI=http://127.0.0.1:10241;
roscore -p 10241;
exit;
exec bash" &
sleep 5

terminator --new-tab -e "cd ~/server_ws/ ;
export WEBOTS_HOME=$webots_home;
source devel/setup.bash;
source $conda_path;
conda activate $env_name;
cd src/webots_ros/scripts;
export ROS_MASTER_URI=http://127.0.0.1:10241;
python task_server_easy.py $inst;
killall -e python roslaunch construct_semma roscore;
rm -rf $webots_ros_home/scripts/*.png;
exit;
exec bash" &
sleep 5

terminator --new-tab -e "cd ~/server_ws/ ;
export WEBOTS_HOME=$webots_home;
source devel/setup.bash;
source $conda_path;
conda activate $env_name;
export ROS_MASTER_URI=http://127.0.0.1:10241;
roslaunch webots_ros random_scene.launch mobile_base:=Moma true_false:=true scene_name:=$scene_name no_gui:=true;
exit;
exec bash" & 
sleep 30

terminator --new-tab -e "cd ~/server_ws/; 
source devel/setup.bash;
source $conda_path;
conda activate $env_name;
export ROS_MASTER_URI=http://127.0.0.1:10241;
roslaunch webots_ros demo_socket.launch rviz:=$rviz true_false:=true map_path:=$map_path --wait;
exit;
exec bash" &
sleep 10


