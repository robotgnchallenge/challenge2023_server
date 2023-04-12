#!/bin/bash
rviz=$1
webots_ros_home=$2
webots_home=$3
map_path=$4
env_name=$5
scene_name=$6
conda_path=$7
inst="\"$8\""
mode=$9

terminator --new-tab -e "source server_ws/devel/setup.bash;
export ROS_MASTER_URI=http://127.0.0.1:10241;
roscore -p 10241;
exit;
exec bash" &
sleep 5

if [ "$mode" = "easy" ];
then
    terminator --new-tab -e "cd ~/server_ws/ ;
    export WEBOTS_HOME=$webots_home;
    source devel/setup.bash;
    source $conda_path;
    conda activate $env_name;
    cd src/webots_ros/scripts;
    export ROS_MASTER_URI=http://127.0.0.1:10241;
    python task_server_easy.py $inst;
    rm -rf $webots_ros_home/scripts/*.png;
    killall -e roslaunch construct_semma roscore;
    exit;
    exec bash" &
    sleep 5
else
    terminator --new-tab -e "cd ~/server_ws/ ;
    export WEBOTS_HOME=$webots_home;
    source devel/setup.bash;
    source $conda_path;
    conda activate $env_name;
    cd src/webots_ros/scripts;
    export ROS_MASTER_URI=http://127.0.0.1:10241;
    python task_server_normal.py $inst;
    rm -rf $webots_ros_home/scripts/*.png;
    killall -e roslaunch construct_semma roscore;
    exit;
    exec bash" &
    sleep 5
fi

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


