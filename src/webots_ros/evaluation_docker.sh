xhost +local:*
rviz=$1
webots_ros_home=$2
webots_home=$3
map_path=$4
env_name=$5
scene_name=$6
conda_path=$7
inst="\"$8\""
mode=$9

terminator --new-tab -e "docker run --rm -it --privileged --network host -p 127.0.0.1:10241:10241 --gpus all --env="QT_X11_NO_MITSHM=1" --env="DISPLAY=$DISPLAY" -v /tmp/.X11-unix/:/tmp/.X11-unix:rw task_server /bin/bash" & sleep 1

echo "finish init environment"

terminator --new-tab -e "docker exec $(docker container ls | grep task_server | awk '{print $1}') sh /root/server_ws/src/webots_ros/eval_run_docker.sh $rviz $webots_ros_home $webots_home $map_path $env_name $scene_name $conda_path $inst $mode" & sleep 5

echo "finish start server"

