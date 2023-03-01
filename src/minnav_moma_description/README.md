# minnav_moma_description
description pkg for minnav mobile manipulator, including robot URDF and mesh files.

# usage
```shell
catkin build minnav_moma_description
source path_to_catkin_ws/devel/setup.bash
# launch the robot and display 
# we have several version of moma models, you should specify version when launch
roslaunch minnav_moma_description display.launch model_version:=v2
# roslaunch minnav_moma_description display.launch model_version:=v3
#roslaunch minnav_moma_description display.launch model_version:=v7
```

> to launch different robot model, please specify robot_type and model_version:
```shell
# launch j2s6s200 with v3 base
roslaunch minnav_moma_description display.launch robot_type:=j2s6s200 model_version:=v3
# launch j2s7s200 with v3 base
roslaunch minnav_moma_description display.launch robot_type:=j2s7s200 model_version:=v3

```





# TODO
- [x] integrate robot arm, e.g. kinova j2s6s200/j2s7s300. -- Done
- [x] add moveit_config for minnav_moma robot.
- [x] test on real hardware, demo with mobile navigation and arm movement.
- [x] Milestone1: Mobile Grasp Demo.
- [x] Milestone2: Object Navigation Demo.
- [x] support j2s6s300/j2s6s200/j2s7s300/j2s7s200 + v3 base + wrist camera.
