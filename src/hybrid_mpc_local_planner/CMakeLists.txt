cmake_minimum_required(VERSION 3.0.2)
project(hybrid_mpc_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
        )
find_package(Eigen3 REQUIRED)

catkin_python_setup()

remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    ./include
    ${catkin_INCLUDE_DIRS}
    ${hybrid_mpc_local_planner_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    /usr/local/include
    )
add_definitions(${EIGEN3_DEFINITIONS})

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MPCTrajPlanner.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/DWAPlanner.cfg
#)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hybrid_mpc_local_planner pid
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
        message_generation
        message_runtime
)

add_library(pid src/pid.cc)
add_dependencies(pid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid ${catkin_LIBRARIES})

add_library(hybrid_mpc_local_planner src/hybrid_mpc_planner.cpp src/hybrid_mpc_planner_ros.cpp src/hybrid_mpc_config.cpp src/utils.cc)
add_library(mpc_local_planner_plugin src/hybrid_mpc_planner.cpp src/mpc_local_planner_plugin.cpp src/hybrid_mpc_config.cpp src/utils.cc)

add_dependencies(hybrid_mpc_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_mpc_local_planner ${catkin_LIBRARIES} pid)
add_dependencies(mpc_local_planner_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_local_planner_plugin ${catkin_LIBRARIES} pid)