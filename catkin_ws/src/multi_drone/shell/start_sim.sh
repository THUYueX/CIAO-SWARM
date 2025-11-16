#!/bin/bash

cd ~/swarm/catkin_ws

# 设置ROS环境
source devel/setup.bash
# 1. 导出 Gazebo 插件 & 模型路径
source /home/wumengyu/swarm/gazabo/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
        /home/wumengyu/swarm/gazabo/PX4-Autopilot \
        /home/wumengyu/swarm/gazabo/PX4-Autopilot/build/px4_sitl_default
# 设置PX4环境
# 手动设置环境变量，绕过缺失的setup_gazebo.bash
export GAZEBO_PLUGIN_PATH=~/swarm/gazabo/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=~/swarm/gazabo/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/swarm/gazabo/PX4-Autopilot:~/swarm/gazabo/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# 启动仿真
roslaunch multi_drone multi_drone.launch
