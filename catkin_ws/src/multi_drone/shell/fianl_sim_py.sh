#!/bin/bash

echo "=== 启动无人机仿真 ==="

cd ~/swarm/catkin_ws

echo "1. 编译代码..."
catkin_make
if [ $? -ne 0 ]; then
    echo "编译失败！"
    exit 1
fi

echo "2. 设置环境..."
source devel/setup.bash

echo "3. 设置PX4环境..."
export GAZEBO_PLUGIN_PATH=~/swarm/gazabo/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=~/swarm/gazabo/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/swarm/gazabo/PX4-Autopilot:~/swarm/gazabo/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

echo "4. 启动仿真..."
gnome-terminal --tab --title="Simulation" -- bash -c "
cd ~/swarm/catkin_ws
source devel/setup.bash
export GAZEBO_PLUGIN_PATH=~/swarm/gazabo/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=~/swarm/gazabo/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/swarm/gazabo/PX4-Autopilot:~/swarm/gazabo/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch multi_drone multi_drone.launch
exec bash"

echo "等待10秒让仿真完全启动..."
sleep 10

echo "5. 启动Python控制节点（使用conda环境Python解释器）..."
gnome-terminal --tab --title="Python Control" -- bash -c "
cd ~/swarm/catkin_ws/src/multi_drone/src
source ~/swarm/catkin_ws/devel/setup.bash
source ~/anaconda3/etc/profile.d/conda.sh  # 加载conda
conda activate drone  # 激活您的drone环境
echo '使用conda环境的Python解释器直接运行...'
python multi_drone.py
echo 'Python 控制节点已退出，按任意键关闭窗口...'
read -n 1
exec bash"

echo "=== 仿真启动完成 ==="