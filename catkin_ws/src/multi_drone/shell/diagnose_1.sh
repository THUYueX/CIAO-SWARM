#!/bin/bash

echo "=== ORB-SLAM3 Mono 终极诊断 ==="

source /opt/ros/melodic/setup.bash

# 1. 检查话题数据
echo "1. 检查话题数据流..."
echo "原始话题状态:"
rostopic info /wmy_iris_camRay0/usb_cam/image_raw

echo ""
echo "实时数据流检查:"
timeout 3 rostopic hz /wmy_iris_camRay0/usb_cam/image_raw --window=3

echo ""
echo "话题内容样本:"
rostopic echo /wmy_iris_camRay0/usb_cam/image_raw -n 1 --noarr | head -5

# 2. 检查ORB-SLAM3节点
echo ""
echo "2. 检查ORB-SLAM3节点状态..."
ORB_NODE=$(rosnode list | grep -i "orb\|mono")
if [ -n "$ORB_NODE" ]; then
    echo "ORB-SLAM3节点: $ORB_NODE"
    echo "节点订阅状态:"
    rosnode info $ORB_NODE | grep -A 15 "Subscriptions"
else
    echo "❌ 未找到ORB-SLAM3节点"
fi

# 3. 检查话题连接
echo ""
echo "3. 检查话题连接..."
echo "ORB-SLAM3期望的话题订阅者:"
rostopic info /camera/image_raw | grep -A 10 "Subscribers"

echo ""
echo "实际话题的订阅者:"
rostopic info /wmy_iris_camRay0/usb_cam/image_raw | grep -A 10 "Subscribers"

# 4. 验证重映射
echo ""
echo "4. 验证重映射..."
echo "当前所有图像话题:"
rostopic list | grep -E "(image|camera)"