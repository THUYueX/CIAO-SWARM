#!/bin/bash

echo "=== ORB-SLAM3 图像输入深度诊断 ==="

source /opt/ros/melodic/setup.bash

echo "1. 检查 ORB-SLAM3 节点状态..."
rosnode list | grep -E "(RGBD|orb_slam3)"
if rosnode list | grep -q "/RGBD"; then
    echo "✅ /RGBD 节点存在"
else
    echo "❌ /RGBD 节点不存在"
    exit 1
fi

echo ""
echo "2. 检查 ORB-SLAM3 订阅状态..."
echo "RGB图像订阅:"
rostopic info /camera/rgb/image_raw 2>/dev/null | grep -A 5 "Subscribers:" || echo "  ❌ 话题不存在"
echo "深度图订阅:"
rostopic info /camera/depth_registered/image_raw 2>/dev/null | grep -A 5 "Subscribers:" || echo "  ❌ 话题不存在"

echo ""
echo "3. 检查数据流状态（快速检测）..."
echo "RGB图像数据流:"
timeout 2 rostopic hz /camera/rgb/image_raw --window=3 2>/dev/null && echo "  ✅ RGB数据流正常" || echo "  ❌ RGB数据流异常"
echo "深度图数据流:"
timeout 2 rostopic hz /camera/depth_registered/image_raw --window=3 2>/dev/null && echo "  ✅ 深度图数据流正常" || echo "  ❌ 深度图数据流异常"

echo ""
echo "4. 检查图像数据详情..."
echo "RGB图像详情:"
rostopic echo /camera/rgb/image_raw -n 1 --noarr 2>/dev/null | head -10 || echo "  ❌ 无法获取RGB数据"
echo "深度图详情:"
rostopic echo /camera/depth_registered/image_raw -n 1 --noarr 2>/dev/null | head -10 || echo "  ❌ 无法获取深度图数据"

echo ""
echo "5. 检查时间戳..."
echo "RGB图像时间戳:"
rostopic echo /camera/rgb/image_raw -n 1 --noarr 2>/dev/null | grep "stamp:" -A 1 | head -3 || echo "  ❌ 无法获取时间戳"
echo "深度图时间戳:"
rostopic echo /camera/depth_registered/image_raw -n 1 --noarr 2>/dev/null | grep "stamp:" -A 1 | head -3 || echo "  ❌ 无法获取时间戳"

echo ""
echo "6. 检查转发链路状态..."
echo "原始RGB话题:"
timeout 1 rostopic hz /wmy_iris_camRay0/usb_cam/image_raw --window=2 2>/dev/null && echo "  ✅ 原始RGB正常" || echo "  ❌ 原始RGB异常"
echo "单通道深度图话题:"
timeout 1 rostopic hz /wmy_iris_camRay0/depth_map_mono --window=2 2>/dev/null && echo "  ✅ 单通道深度图正常" || echo "  ❌ 单通道深度图异常"

echo ""
echo "7. 检查话题是否存在..."
echo "所有深度图相关话题:"
rostopic list | grep depth_map
echo "所有图像相关话题:"
rostopic list | grep -E "(usb_cam|camera/rgb|camera/depth)"

echo ""
echo "8. 检查 ORB-SLAM3 配置..."
CONFIG_FILE=$(ps aux | grep "ORB_SLAM3" | grep -v grep | grep -o "Examples/RGB-D/[^ ]*" | head -1)
if [ -n "$CONFIG_FILE" ]; then
    echo "使用的配置文件: $CONFIG_FILE"
    if [ -f "$HOME/swarm/ORB_SLAM3/ORB_SLAM3/$CONFIG_FILE" ]; then
        echo "配置文件内容摘要:"
        grep -E "(width|height|encoding|fx|fy)" "$HOME/swarm/ORB_SLAM3/ORB_SLAM3/$CONFIG_FILE" | head -10
    fi
else
    echo "❌ 无法确定配置文件"
fi

echo ""
echo "=== 诊断完成 ==="
echo "如果还是 'waiting for images'，请检查:"
echo "1. 确保 /wmy_iris_camRay0/depth_map_mono 话题有数据"
echo "2. 确认深度图编码是 mono8"
echo "3. 检查图像时间戳是否同步"
echo "4. 重启 ORB-SLAM3 和话题转发"