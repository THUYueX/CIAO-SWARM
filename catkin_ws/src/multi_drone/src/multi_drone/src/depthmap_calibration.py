#!/usr/bin/env python3
# depthmap_calibration.py

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
import message_filters
from cv_bridge import CvBridge
import ros_numpy
from std_msgs.msg import Float32

class DepthmapCalibration:
    def __init__(self, fx=277.191356, fy=277.191356, cx=160.0, cy=120.0, 
                 orb_topic='/orb_slam3_ros/map_points', 
                 depth_topic='/wmy_iris_camRay0/depth_map_mono'):
        """
        深度图标定节点
        
        Args:
            fx, fy: 相机焦距
            cx, cy: 相机主点
            orb_topic: ORB-SLAM3点云话题
            depth_topic: 深度图话题
        """
        self.bridge = CvBridge()
        self.scale_factor = 1.0
        
        # 相机参数
        self.fx = fx
        self.fy = fy  
        self.cx = cx
        self.cy = cy
        
        # 发布标定结果
        self.scale_pub = rospy.Publisher('/depth_calibration/scale_factor', Float32, queue_size=10)
        
        # 订阅ORB-SLAM3点云和深度图
        orb_sub = message_filters.Subscriber(orb_topic, PointCloud2)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        
        # 时间同步
        ts = message_filters.TimeSynchronizer([orb_sub, depth_sub], 10)
        ts.registerCallback(self.calibration_callback)
        
        rospy.loginfo(f"深度图标定节点启动")
        rospy.loginfo(f"相机参数: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
        rospy.loginfo(f"订阅话题: ORB={orb_topic}, Depth={depth_topic}")
        
    def calibration_callback(self, orb_pcd_msg, depth_msg):
        """
        标定回调函数
        """
        try:
            # 转换深度图
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            
            # 转换ORB点云
            orb_points = ros_numpy.point_cloud2.pointcloud2_to_array(orb_pcd_msg)
            
            # 计算尺度因子
            new_scale = self.calibrate_scale(depth_image, orb_points)
            
            if new_scale > 0:
                self.scale_factor = new_scale
                rospy.loginfo(f"标定尺度更新: {self.scale_factor:.4f}")
                
                # 发布标定结果
                scale_msg = Float32()
                scale_msg.data = self.scale_factor
                self.scale_pub.publish(scale_msg)
                
        except Exception as e:
            rospy.logerr(f"标定错误: {e}")
    
    def calibrate_scale(self, ai_depth, orb_pcd):
        """
        对标AI深度图和ORB点云的尺度
        
        Args:
            ai_depth: AI估计的深度图
            orb_pcd: ORB-SLAM3的点云
            
        Returns:
            scale: 尺度因子
        """
        # 将ORB点云投影生成稀疏深度图
        orb_depth = self.project_orb_to_depth(orb_pcd, ai_depth.shape)
        
        if np.sum(orb_depth > 0) < 10:  # ORB点云太少
            rospy.logwarn("ORB点云数量不足，跳过标定")
            return self.scale_factor
        
        # 消除前30%远的点
        filter_threshold = np.percentile(ai_depth[ai_depth > 0], 70) if np.any(ai_depth > 0) else 0
        ai_depth_filtered = ai_depth.copy()
        ai_depth_filtered[ai_depth_filtered > filter_threshold] = 0
        
        # 去除无穷大值
        ai_depth_filtered[np.isinf(ai_depth_filtered)] = 0
        
        # 找到都有值的区域
        valid_mask = (ai_depth_filtered != 0) & (orb_depth != 0)
        ai_valid = ai_depth_filtered[valid_mask]
        orb_valid = orb_depth[valid_mask]
        
        if len(ai_valid) < 10:
            rospy.logwarn("有效对应点太少，跳过标定")
            return self.scale_factor
            
        # 最小二乘计算尺度
        try:
            scale = np.linalg.lstsq(ai_valid.reshape(-1,1), orb_valid, rcond=None)[0][0]
            rospy.loginfo(f"标定成功: {len(ai_valid)}个对应点, 尺度={scale:.4f}")
            return float(scale)
        except Exception as e:
            rospy.logerr(f"最小二乘计算失败: {e}")
            return self.scale_factor
    
    def project_orb_to_depth(self, orb_pcd, depth_shape):
        """
        将ORB点云投影到图像平面生成深度图
        
        Args:
            orb_pcd: ORB点云数据
            depth_shape: 深度图形状 (height, width)
            
        Returns:
            orb_depth: 投影后的稀疏深度图
        """
        height, width = depth_shape
        orb_depth = np.zeros((height, width))
        
        point_count = 0
        for point in orb_pcd:
            # 提取3D坐标
            x = point['x']
            y = point['y'] 
            z = point['z']
            
            # 过滤无效点
            if z > 0.1 and z < 20.0 and np.isfinite(x) and np.isfinite(y):
                # 投影到像素坐标
                u = int(x / z * self.fx + self.cx)
                v = int(y / z * self.fy + self.cy)
                
                # 检查边界
                if 0 <= u < width and 0 <= v < height:
                    # 如果该位置已经有值，取最近的
                    if orb_depth[v, u] == 0 or z < orb_depth[v, u]:
                        orb_depth[v, u] = z
                        point_count += 1
        
        rospy.logdebug(f"ORB点云投影: {point_count}个点投影到图像")
        return orb_depth

    def get_scale_factor(self):
        """获取当前尺度因子"""
        return self.scale_factor