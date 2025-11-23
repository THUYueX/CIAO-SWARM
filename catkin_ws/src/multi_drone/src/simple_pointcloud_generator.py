#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pcd2
from cv_bridge import CvBridge

class SimplePointCloudGenerator:
    def __init__(self, fx = 277.191356, fy = 277.191356, cx = 320.5, cy = 240.5):
        """
        简化版点云生成器
        fx, fy: 相机焦距
        cx, cy: 相机主点
        """
        self.fx = fx  # 焦距x
        self.fy = fy  # 焦距y  
        self.cx = cx       # 主点x
        self.cy = cy       # 主点y
        # 相机内参矩阵
        self.camera_matrix = np.array([
            [self.fx, 0,  self.cx],
            [0,  self.fy, self.cy],
            [0,  0,  1]
        ])
        
        # 点云字段定义
        self.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        
        self.bridge = CvBridge()
        
    def depth_to_pcd(self, depth_image):
        """
        将深度图转换为点云
        depth_image: 深度图，形状 (H, W)，单位米
        返回: Nx3的numpy数组，每行是(x,y,z)坐标
        """
        height, width = depth_image.shape
        
        # 生成像素网格
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)
        
        # 反投影到3D空间
        z = depth_image
        x = (u - self.camera_matrix[0,2]) * z / self.camera_matrix[0,0]
        y = (v - self.camera_matrix[1,2]) * z / self.camera_matrix[1,1]
        
        # 重组为点云 (N, 3)
        points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        
        # 移除无效点 (深度为0或无穷大)
        valid_mask = (z.reshape(-1) > 0) & (z.reshape(-1) < 10)  # 假设有效深度范围0-10米
        points = points[valid_mask]
        
        return points
    
    def create_pointcloud2_msg(self, points, frame_id=None):
        """
        将点云数组转换为ROS PointCloud2消息
        """
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        
        msg = pcd2.create_cloud(header, self.fields, points)
        return msg

class PointCloudNode:
    def __init__(self):
        rospy.init_node('simple_pointcloud_generator')
        
        # 创建点云生成器
        self.pcd_generator = SimplePointCloudGenerator()
        
        # 发布器
        self.pcd_pub = rospy.Publisher('/simple_pointcloud', PointCloud2, queue_size=1)
        
        # 订阅深度图 (根据你的实际话题修改)
        self.depth_sub = rospy.Subscriber('/camera/depth', Image, self.depth_callback)
        
        rospy.loginfo("简单点云生成器已启动")
    
    def depth_callback(self, depth_msg):
        try:
            # 转换深度图
            depth_image = self.pcd_generator.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            
            # 生成点云
            points = self.pcd_generator.depth_to_pcd(depth_image)
            
            # 发布点云
            pcd_msg = self.pcd_generator.create_pointcloud2_msg(points)
            self.pcd_pub.publish(pcd_msg)
            
            rospy.loginfo_once("成功生成并发布点云，点数: {}".format(len(points)))
            
        except Exception as e:
            rospy.logerr("点云生成失败: {}".format(e))

if __name__ == "__main__":
    try:
        node = PointCloudNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass