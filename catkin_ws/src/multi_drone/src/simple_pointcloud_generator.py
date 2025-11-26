#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pcd2
from cv_bridge import CvBridge

class SimplePointCloudGenerator:
    def __init__(self, fx = 277.191356, fy = 277.191356, cx = 160.0, cy = 120.0):
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
        
    def depth_to_pcd(self, depth_image, distance = 10, scale = 200.0, remove_far_percentile=30):
        """
        将深度图转换为点云 - 使用减法反转远近关系
        """
        print(f"输入深度图范围: {depth_image.min():.3f} - {depth_image.max():.3f}")
        height, width = depth_image.shape
        
        # 生成像素网格
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)
        
        # 用减法反转远近关系
        # z = distance - depth_image
        # print(f"反转后深度范围: {z.min():.3f} - {z.max():.3f}米")        
        # 取倒数并缩放
        depth_safe = np.where(depth_image > 0, depth_image, 0.001)
        z_inverted = 1.0 / depth_safe  # 取倒数：原来值大=近 → 现在值大=远
        z = z_inverted * scale         # 缩放到合理范围
    
        print(f"取倒数后范围: {z_inverted.min():.6f} - {z_inverted.max():.6f}")
        print(f"缩放后深度范围: {z.min():.3f} - {z.max():.3f}米")
        
        # 反投影到3D空间
        x = (u - self.camera_matrix[0,2]) * z / self.camera_matrix[0,0]
        y = (v - self.camera_matrix[1,2]) * z / self.camera_matrix[1,1]
        
        # 重组为点云 (N, 3)
        points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        
        # 使用反转后的深度值进行百分比过滤
        z_flat = z.reshape(-1)
        
        # 移除最远的30%点（现在z值小的点是最远的）
        far_threshold = np.percentile(z_flat[z_flat > 0], remove_far_percentile)
        
        valid_mask = (
            (z_flat <= far_threshold) &           # 保留较近的点（z值大的）
            (z_flat > 0.1) &                      # 最小深度
            (z_flat < 10.0) &                     # 最大深度
            (np.isfinite(z_flat)) &
            (np.abs(x.reshape(-1)) < 10.0) &
            (np.abs(y.reshape(-1)) < 8.0)
        )
        
        points = points[valid_mask]
        
        valid_count = len(points)
        total_count = height * width
        
        print(f"过滤后: {valid_count} / {total_count} ({valid_count/total_count*100:.1f}%)")
        print(f"点云距离范围: {points[:,2].min():.3f} - {points[:,2].max():.3f}米")
        
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