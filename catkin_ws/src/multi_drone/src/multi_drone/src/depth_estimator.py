#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import torch
import numpy as np
import cv2
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from simple_pointcloud_generator import SimplePointCloudGenerator
from sensor_msgs.msg import PointCloud2, PointField
import rospy
import time
import threading

class DepthEstimator:
    def __init__(self, ns):
        self.ns = ns
        self.bridge = CvBridge()
        self.processing_counter = 0
        # 创建点云生成器实例
        self.pcd_generator = SimplePointCloudGenerator()
        
        # 点云发布器
        self.pcd_publisher = rospy.Publisher(f'/{ns}/pointcloud', PointCloud2, queue_size=10)
        
        # 每个无人机独立加载自己的模型实例
        rospy.loginfo(f"[{ns}] Loading dedicated MiDaS model...")
        start_time = time.time()
        
        self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
        self.model.eval()
        
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = midas_transforms.small_transform
        
        load_time = time.time() - start_time
        rospy.loginfo(f"[{ns}] Dedicated MiDaS model loaded in {load_time:.2f}s")

    def process_ros_image(self, ros_image_msg):
        """处理ROS图像消息"""
        try:
            # 使用RGB图像的精确时间戳
            rgb_timestamp = ros_image_msg.header.stamp
            
            # 1. 转换ROS图像到numpy数组
            rgb_image = self._convert_ros_to_numpy(ros_image_msg)
            if rgb_image is None:
                return None, None, None
                
            # 2. 深度估计
            depth_map = self.estimate_depth(rgb_image)
            if depth_map is None:
                return None, None, None

            depth_map_resized = cv2.resize(depth_map, (320, 240), interpolation=cv2.INTER_NEAREST)
                
            # 3. 创建深度图时强制使用相同时间戳
            color_msg, mono_msg = self.create_depth_ros_msgs(depth_map_resized, rgb_timestamp)
            # 调用simple点云生成器
            points = self.pcd_generator.depth_to_pcd(depth_map_resized)
            pointcloud_msg = self.pcd_generator.create_pointcloud2_msg(points, frame_id = f"{self.ns}/robot_camera_link")
            self.pcd_publisher.publish(pointcloud_msg)
            
            # 记录时间戳信息（用于调试）
            if self.processing_counter % 10 == 0:
                rospy.loginfo(f"[{self.ns}] Timestamp: {rgb_timestamp.secs}.{rgb_timestamp.nsecs}")
            
            return depth_map, color_msg, mono_msg
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Processing error: {str(e)}")
            return None, None, None
        
    def create_depth_ros_msgs(self, depth_map, timestamp=None):
        """创建深度图ROS消息 - 使用指定的时间戳"""
        try:
            if depth_map is None:
                return None, None
                
            # 使用传入的时间戳，或当前时间
            if timestamp is None:
                timestamp = rospy.Time.now()
                
            
            # 1. 彩色深度图
            depth_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_uint8 = depth_normalized.astype(np.uint8)
            depth_colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
            
            color_msg = ROSImage()
            color_msg.header.stamp = timestamp  # 使用同步的时间戳
            color_msg.header.frame_id = f"{self.ns}/camera_frame"
            color_msg.height = 240
            color_msg.width = 320
            color_msg.encoding = "bgr8"
            color_msg.is_bigendian = 0
            color_msg.step = 320 * 3
            color_msg.data = depth_colored.tobytes()
            
            # 2. 单通道深度图
            depth_mm = (depth_map * 5000).astype(np.uint16)
            
            mono_msg = ROSImage()
            mono_msg.header.stamp = timestamp  # 使用相同的时间戳
            mono_msg.header.frame_id = f"{self.ns}/camera_frame"  
            mono_msg.height = 240
            mono_msg.width = 320
            mono_msg.encoding = "16UC1"
            mono_msg.is_bigendian = 0
            mono_msg.step = 320 * 2
            mono_msg.data = depth_mm.tobytes()
            
            return color_msg, mono_msg
                
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Depth message creation error: {str(e)}")
            return None, None

    def _convert_ros_to_numpy(self, ros_image_msg):
        """手动转换ROS图像到numpy数组"""
        try:
            # 检查图像编码
            if ros_image_msg.encoding not in ['bgr8', 'rgb8', 'mono8']:
                rospy.logerr(f"[{self.ns}] Unsupported image encoding: {ros_image_msg.encoding}")
                return None
            
            # 将ROS图像数据转换为numpy数组
            image_data = np.frombuffer(ros_image_msg.data, dtype=np.uint8)
            
            # 根据编码确定通道数
            if ros_image_msg.encoding == 'mono8':
                # 单通道图像
                image_data = image_data.reshape(ros_image_msg.height, ros_image_msg.width)
                rgb_image = cv2.cvtColor(image_data, cv2.COLOR_GRAY2BGR)
            else:
                # 三通道图像
                image_data = image_data.reshape(ros_image_msg.height, ros_image_msg.width, 3)
                if ros_image_msg.encoding == 'rgb8':
                    rgb_image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)
                else:  # bgr8
                    rgb_image = image_data
            
            return rgb_image
            
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Manual image conversion error: {str(e)}")
            return None

    def estimate_depth(self, image):
        """深度估计核心函数"""
        try:
            # 预处理
            image_np = np.array(image)
            input_batch = self.transform(image_np)
            print(f"输入图像尺寸: {image_np.shape}")  # 添加这行
            print(f"MiDaS输入尺寸: {input_batch.shape}")  # 添加这

            # 模型推理 ← 这里调用MiDaS
            inference_start = time.time()
            with torch.no_grad():
                depth_map = self.model(input_batch)  # ← MiDaS模型调用
            inference_time = time.time() - inference_start

            # 将深度图转换为米为单位
            depth_np = depth_map.squeeze().numpy()
            
            # 方案1: 归一化并缩放到合理范围 (200-700 -> 0-100)
            depth_normalized = (depth_np - depth_np.min()) / (depth_np.max() - depth_np.min())
            depth_meters = depth_normalized * 100.0  # 缩放到0-100范围
            
            # 或者方案2: 直接除以一个缩放因子 (根据你的场景调整)
            # depth_meters = depth_np / 100.0
            
            print(f"原始深度范围: {depth_np.min():.3f} - {depth_np.max():.3f}")
            print(f"转换后深度范围: {depth_meters.min():.3f} - {depth_meters.max():.3f}米")

            return depth_meters

        except Exception as e:
            rospy.logerr(f"[{self.ns}] Depth estimation failed: {str(e)}")
            return None