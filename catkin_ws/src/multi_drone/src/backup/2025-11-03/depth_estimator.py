#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import torch
import numpy as np
import cv2
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import rospy
import time
import threading

class DepthEstimator:
    def __init__(self, ns):
        self.ns = ns
        self.bridge = CvBridge()
        self.processing_counter = 0
        
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
        """处理ROS图像消息 - 完整的深度估计流程"""
        self.processing_counter += 1
        total_start = time.time()
        
        try:
            # 1. 转换ROS图像到numpy数组
            convert_start = time.time()
            rgb_image = self._convert_ros_to_numpy(ros_image_msg)
            convert_time = time.time() - convert_start
            if rgb_image is None:
                return None, None
                
            # 2. 深度估计
            depth_start = time.time()
            depth_map = self.estimate_depth(rgb_image)
            depth_time = time.time() - depth_start
            if depth_map is None:
                return None, None
                
            # 3. 创建深度图ROS消息（保持色彩）
            msg_start = time.time()
            depth_msg = self.create_depth_ros_msg(depth_map)
            msg_time = time.time() - msg_start
            
            total_time = time.time() - total_start
            
            # 每10帧输出一次性能信息
            if self.processing_counter % 10 == 0:
                rospy.loginfo(
                    f"[{self.ns}] Performance (Frame {self.processing_counter}):\n"
                    f"  Total: {total_time:.3f}s\n"
                    f"  - Convert: {convert_time:.3f}s\n"
                    f"  - DepthEst: {depth_time:.3f}s\n"
                    f"  - MsgCreate: {msg_time:.3f}s\n"
                    f"  - FPS: {1.0/total_time:.1f}"
                )
            
            return depth_map, depth_msg
            
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Depth processing error: {str(e)}")
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
        """深度估计核心函数 - 每个实例独立运行，无需锁"""
        try:
            # 预处理（这部分在test_midas中也是包含的）
            image_np = np.array(image)
            input_batch = self.transform(image_np)
            
            # 只测纯模型推理时间（对应test_midas的推理时间）
            inference_start = time.time()
            with torch.no_grad():
                depth_map = self.model(input_batch)
            inference_time = time.time() - inference_start
            
            # 每20帧输出一次纯推理时间
            if self.processing_counter % 20 == 0:
                rospy.loginfo(f"[{self.ns}] Pure Inference Time: {inference_time:.3f}s, FPS: {1.0/inference_time:.1f}")
            
            return depth_map.squeeze().numpy()
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Depth estimation failed: {str(e)}")
            return None

    def create_depth_ros_msg(self, depth_map):
        """创建深度图ROS消息 - 保持JET色彩映射"""
        try:
            if depth_map is None:
                return None
                
            # 归一化到0-255
            normalize_start = time.time()
            depth_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_uint8 = depth_normalized.astype(np.uint8)
            normalize_time = time.time() - normalize_start
            
            # 使用JET色彩映射（保持色彩显示）
            colormap_start = time.time()
            depth_colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
            colormap_time = time.time() - colormap_start
            
            # 手动创建ROS消息
            msg_start = time.time()
            depth_msg = ROSImage()
            depth_msg.header.stamp = rospy.Time.now()
            depth_msg.header.frame_id = f"{self.ns}/camera_frame"
            depth_msg.height = depth_colored.shape[0]
            depth_msg.width = depth_colored.shape[1]
            depth_msg.encoding = "bgr8"
            depth_msg.is_bigendian = 0
            depth_msg.step = depth_msg.width * 3
            depth_msg.data = depth_colored.tobytes()
            msg_time = time.time() - msg_start
            
            if self.processing_counter % 30 == 0:
                rospy.loginfo(
                    f"[{self.ns}] Color MsgCreate Details:\n"
                    f"  - Normalize: {normalize_time:.3f}s\n"
                    f"  - Colormap: {colormap_time:.3f}s\n"
                    f"  - ROSMsg: {msg_time:.3f}s"
                )
            
            return depth_msg
            
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Color depth message creation error: {str(e)}")
            return None