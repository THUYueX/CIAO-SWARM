import numpy as np
import cv2

class DepthFrameStacker:
    """深度图帧堆叠模块"""
    
    def __init__(self, stack_size=4, target_size=(112, 112), max_depth=20.0):
        """
        初始化
        Args:
            stack_size: 堆叠帧数 (论文中用4帧)
            target_size: 目标图像尺寸 (论文中用112x112) 
            max_depth: 最大深度限制 (论文中20米)
        """
        self.stack_size = stack_size
        self.target_size = target_size
        self.max_depth = max_depth
        self.frame_buffer = []
        
    def preprocess_single_frame(self, depth_map):
        """
        预处理单帧深度图 - 按照论文方法
        1. 限制最大深度为20m
        2. 归一化到[0,255]
        3. 调整尺寸为112x112
        """
        # 1. 限制最大深度
        depth_clipped = np.clip(depth_map, 0, self.max_depth)
        
        # 2. 归一化到[0,255]
        depth_normalized = (depth_clipped / self.max_depth * 255).astype(np.uint8)
        
        # 3. 调整尺寸
        depth_resized = cv2.resize(depth_normalized, self.target_size)
        
        return depth_resized
    
    def add_frame(self, depth_map):
        """
        添加新帧到堆叠缓冲区
        Args:
            depth_map: 原始深度图
        Returns:
            stacked_frames: 堆叠后的帧序列 [4, 112, 112] 或 None(如果缓冲区未满)
        """
        # 预处理当前帧
        processed_frame = self.preprocess_single_frame(depth_map)
        
        # 添加到缓冲区
        self.frame_buffer.append(processed_frame)
        
        # 保持缓冲区长度不超过stack_size
        if len(self.frame_buffer) > self.stack_size:
            self.frame_buffer.pop(0)
        
        # 如果缓冲区已满，返回堆叠结果
        if len(self.frame_buffer) == self.stack_size:
            return self.get_stacked_frames()
        else:
            return None
    
    def get_stacked_frames(self):
        """
        获取堆叠的帧序列
        Returns:
            stacked_array: numpy数组 [stack_size, height, width]
        """
        if len(self.frame_buffer) < self.stack_size:
            # 如果缓冲区不满，用最后一帧填充
            while len(self.frame_buffer) < self.stack_size:
                self.frame_buffer.append(self.frame_buffer[-1] if self.frame_buffer else np.zeros(self.target_size))
        
        # 堆叠为 [4, 112, 112]
        stacked_array = np.stack(self.frame_buffer, axis=0)
        return stacked_array
    
    def reset(self):
        """重置缓冲区"""
        self.frame_buffer = []
    
    def is_ready(self):
        """检查是否已准备好堆叠数据"""
        return len(self.frame_buffer) >= self.stack_size
    
    def get_buffer_status(self):
        """获取缓冲区状态"""
        return f"Buffer: {len(self.frame_buffer)}/{self.stack_size} frames"
