import numpy as np
import cv2
import rospy

class DepthFrameStacker:
    """深度图帧堆叠模块 - 带时序衰减记忆"""
    
    def __init__(self, stack_size=4, target_size=(112, 112), max_depth=20.0, decay_rate=0.6):
        """
        初始化
        Args:
            stack_size: 堆叠帧数 (论文中用4帧)
            target_size: 目标图像尺寸 (论文中用112x112)
            max_depth: 最大深度限制 (论文中20米)
            decay_rate: 衰减率，0.6表示每帧保留60%的强度
        """
        self.stack_size = stack_size
        self.target_size = target_size
        self.max_depth = max_depth
        self.decay_rate = decay_rate
        
        self.frame_buffer = []
        self.timestamps = []  # 存储时间戳用于时序衰减

    def add_frame(self, depth_map):
        """
        添加新帧到堆叠缓冲区 - 带时序衰减
        Args:
            depth_map: 原始深度图
        Returns:
            fused_frame: 时序融合后的帧 或 None(如果缓冲区未满)
        """
        # 预处理当前帧
        processed_frame = self.preprocess_single_frame(depth_map)
        
        # 添加到缓冲区
        self.frame_buffer.append(processed_frame)
        self.timestamps.append(rospy.Time.now().to_sec())
        
        # 保持缓冲区长度不超过stack_size
        if len(self.frame_buffer) > self.stack_size:
            self.frame_buffer.pop(0)
            self.timestamps.pop(0)
        
        # 如果缓冲区已满，返回时序融合结果
        if len(self.frame_buffer) == self.stack_size:
            return self.get_temporal_fused_frames()
        else:
            return None

    def get_temporal_fused_frames(self):
        """
        获取时序衰减融合的帧序列
        Returns:
            fused_frame: 融合后的单帧 [height, width]
        """
        if len(self.frame_buffer) < self.stack_size:
            # 如果缓冲区不满，用最后一帧填充
            while len(self.frame_buffer) < self.stack_size:
                self.frame_buffer.append(self.frame_buffer[-1] if self.frame_buffer else np.zeros(self.target_size))
                self.timestamps.append(rospy.Time.now().to_sec())
        
        # 计算时序衰减权重（越新的帧权重越高）
        weights = []
        current_time = rospy.Time.now().to_sec()
        
        for i in range(len(self.frame_buffer)):
            # 基于帧顺序的指数衰减，最新的帧权重最高
            weight = self.decay_rate ** (len(self.frame_buffer) - 1 - i)
            weights.append(weight)
        
        # 归一化权重
        total_weight = sum(weights)
        normalized_weights = [w / total_weight for w in weights]
        
        # 加权融合所有帧
        fused_frame = np.zeros_like(self.frame_buffer[0], dtype=np.float32)
        for i, frame in enumerate(self.frame_buffer):
            fused_frame += frame.astype(np.float32) * normalized_weights[i]
        
        return fused_frame.astype(np.uint8)

    def get_stacked_frames(self):
        """
        获取堆叠的帧序列 - 保持原有接口
        Returns:
            stacked_array: numpy数组 [stack_size, height, width]
        """
        if len(self.frame_buffer) < self.stack_size:
            while len(self.frame_buffer) < self.stack_size:
                self.frame_buffer.append(self.frame_buffer[-1] if self.frame_buffer else np.zeros(self.target_size))
        
        # 堆叠为 [4, 112, 112]
        stacked_array = np.stack(self.frame_buffer, axis=0)
        return stacked_array

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

    def reset(self):
        """重置缓冲区"""
        self.frame_buffer = []
        self.timestamps = []

    def is_ready(self):
        """检查是否已准备好堆叠数据"""
        return len(self.frame_buffer) >= self.stack_size

    def get_buffer_status(self):
        """获取缓冲区状态"""
        return f"Buffer: {len(self.frame_buffer)}/{self.stack_size} frames, decay_rate: {self.decay_rate}"
