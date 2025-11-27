#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from geometry_msgs.msg import Twist
from depth_frame_stacker import DepthFrameStacker
import rospy


class DRLAgent:
    def __init__(self, ns):
        self.ns = ns
        self.base_speed = 0.4

        # 深度图堆叠器
        self.depth_stacker = DepthFrameStacker(
            stack_size=4, 
            target_size=(112, 112), 
            max_depth=20.0
        )
        
        # 强化学习组件
        self.rl_network = None  # 待实现
        self.memory = []  # 经验回放缓冲区
        
        rospy.loginfo(f"[{self.ns}] DRLAgent initialized")

    def make_decision(self, depth_map, mode):
        """决策主入口"""
        if depth_map is None:
            return self._create_twist(self.base_speed, 0, 0)
        
        if mode == '0':
            return self._rule_based_avoidance(depth_map)
        elif mode == 'rl_train':
            return self._rl_training_step(depth_map)
        elif mode == 'rl_inference':
            return self._rl_inference_step(depth_map)
        else:
            return self._create_twist(self.base_speed, 0, 0)

    # ==================== 基于规则的避障 ====================
    def _rule_based_avoidance(self, depth_map):
        """基于规则的避障决策"""
        try:
            height, width = depth_map.shape
            
            # 分区域
            left_region = depth_map[:, :width//3]
            center_region = depth_map[:, width//3:2*width//3]
            right_region = depth_map[:, 2*width//3:]
            
            # 计算原始平均值
            left_raw = np.mean(left_region)
            center_raw = np.mean(center_region)
            right_raw = np.mean(right_region)
            
            # 计算整体深度统计
            overall_mean = np.mean(depth_map)
            
            # 动态计算阈值
            collision_threshold = overall_mean * 1.5
            warning_threshold = overall_mean * 1.3
            
            rospy.loginfo(f"[{self.ns}] Raw depths - L:{left_raw:.1f}, C:{center_raw:.1f}, R:{right_raw:.1f}")
            rospy.loginfo(f"[{self.ns}] Dynamic thresholds - Collision:{collision_threshold:.1f}, Warning:{warning_threshold:.1f}")
            
            return self._final_avoidance(left_raw, center_raw, right_raw, collision_threshold, warning_threshold)
            
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Decision error: {e}")
            return self._create_twist(self.base_speed, 0, 0)

    def _final_avoidance(self, left_avg, center_avg, right_avg, collision_threshold, warning_threshold):
        """最终避障逻辑"""
        forward_speed = self.base_speed
        lateral_speed = 0.0
        
        # 紧急情况
        if (center_avg > collision_threshold or 
            left_avg > collision_threshold or 
            right_avg > collision_threshold):
            
            if left_avg < right_avg and left_avg < center_avg:
                rospy.logwarn(f"[{self.ns}] EMERGENCY! Turn LEFT (L:{left_avg:.1f} safest)")
                lateral_speed = 0.3
                forward_speed *= 0.5
            elif right_avg < left_avg and right_avg < center_avg:
                rospy.logwarn(f"[{self.ns}] EMERGENCY! Turn RIGHT (R:{right_avg:.1f} safest)")
                lateral_speed = -0.3
                forward_speed *= 0.5
            else:
                rospy.logwarn(f"[{self.ns}] EMERGENCY! Slowing down")
                forward_speed *= 0.3
        
        # 警告情况
        elif (center_avg > warning_threshold or 
              left_avg > warning_threshold or 
              right_avg > warning_threshold):
            
            if left_avg < right_avg and left_avg < center_avg:
                rospy.loginfo(f"[{self.ns}] Warning: Steering LEFT (L:{left_avg:.1f} safest)")
                lateral_speed = 0.2
                forward_speed *= 0.7
            elif right_avg < left_avg and right_avg < center_avg:
                rospy.loginfo(f"[{self.ns}] Warning: Steering RIGHT (R:{right_avg:.1f} safest)")
                lateral_speed = -0.2
                forward_speed *= 0.7
            else:
                rospy.loginfo(f"[{self.ns}] Warning: Slowing down")
                forward_speed *= 0.5
        
        # 安全情况
        else:
            rospy.loginfo(f"[{self.ns}] Safe: Straight")
        
        return self._create_twist(forward_speed, lateral_speed, 0)

    # ==================== 强化学习相关方法 ====================
    def _rl_training_step(self, depth_map):
        """RL训练步骤"""
        # 堆叠深度图
        stacked_frames = self.depth_stacker.add_frame(depth_map)
        
        if stacked_frames is None:
            # 缓冲区还没满，先用规则避障
            rospy.loginfo(f"[{self.ns}] Frame buffer not ready, using rule-based")
            return self._rule_based_avoidance(depth_map)
        
        # 准备状态
        state = self._prepare_state(stacked_frames)
        
        # RL网络决策
        action = self._rl_network_predict(state)
        
        # 转换为控制指令
        vel_cmd = self._action_to_twist(action)
        
        # 记录经验（用于训练）
        self._record_experience(state, action, depth_map)
        
        return vel_cmd

    def _rl_inference_step(self, depth_map):
        """RL推理步骤"""
        # 堆叠深度图
        stacked_frames = self.depth_stacker.add_frame(depth_map)
        
        if stacked_frames is None:
            return self._rule_based_avoidance(depth_map)
        
        # 准备状态
        state = self._prepare_state(stacked_frames)
        
        # RL网络决策
        action = self._rl_network_predict(state)
        
        # 转换为控制指令
        vel_cmd = self._action_to_twist(action)
        
        return vel_cmd

    def _prepare_state(self, stacked_frames):
        """准备RL状态"""
        # TODO: 实现状态准备
        pass

    def _rl_network_predict(self, state):
        """RL网络预测"""
        # TODO: 实现网络预测
        pass

    def _action_to_twist(self, action):
        """将RL动作转换为Twist消息"""
        # TODO: 实现动作转换
        pass

    def _record_experience(self, state, action, depth_map):
        """记录经验"""
        # TODO: 实现经验记录
        pass

    def train(self):
        """训练RL网络"""
        # TODO: 实现训练逻辑
        pass

    def save_model(self, path):
        """保存模型"""
        # TODO: 实现模型保存
        pass

    def load_model(self, path):
        """加载模型"""
        # TODO: 实现模型加载
        pass

    # ==================== 工具方法 ====================
    def _create_twist(self, linear_x, linear_y, angular_z):
        """创建速度指令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.angular.z = angular_z
        return cmd_vel

    def reset(self):
        """重置智能体状态"""
        self.depth_stacker.reset()
        # TODO: 其他重置逻辑