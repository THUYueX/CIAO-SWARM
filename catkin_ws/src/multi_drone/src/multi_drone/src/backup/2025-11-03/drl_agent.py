#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from geometry_msgs.msg import Twist
import rospy

class DRLAgent:
    def __init__(self, ns):
        self.ns = ns
        self.base_speed = 0.2
        
    def make_decision(self, depth_map):
        if depth_map is None:
            return self._create_twist(self.base_speed, 0, 0)

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
            
            # 动态计算阈值 - 现在：值越大表示越近，越危险！
            collision_threshold = overall_mean * 1.5  # 平均深度的?%（较近）
            warning_threshold = overall_mean * 1.3    # 平均深度的?%（中等）
            
            rospy.loginfo(f"[{self.ns}] Raw depths - L:{left_raw:.1f}, C:{center_raw:.1f}, R:{right_raw:.1f}")
            rospy.loginfo(f"[{self.ns}] Dynamic thresholds - Collision:{collision_threshold:.1f}, Warning:{warning_threshold:.1f}")
            
            # 决策逻辑：值越大表示越近，越危险！
            return self._final_avoidance(left_raw, center_raw, right_raw, collision_threshold, warning_threshold)
            
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Decision error: {e}")
            return self._create_twist(self.base_speed, 0, 0)

    def _final_avoidance(self, left_avg, center_avg, right_avg, collision_threshold, warning_threshold):
        """最终逻辑：深度值越大表示越近，越危险！"""
        forward_speed = self.base_speed
        lateral_speed = 0.0
        
        # 紧急情况：任何区域深度值很大（很近了！）
        if (center_avg > collision_threshold or 
            left_avg > collision_threshold or 
            right_avg > collision_threshold):
            
            # 找到深度值最小的区域（最安全）
            if left_avg < right_avg and left_avg < center_avg:
                rospy.logwarn(f"[{self.ns}] EMERGENCY! Turn LEFT (L:{left_avg:.1f} safest)")
                lateral_speed = 0.3  # 往左转
                forward_speed *= 0.5
            elif right_avg < left_avg and right_avg < center_avg:
                rospy.logwarn(f"[{self.ns}] EMERGENCY! Turn RIGHT (R:{right_avg:.1f} safest)")
                lateral_speed = -0.3  # 往右转
                forward_speed *= 0.5
            else:
                rospy.logwarn(f"[{self.ns}] EMERGENCY! Slowing down")
                forward_speed *= 0.3
        
        # 警告情况：任何区域深度值较大
        elif (center_avg > warning_threshold or 
              left_avg > warning_threshold or 
              right_avg > warning_threshold):
            
            # 选择深度值最小的方向（最安全）
            if left_avg < right_avg and left_avg < center_avg:
                rospy.loginfo(f"[{self.ns}] Warning: Steering LEFT (L:{left_avg:.1f} safest)")
                lateral_speed = 0.2  # 往左转
                forward_speed *= 0.7
            elif right_avg < left_avg and right_avg < center_avg:
                rospy.loginfo(f"[{self.ns}] Warning: Steering RIGHT (R:{right_avg:.1f} safest)")
                lateral_speed = -0.2  # 往右转
                forward_speed *= 0.7
            else:
                rospy.loginfo(f"[{self.ns}] Warning: Slowing down")
                forward_speed *= 0.5
        
        # 安全情况
        else:
            rospy.loginfo(f"[{self.ns}] Safe: Straight")
        
        return self._create_twist(forward_speed, lateral_speed, 0)

    def _create_twist(self, linear_x, linear_y, angular_z):
        """创建速度指令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.angular.z = angular_z
        return cmd_vel