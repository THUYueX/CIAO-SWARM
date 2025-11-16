#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from geometry_msgs.msg import Twist
import rospy
import math
import random

class DRLAgent:
    def __init__(self, ns):
        self.ns = ns
        self.base_speed = 0.4
        
        # 探索导航参数
        self.last_direction_change = rospy.Time.now()
        self.direction_change_interval = 15.0
        self.is_rotating = False
        self.target_heading = 0
        self.rotation_start_time = None
        
    def make_decision(self, depth_map, height_control=0, mode='exploration'):
        """决策主入口"""
        if depth_map is None:
            return self._create_twist(self.base_speed, 0, height_control)

        if mode == 'exploration':
            # 检查是否需要转向
            if self._should_change_direction():
                self._change_direction()
            
            if self.is_rotating:
                # 旋转模式
                if self._rotate_to_target():
                    self.is_rotating = False
                    rospy.loginfo(f"[{self.ns}] 旋转完成")
                    # 旋转完成后立即使用原来的避障逻辑
                    body_vel = self._original_avoidance_logic(depth_map, height_control)
                    return self._body_to_world_velocity(body_vel)
                else:
                    # 旋转中：小速度前进 + 旋转
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.1
                    cmd_vel.linear.z = height_control
                    if self.target_heading > 0:
                        cmd_vel.angular.z = 0.8
                    else:
                        cmd_vel.angular.z = -0.8
                    return cmd_vel
            else:
                body_vel = self._original_avoidance_logic(depth_map, height_control)
                return self._body_to_world_velocity(body_vel)
        else:
            body_vel = self._original_avoidance_logic(depth_map, height_control)
            return self._body_to_world_velocity(body_vel)

    def _body_to_world_velocity(self, body_vel):
        """将机体坐标系速度转换到世界坐标系"""
        if not hasattr(self, 'current_yaw'):
            return body_vel
        
        yaw = self.current_yaw
        
        # 坐标系转换
        world_vel = Twist()
        world_vel.linear.x = body_vel.linear.x * math.cos(yaw) - body_vel.linear.y * math.sin(yaw)
        world_vel.linear.y = body_vel.linear.x * math.sin(yaw) + body_vel.linear.y * math.cos(yaw)
        world_vel.linear.z = body_vel.linear.z
        world_vel.angular.z = body_vel.angular.z
        
        rospy.loginfo(f"[{self.ns}] 机体速度: 前{body_vel.linear.x:.2f} 右{body_vel.linear.y:.2f} | "
                    f"世界速度: X{world_vel.linear.x:.2f} Y{world_vel.linear.y:.2f}")
        
        return world_vel

    def _original_avoidance_logic(self, depth_map, height_control):
        if depth_map is None:
            return self._create_twist(self.base_speed, 0, height_control)

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
            collision_threshold = overall_mean * 1.5  # 平均深度的?%（较近）
            warning_threshold = overall_mean * 1.3    # 平均深度的?%（中等）
            
            rospy.loginfo(f"[{self.ns}] Raw depths - L:{left_raw:.1f}, C:{center_raw:.1f}, R:{right_raw:.1f}")
            rospy.loginfo(f"[{self.ns}] Dynamic thresholds - Collision:{collision_threshold:.1f}, Warning:{warning_threshold:.1f}")
            
            # 决策逻辑：值越大表示越近，越危险
            return self._final_avoidance(left_raw, center_raw, right_raw, collision_threshold, warning_threshold, height_control)
            
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Decision error: {e}")
            return self._create_twist(self.base_speed, 0, height_control)

    def _final_avoidance(self, left_avg, center_avg, right_avg, collision_threshold, warning_threshold, height_control):
        """最终逻辑：深度值越大表示越近，越危险！"""
        forward_speed = self.base_speed
        lateral_speed = 0.0
        
        # 紧急情况：任何区域深度值很大
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
            
            # 选择深度值最小的方向
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
        
        return self._create_twist(forward_speed, lateral_speed, height_control)

    def _should_change_direction(self):
        time_since_last = (rospy.Time.now() - self.last_direction_change).to_sec()
        return time_since_last > self.direction_change_interval

    def _change_direction(self):
        """选择转向角度"""
        direction_options = [45, -45]
        angle_change = random.choice(direction_options)
        self.target_heading = angle_change
        self.is_rotating = True
        self.rotation_start_time = rospy.Time.now()
        self.last_direction_change = rospy.Time.now()
        
        direction_names = {45: "左前方45°", -45: "右前方45°"}
        direction_name = direction_names.get(angle_change, f"{angle_change}°")
        rospy.loginfo(f"[{self.ns}] 转向: {direction_name}")

    def _rotate_to_target(self):
        if not hasattr(self, 'rotation_start_time'):
            self.rotation_start_time = rospy.Time.now()
            return False
            
        rotation_time = abs(self.target_heading) / (0.8 * 180 / 3.14159)
        elapsed_time = (rospy.Time.now() - self.rotation_start_time).to_sec()
        
        if elapsed_time >= rotation_time:
            del self.rotation_start_time
            return True
        else:
            return False

    def _create_twist(self, linear_x, linear_y, linear_z):
        """创建速度指令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.linear.z = linear_z
        return cmd_vel

    def stop(self):
        """停止智能体"""
        pass

    def reset(self):
        """重置智能体状态"""
        self.last_direction_change = rospy.Time.now()
        self.is_rotating = False
        self.target_heading = 0