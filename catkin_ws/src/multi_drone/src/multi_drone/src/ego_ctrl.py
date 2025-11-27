#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
import rospy
import math

class EgoController:
    def __init__(self, ns):
        self.ns = ns
        
        # 状态变量
        self.current_pose = None
        self.current_yaw = 0.0
        self.has_path = False
        self.path_points = []
        self.current_waypoint_index = 0
        
        # 控制参数
        self.kp_xy = 0.8      # 位置P增益
        self.kp_z = 0.6       # 高度P增益
        self.kp_yaw = 0.5     # 偏航P增益
        self.reached_threshold = 0.5  # 到达航点的距离阈值
        self.lookahead_distance = 1.0  # 前视距离
        
        # 订阅ego_planner路径
        self.path_sub = rospy.Subscriber(
            '/ego_planner_node/optimal_list',
            Marker,
            self.path_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"[{ns}] Ego Controller initialized - waiting for planner paths")

    def path_callback(self, data):
        """接收ego_planner的路径点"""
        if len(data.points) == 0:
            rospy.logwarn(f"[{self.ns}] Received empty path")
            return
            
        self.path_points = data.points
        self.current_waypoint_index = 0
        self.has_path = True
        
        rospy.loginfo(f"[{self.ns}] Received path with {len(data.points)} points")

    def make_decision(self, depth_map=None, height_control=0, mode='ego_planner'):
        """
        纯路径跟踪决策
        返回世界坐标系速度指令
        """
        # 如果没有路径或位姿，悬停
        if not self.has_path or self.current_pose is None:
            rospy.loginfo_throttle(5, f"[{self.ns}] No path or pose, hovering")
            return self._create_twist(0, 0, height_control)
        
        try:
            # 检查路径完成
            if self.current_waypoint_index >= len(self.path_points):
                rospy.loginfo(f"[{self.ns}] Path completed, hovering")
                return self._create_twist(0, 0, height_control)
                
            # 获取当前目标点（使用前视点）
            lookahead_index = self._get_lookahead_waypoint()
            target_point = self.path_points[lookahead_index]
            current_pos = self.current_pose.pose.position
            
            # 计算世界坐标系位置误差
            error_x = target_point.x - current_pos.x
            error_y = target_point.y - current_pos.y
            error_z = target_point.z - current_pos.z
            
            # 检查是否到达最终目标点
            if lookahead_index == len(self.path_points) - 1:
                distance_to_goal = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                if distance_to_goal < self.reached_threshold:
                    rospy.loginfo(f"[{self.ns}] Reached final goal!")
                    self.has_path = False
                    return self._create_twist(0, 0, height_control)
            
            # 世界坐标系速度 (P控制)
            world_vel_x = error_x * self.kp_xy
            world_vel_y = error_y * self.kp_xy
            world_vel_z = error_z * self.kp_z + height_control
            
            # 偏航角控制：朝向目标点
            target_yaw = math.atan2(error_y, error_x)
            yaw_error = target_yaw - self.current_yaw
            # 角度规范化
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            vel_yaw = yaw_error * self.kp_yaw
            
            # 速度限制
            world_vel_x = np.clip(world_vel_x, -1.0, 1.0)
            world_vel_y = np.clip(world_vel_y, -1.0, 1.0)
            world_vel_z = np.clip(world_vel_z, -0.5, 0.5)
            vel_yaw = np.clip(vel_yaw, -1.0, 1.0)
            
            cmd_vel = self._create_twist(world_vel_x, world_vel_y, world_vel_z, vel_yaw)
            
            rospy.loginfo_throttle(2,
                f"[{self.ns}] Tracking WP:{lookahead_index}/{len(self.path_points)-1} "
                f"Vel:({world_vel_x:.2f},{world_vel_y:.2f},{world_vel_z:.2f}) "
                f"Yaw:{vel_yaw:.2f}")
            
            return cmd_vel
            
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Path tracking error: {e}")
            return self._create_twist(0, 0, height_control)

    def _get_lookahead_waypoint(self):
        """获取前视航点，提供更平滑的跟踪"""
        if self.current_pose is None or not self.path_points:
            return 0
            
        current_pos = self.current_pose.pose.position
        
        # 寻找距离当前点最近且在前方的航点
        for i in range(self.current_waypoint_index, len(self.path_points)):
            point = self.path_points[i]
            distance = math.sqrt(
                (point.x - current_pos.x)**2 + 
                (point.y - current_pos.y)**2
            )
            
            # 如果找到足够远的点，或者是最后一个点
            if distance >= self.lookahead_distance or i == len(self.path_points) - 1:
                self.current_waypoint_index = min(i, len(self.path_points) - 1)
                return i
        
        return len(self.path_points) - 1

    def _create_twist(self, linear_x, linear_y, linear_z, angular_z=0):
        """创建速度指令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.linear.z = linear_z
        cmd_vel.angular.z = angular_z
        return cmd_vel

    def stop(self):
        """停止控制器"""
        self.has_path = False

    def reset(self):
        """重置控制器状态"""
        self.has_path = False
        self.current_waypoint_index = 0