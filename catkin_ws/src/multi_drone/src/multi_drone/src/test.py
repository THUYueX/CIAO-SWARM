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

    # def _final_avoidance(self, left_avg, center_avg, right_avg, collision_threshold, warning_threshold, height_control):
    
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
    def _final_avoidance(self, left_avg, center_avg, right_avg, collision_threshold, warning_threshold, height_control):
        """9区域连续控制逻辑"""
        
        forward_speed = self.base_speed
        lateral_speed = 0.0
        
        # 紧急情况：保持原有逻辑
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
        
        # 警告情况：保持原有逻辑
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
        
        # 安全情况：使用9区域连续控制
        else:
            forward_speed, lateral_speed = self._nine_sector_continuous_control()
            rospy.loginfo(f"[{self.ns}] Safe: 9区域连续控制 前{forward_speed:.2f} 侧{lateral_speed:.2f}")
        
        return self._create_twist(forward_speed, lateral_speed, height_control)

    def _nine_sector_continuous_control(self):
        """9区域连续控制"""
        if not hasattr(self, 'depth_map'):
            return self.base_speed, 0.0
            
        height, width = self.depth_map.shape
        sector_width = width // 9
        
        # 计算9个区域的最小深度
        sector_mins = []
        for i in range(9):
            start_col = i * sector_width
            end_col = (i + 1) * sector_width if i < 8 else width
            sector = self.depth_map[:, start_col:end_col]
            sector_mins.append(np.min(sector))
        
        # 计算左右侧的加权安全分数
        left_score = 0.0    # 左侧4个区域（索引0-3）
        right_score = 0.0   # 右侧4个区域（索引5-8）
        
        # 左侧：越靠左的区域权重越高
        for i in range(4):  # 索引0,1,2,3
            weight = (4 - i) * 0.25  # 权重：1.0, 0.75, 0.5, 0.25
            left_score += sector_mins[i] * weight
        
        # 右侧：越靠右的区域权重越高  
        for i in range(5, 9):  # 索引5,6,7,8
            weight = (i - 4) * 0.25  # 权重：0.25, 0.5, 0.75, 1.0
            right_score += sector_mins[i] * weight
        
        # 计算横向速度（连续值）
        total_score = left_score + right_score
        if total_score > 0:
            # 横向速度范围：-0.3 到 0.3
            lateral_speed = (right_score - left_score) / total_score * 0.3
        else:
            lateral_speed = 0.0
        
        # 如果偏向一侧，稍微减速
        if abs(lateral_speed) > 0.1:
            forward_speed = self.base_speed * 0.9
        else:
            forward_speed = self.base_speed
        
        return forward_speed, lateral_speed
    def _should_change_direction(self):
        time_since_last = (rospy.Time.now() - self.last_direction_change).to_sec()
        return time_since_last > self.direction_change_interval

    def _change_direction(self):
        """选择转向角度 - 包含反向选择"""
        # 方向选项：45°左前, -45°右前, 135°左后, -135°右后
        direction_options = [45, -45, 135, -135]
        angle_change = random.choice(direction_options)
        self.target_heading = angle_change
        self.is_rotating = True
        self.rotation_start_time = rospy.Time.now()
        self.last_direction_change = rospy.Time.now()
        
        direction_names = {
            45: "左前方45°", 
            -45: "右前方45°",
            135: "左后方135°", 
            -135: "右后方135°"
        }
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