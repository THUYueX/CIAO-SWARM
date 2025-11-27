#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import threading
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image as ROSImage
from depth_estimator import DepthEstimator
from drl_agent import DRLAgent
import math

class DroneUnit:
    def __init__(self, ns):
        self.ns = ns
        self.depth_estimator = DepthEstimator(ns)
        self.drl_agent = DRLAgent(ns)  # 所有运动逻辑
        
        # 无人机状态
        self.state = State()
        self.offboard_good = False
        self.arm_good = False
        self.last_req = rospy.Time.now()
        
        # 传感器数据
        self.local_pose = None
        self.local_pose_received = False
        self.current_image = None
        self.image_ready = False
        
        # ROS通信
        self.sub_image = rospy.Subscriber(f'/{ns}/usb_cam/image_raw', ROSImage, self.cb_image)
        self.sub_pose = rospy.Subscriber(f'/{ns}/mavros/local_position/pose', PoseStamped, self.cb_pose)
        self.sub_state = rospy.Subscriber(f'/{ns}/mavros/state', State, self.cb_state)
        self.pub_vel = rospy.Publisher(f'/{ns}/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.pub_sp = rospy.Publisher(f'/{ns}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_depth_color = rospy.Publisher(f'/{ns}/depth_map_color', ROSImage, queue_size=10)
        self.pub_depth_mono = rospy.Publisher(f'/{ns}/depth_map_mono', ROSImage, queue_size=10)
        
        # MAVROS服务
        rospy.wait_for_service(f'/{ns}/mavros/cmd/arming')
        rospy.wait_for_service(f'/{ns}/mavros/set_mode')
        self.cli_arm = rospy.ServiceProxy(f'/{ns}/mavros/cmd/arming', CommandBool)
        self.cli_mode = rospy.ServiceProxy(f'/{ns}/mavros/set_mode', SetMode)
        
        # 位置设置点
        self.sp = PoseStamped()
        self.sp.pose.orientation.w = 1.0
        
        # 启动控制线程
        self.keep_running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()
        
        rospy.loginfo(f"[{self.ns}] 初始化完成")

    def _run(self):
        rate = rospy.Rate(20)
        
        # 阶段1: 初始化设置点
        for i in range(40):
            if rospy.is_shutdown() or not self.keep_running:
                return
            self.publish_sp()
            rate.sleep()
            
        # 阶段2: OFFBOARD和解锁
        while not rospy.is_shutdown() and self.keep_running:
            self.publish_sp()
            if not self.offboard_good and self.state.mode != "OFFBOARD":
                if self.try_set_mode("OFFBOARD"):
                    self.offboard_good = True
                    self.last_req = rospy.Time.now()
            if not self.arm_good and not self.state.armed:
                if self.try_arm(True):
                    self.arm_good = True
                    self.last_req = rospy.Time.now()
            if self.offboard_good and self.arm_good:
                break
            rate.sleep()
            
        # 阶段3: 起飞
        target_height = 2.0
        takeoff_duration = 15.0
        takeoff_start_time = rospy.Time.now()
        while not rospy.is_shutdown() and self.keep_running:
            current_time = (rospy.Time.now() - takeoff_start_time).to_sec()
            if current_time > takeoff_duration:
                break
            progress = current_time / takeoff_duration
            current_height = target_height * progress
            self.sp.pose.position.z = current_height
            self.publish_sp()
            rate.sleep()
            
        # 阶段4: 探索导航 - 所有逻辑在DRLAgent中
        while not rospy.is_shutdown() and self.keep_running:
            if self.image_ready and self.current_image is not None and self.local_pose_received:
                try:
                    depth_map, color_msg, mono_msg = self.depth_estimator.process_ros_image(self.current_image)
                    if depth_map is not None:
                        if color_msg: self.pub_depth_color.publish(color_msg)
                        if mono_msg: self.pub_depth_mono.publish(mono_msg)
                        
                        # 高度控制
                        height_control = self._height_control()
                        
                        # DRLAgent
                        vel_cmd = self.drl_agent.make_decision(
                            depth_map=depth_map,
                            height_control=height_control,
                            mode='exploration'  # 探索模式
                        )
                        
                        self.publish_vel(vel_cmd)
                        self.image_ready = False
                except Exception as e:
                    rospy.logerr(f"[{self.ns}] 导航错误: {e}")
                    self.image_ready = False
            rate.sleep()

    def _height_control(self):
        if not self.local_pose_received:
            return 0.0
        target_height = 2.0
        current_height = self.local_pose.pose.position.z
        height_error = target_height - current_height
        control_output = height_error * 0.5
        control_output = max(min(control_output, 0.5), -0.5)
        return control_output

    # ROS回调函数
    def cb_image(self, msg):
        self.current_image = msg
        self.image_ready = True

    def cb_pose(self, msg):
        self.local_pose = msg
        self.local_pose_received = True
        
        # 计算偏航角并传递给DRLAgent
        orientation = msg.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        self.drl_agent.current_yaw = yaw

    def cb_state(self, msg):
        self.state = msg

    def try_set_mode(self, mode):
        try:
            resp = self.cli_mode(custom_mode=mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            return False

    def try_arm(self, arm):
        try:
            resp = self.cli_arm(value=arm)
            return resp.success
        except rospy.ServiceException as e:
            return False

    def publish_sp(self):
        self.sp.header.stamp = rospy.Time.now()
        self.sp.header.frame_id = "base_link"
        self.pub_sp.publish(self.sp)

    def publish_vel(self, vel_cmd):
        self.pub_vel.publish(vel_cmd)

    def stop(self):
        self.keep_running = False
        self.drl_agent.stop()

class MultiDroneController:
    def __init__(self):
        self.drones = [
            DroneUnit('wmy_iris_camRay0')
            # DroneUnit('wmy_iris_camRay1')
            # DroneUnit('wmy_iris_camRay2')
        ]
        
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            for drone in self.drones:
                drone.stop()

if __name__ == '__main__':
    try:
        rospy.init_node('multi_drone_controller', anonymous=True)
        controller = MultiDroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass