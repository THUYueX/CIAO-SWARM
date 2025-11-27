#!/home/wumengyu/anaconda3/envs/drone/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image as ROSImage
import threading

from depth_estimator import DepthEstimator
from drl_agent import DRLAgent

class DroneUnit:
    def __init__(self, ns):
        self.ns = ns
        self.depth_estimator = DepthEstimator(ns)
        self.drl_agent = DRLAgent(ns)
        self.state = State()
        self.offboard_good = False
        self.arm_good = False
        self.last_req = rospy.Time.now()
        
        # 定位数据
        self.local_pose = None
        self.local_pose_received = False
        
        # 图像数据
        self.current_image = None
        self.image_ready = False
        
        # ROS通信
        self.sub_image = rospy.Subscriber(f'/{ns}/usb_cam/image_raw', ROSImage, self.cb_image)
        self.sub_pose = rospy.Subscriber(f'/{ns}/mavros/local_position/pose', PoseStamped, self.cb_pose)
        self.sub_state = rospy.Subscriber(f'/{ns}/mavros/state', State, self.cb_state)
        self.pub_vel = rospy.Publisher(f'/{ns}/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.pub_sp = rospy.Publisher(f'/{ns}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_depth = rospy.Publisher(f'/{ns}/depth_map', ROSImage, queue_size=10)
        
        # 服务
        rospy.wait_for_service(f'/{ns}/mavros/cmd/arming')
        rospy.wait_for_service(f'/{ns}/mavros/set_mode')
        self.cli_arm = rospy.ServiceProxy(f'/{ns}/mavros/cmd/arming', CommandBool)
        self.cli_mode = rospy.ServiceProxy(f'/{ns}/mavros/set_mode', SetMode)
        
        # 设置点
        self.sp = PoseStamped()
        self.sp.pose.orientation.w = 0.0
        
        # 启动独立运行线程
        self.keep_running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()
        
        rospy.loginfo(f"[{self.ns}] DroneUnit initialized and started")

    def _run(self):
        """每个无人机独立运行完整的流程"""
        rate = rospy.Rate(20)
        
        # 阶段1: 初始化点位
        rospy.loginfo(f"[{self.ns}] Stage1: Initializing setpoints...")
        for i in range(40):
            if rospy.is_shutdown() or not self.keep_running:
                return
            self.publish_sp()
            rate.sleep()
        
        # 阶段2: OFFBOARD和解锁
        rospy.loginfo(f"[{self.ns}] Stage2: Offboard and arming...")
        while not rospy.is_shutdown() and self.keep_running:
            self.publish_sp()
            
            if not self.offboard_good and self.state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req > rospy.Duration(2.0)):
                if self.try_set_mode("OFFBOARD"):
                    self.offboard_good = True
                    rospy.loginfo(f"[{self.ns}] Offboard success!")
                self.last_req = rospy.Time.now()
            
            if not self.arm_good and not self.state.armed and (rospy.Time.now() - self.last_req > rospy.Duration(2.0)):
                if self.try_arm(True):
                    self.arm_good = True
                    rospy.loginfo(f"[{self.ns}] Armed success!")
                self.last_req = rospy.Time.now()
            
            if self.offboard_good and self.arm_good:
                break
                
            rate.sleep()
        
        # 阶段3: 起飞
        rospy.loginfo(f"[{self.ns}] Stage3: Taking off...")
        target_height = 2.0
        takeoff_duration = 5.0
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
        
        rospy.loginfo(f"[{self.ns}] Reached target height!")
        
        # 阶段4: DRL避障 - 纯速度控制
        rospy.loginfo(f"[{self.ns}] Stage4: Starting DRL navigation (velocity control)...")
        while not rospy.is_shutdown() and self.keep_running:
            if self.image_ready and self.current_image is not None:
                try:
                    depth_map, depth_msg = self.depth_estimator.process_ros_image(self.current_image)
                    if depth_map is not None:
                        if depth_msg:
                            self.pub_depth.publish(depth_msg)
                        vel_cmd = self.drl_agent.make_decision(depth_map)
                        
                        # 添加高度控制到速度指令
                        vel_cmd.linear.z = self._height_control()
                        self.publish_vel(vel_cmd)
                        
                        rospy.loginfo_throttle(2, f"[{self.ns}] Control - x:{vel_cmd.linear.x:.2f}, y:{vel_cmd.linear.y:.2f}, z:{vel_cmd.linear.z:.2f}")
                    self.image_ready = False
                except Exception as e:
                    rospy.logerr(f"[{self.ns}] Processing error: {e}")
                    self.image_ready = False
            
            rate.sleep()

    def _height_control(self):
        """简单的高度控制PID"""
        if not self.local_pose_received:
            return 0.0
        
        target_height = 2.0
        current_height = self.local_pose.pose.position.z
        height_error = target_height - current_height
        
        control_output = height_error * 0.5  # P控制
        control_output = max(min(control_output, 0.5), -0.5)  # 限制输出范围
        
        return control_output

    def cb_image(self, msg):
        """图像回调"""
        try:
            self.current_image = msg
            self.image_ready = True
        except Exception as e:
            rospy.logerr(f"[{self.ns}] Image callback error: {e}")

    def cb_pose(self, msg):
        """定位数据回调"""
        self.local_pose = msg
        self.local_pose_received = True

    def cb_state(self, msg):
        """状态回调"""
        self.state = msg

    def try_set_mode(self, mode):
        """尝试设置模式"""
        try:
            resp = self.cli_mode(custom_mode=mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"[{self.ns}] Set mode failed: {e}")
            return False
        
    def try_arm(self, arm):
        """尝试解锁"""
        try:
            resp = self.cli_arm(value=arm)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"[{self.ns}] Arming failed: {e}")
            return False
        
    def publish_sp(self):
        """发布位置设置点"""
        self.sp.header.stamp = rospy.Time.now()
        self.sp.header.frame_id = "base_link"
        self.pub_sp.publish(self.sp)

    def publish_vel(self, vel_cmd):
        """发布速度指令"""
        self.pub_vel.publish(vel_cmd)
        
    def stop(self):
        """停止无人机运行"""
        self.keep_running = False

class MultiDroneController:
    def __init__(self):
        self.drones = [
            DroneUnit('wmy_iris_camRay0'),
            # DroneUnit('wmy_iris_camRay1'),
            # DroneUnit('wmy_iris_camRay2'),
            
        ]
        rospy.loginfo("Multi Drone Controller initialized - all drones running independently!")

    def run(self):
        """主循环"""
        try:
            rospy.loginfo("All drones are running independently. Press Ctrl+C to stop.")
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down all drones...")
            for drone in self.drones:
                drone.stop()
            rospy.loginfo("All drones stopped.")

if __name__ == '__main__':
    try:
        rospy.init_node('multi_drone_controller', anonymous=True)
        rospy.loginfo("Multi Drone Controller starting...")
        controller = MultiDroneController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Multi drone controller shutdown.")
    except Exception as e:
        rospy.logerr(f"Error in multi-drone controller: {str(e)}")