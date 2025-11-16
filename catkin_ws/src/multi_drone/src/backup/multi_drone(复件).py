#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: wumengyu
"""

import rospy
import math
from geometry_msgs.msg import  PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class DroneUnit:
    def __init__(self, ns):
        self.ns = ns
        self.state = State()
        self.offboard_good = False
        self.arm_good = False
        self.last_req = rospy.Time.now()
            
        #publishers 
        self.pub_sp = rospy.Publisher('/' + self.ns + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        #subscribers
        self.sub_state = rospy.Subscriber('/' + self.ns + "/mavros/state", State, self.cb_state)
        #service clients
        rospy.wait_for_service('/' + ns + '/mavros/cmd/arming')
        self.cli_arm = rospy.ServiceProxy('/' + ns + '/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('/' + ns + '/mavros/set_mode')
        self.cli_mode = rospy.ServiceProxy('/' + ns + '/mavros/set_mode', SetMode)
        #setPoint
        self.sp = PoseStamped() #带时间戳的位置信息
        self.sp.pose.orientation.w = 1.0
    
    def cb_state(self, msg):
        self.state = msg

    def try_set_mode(self, mode):
        try:
            resp = self.cli_mode(custom_mode = mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False
        
    def try_arm(self, arm):
        try:
            resp = self.cli_arm(value = arm)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False
    def publish_sp(self):
        self.sp.header.stamp = rospy.Time.now()
        self.pub_sp.publish(self.sp)
        
class MultiDroneController:
    def __init__(self):
        self.drones = []
        self.drones.append(DroneUnit('wmy_iris_camRay0'))
        self.drones.append(DroneUnit('wmy_iris_camRay1'))
        self.drones.append(DroneUnit('wmy_iris_camRay2'))
        self.drones.append(DroneUnit('wmy_iris_camRay3'))
        self.drones.append(DroneUnit('wmy_iris_camRay4'))
        self.drones.append(DroneUnit('wmy_iris_camRay5'))
        self.drones.append(DroneUnit('wmy_iris_camRay6'))
        self.drones.append(DroneUnit('wmy_iris_camRay7'))
        self.drones.append(DroneUnit('wmy_iris_camRay8'))
        self.drones.append(DroneUnit('wmy_iris_camRay9'))

    def run(self):
        rate = rospy.Rate(20)

        rospy.loginfo("stage1: Initializing point...")
        for i in range(40):
            if rospy.is_shutdown():
                break
            # 为每架无人机发布初始位置
            for drone in self.drones:
                drone.publish_sp()
            rate.sleep()

        rospy.loginfo("stage2: offboarding and arming...")
        all_ready = False
        while not rospy.is_shutdown() and not all_ready:
            # 发布所有无人机的目标位置
            for drone in self.drones:
                drone.publish_sp()
            all_ready = True
            for drone in self.drones:
                if not drone.state.connected:
                    all_ready = False
                    continue

                if(not drone.offboard_good and drone.state.mode != "OFFBOARD" and (rospy.Time.now() - drone.last_req > rospy.Duration(2.0))):
                    if drone.try_set_mode("OFFBOARD"):
                        drone.offboard_good = True
                        rospy.loginfo("%s successfully offboard!" % drone.ns)
                    drone.last_req = rospy.Time.now()

                # 如果还没进入OFFBOARD模式，继续等待
                if not drone.offboard_good:
                    all_ready = False
                    continue
                
                # 尝试解锁
                if (not drone.arm_good and not drone.state.armed and 
                    (rospy.Time.now() - drone.last_req) > rospy.Duration(2.0)):
                    if drone.try_arm(True):
                        drone.arm_good = True
                        rospy.loginfo("%s successfully armed! ", drone.ns)
                    drone.last_req = rospy.Time.now()
                
                # 如果还没解锁，继续等待
                if not drone.arm_good:
                    all_ready = False
            
            rate.sleep()
        
        rospy.loginfo("All drones are ready, let's go!")

        rospy.loginfo("stage3: taking off to same height...")

        target_height = 2.0
        takeoff_duration = 5.0
        takeoff_start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = (rospy.Time.now() - takeoff_start_time).to_sec()

            if current_time > takeoff_duration:
                break

            # 计算当前高度
            progress = current_time / takeoff_duration
            current_height = target_height * progress

            for drone in self.drones:
                drone.sp.pose.position.z = current_height
                drone.publish_sp()

            rate.sleep()
        
        rospy.loginfo("All drones have reached the desired altitude!")

        while not rospy.is_shutdown():
            for drone in self.drones:
                drone.sp.pose.position.z = target_height
                drone.publish_sp()

            rate.sleep()

        rospy.loginfo("Mission accomplished!")

if __name__ == '__main__':
    try:
        rospy.init_node('multi_drone_controller', anonymous = True)
        rospy.loginfo("Multi Drone Controller is starting...")

        controller = MultiDroneController()
        controller.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("Multi drone controller shutdown. ")
    except Exception as e:
        rospy.logerr("Error in multi-drone crontroller: %s", str(e))
