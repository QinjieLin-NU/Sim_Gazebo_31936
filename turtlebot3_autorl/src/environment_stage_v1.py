#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import time
from std_msgs.msg import Int8
from stage_world1 import StageWorld

class Env():
    def __init__(self,index=-1):
        if(index == -1):
            print("########## index == -1 ##########")
            self.cmd_topic = "/cmd_vel"
            self.odom_topic = "/odom"
            self.scan_topic = "/base_scan"
            self.crash_topic = "/is_crashed"            
        else:
            self.cmd_topic = "robot_"+str(index)+"/cmd_vel"
            self.odom_topic = "robot_"+str(index)+"/odom"
            self.scan_topic = "robot_"+str(index)+"/base_scan"
            self.crash_topic = "robot_"+str(index)+"/is_crashed"
        self.reset_service = 'reset_positions'
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.lastDistance = 0
        self.currentDistance = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.getOdometry)
        self.sub_crash = rospy.Subscriber(self.crash_topic, Int8, self.crash_callback)
        self.reset_stage = rospy.ServiceProxy(self.reset_service, Empty)
        self.respawn_goal = Respawn()
        self.resetX = rospy.get_param('/x_pos',0.0)
        self.resetY = rospy.get_param('/y_pos',0.0)
        self.resetZ = rospy.get_param('/z_pos',0.0)
        self.resetYaw = rospy.get_param('/yaw_angle',0.0)
        self.resetQua = quaternion_from_euler(0.0,0.0,self.resetYaw)
        self.startTime = time.time()
        self.endTime = 0
        self.numSuccess = 0
        self.dis_weight = rospy.get_param('/dis_weight',0.0)
        self.lin_weight =  rospy.get_param('/lin_weight',0.0)
        self.obs_weight =  rospy.get_param('/obs_weight',0.0)
        self.ori_weight =  rospy.get_param('/ori_weight',0.0)
        self.col_weight =  rospy.get_param('/col_weight',0.0)
        self.goal_weight =  rospy.get_param('/goal_weight',0.0)
        self.rot_weight = rospy.get_param('/rot_weight',0.0)
        self.laserStep = rospy.get_param('/laser_step',60)
        self.stepTime = rospy.get_param('/step_time',0.5)
        self.realWorldFlag =  rospy.get_param('/real_world')
        self.simScale = 500.0
        if(self.realWorldFlag):
            self.simScale = 1.0
        rospy.sleep(1.)

    def crash_callback(self,flag):
        self.is_crashed = flag.data

    def get_crash_state(self):
        return self.is_crashed

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.3
        done = False
        full_scan_range = []
        range_dim = int(self.laserStep)
        num_dim = int(len(scan.ranges) / range_dim)
        
        # transform because the laser is different from real world and gazebo
        tranformed_scan = range(360)
        tranformed_scan[0:180] = scan.ranges[180:360]
        tranformed_scan[180:360] = scan.ranges[0:180]


        for i in range(len(tranformed_scan)):
            if tranformed_scan[i] == float('Inf'):
                full_scan_range.append(3.5)
            elif tranformed_scan[i] == 0.0:
                full_scan_range.append(3.5)
            elif np.isnan(tranformed_scan[i]):
                full_scan_range.append(0)
            else:
                full_scan_range.append(tranformed_scan[i])

        for i in range(num_dim):
            begin = i * range_dim
            end = (i + 1) * range_dim - 1
            if (end >= len(tranformed_scan)):
                end = -1
            scan_range.append(min(full_scan_range[begin:end]))

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)

        is_crash = self.get_crash_state()
        if(is_crash == 1):
            done = True

        self.lastDistance = self.currentDistance#qinjielin
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.currentDistance = current_distance#qinjielin
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done



    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-3]
        heading = state[-4]
        obs_min_range = state[-2]

        forward_distance = self.lastDistance - current_distance
        distance_reward = forward_distance*10
        if(forward_distance <= 0):
            distance_reward -= 0.5
        distance_reward = distance_reward * self.dis_weight

        rot_reward = 0
        if(math.fabs(action[0])>0.7):
            rot_reward = -0.5 * math.fabs(action[0]) * self.rot_weight

        ori_reward = 0
        ori_cos = math.cos(heading)
        if((math.fabs(heading) < 1.0) and (forward_distance>0)):
            ori_reward = ori_cos * 0.2 * self.ori_weight

        lin_reward = 0
        if((action[1]>=0.4) and (forward_distance>0)):
            lin_reward = action[1] * 0.5 * self.lin_weight

        obs_reward = 0
        if ((obs_min_range > 0.2) and (obs_min_range < 2.0)):
            obs_reward -= (2.0 - obs_min_range) * 1 * self.obs_weight #0.3

        reward = distance_reward + rot_reward + lin_reward + ori_reward + obs_reward



        if done:
            rospy.loginfo("Collision!!")
            reward = -10*self.col_weight #-15
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            self.numSuccess +=1
            reward = 10*self.goal_weight#15
            self.pub_cmd_vel.publish(Twist())
            self.reset()
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.currentDistance = self.goal_distance#qinjielin
            self.lastDistance = self.currentDistance
            self.get_goalbox = False
            self.endTime = time.time()
            exeTime = self.endTime - self.startTime
            self.startTime = time.time()
            print("exetime:",exeTime)

        return reward

    def step(self, action):
        max_angular_vel = 2.0
        max_linear_vel = 0.6
        scale = self.simScale
        ang_vel = ((action[0]/2))*max_angular_vel#0.15
        linear_vel = ((action[1]+2)/4)*max_linear_vel#0.15

        if(math.fabs(ang_vel)<0.5):
            ang_vel = 0
        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel #* scale # change when learning
        vel_cmd.angular.z = ang_vel #*scale  # change when learing
        transAction =[linear_vel,ang_vel]
        self.pub_cmd_vel.publish(vel_cmd)
        # rospy.sleep(0.001)
        rospy.sleep(self.stepTime/scale)
        # time.sleep(self.stepTime)
        # vel_cmd = Twist()
        # if(not self.realWorldFlag):
            # self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(self.scan_topic, LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        reward = self.setReward(state, done, transAction)

        # print("state in env:",state)
        # print("action:",transAction)
        
        return np.asarray(state), reward, done

    def reset(self):
        # print("waiting service")
        # rospy.wait_for_service('gazebo/reset_simulation')
        # # print("got service")
        # try:
        #     self.reset_proxy()
        # except (rospy.ServiceException) as e:
        #     print("gazebo/reset_simulation service call failed")
        if(self.realWorldFlag):
            print("just stop turtlebot in reset!")
            self.stop_turtlebot3()
        else:
            print("reset turtlebot in reset!")
            self.reset_turtlebot3()

        # print("getting scan")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(self.scan_topic, LaserScan, timeout=5)
            except:
                # print("getting scan pass")
                pass

        # print("got scan")

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        
        # print("init goal done ")

        self.goal_distance = self.getGoalDistace()
        self.currentDistance = self.goal_distance#qinjielin
        self.lastDistance = self.currentDistance
        self.startTime = time.time()
        state, done = self.getState(data)
        return np.asarray(state)

    def reset_turtlebot3(self):
        self.reset_stage()
        rospy.sleep(0.5)
        return
    
    def stop_turtlebot3(self):
        self.pub_cmd_vel.publish(Twist())