#!/usr/bin/env python
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

rospy.init_node('test_sim')
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
time.sleep(1)
vel_cmd = Twist()
vel_cmd.linear.x = 0.1 
pub_cmd_vel.publish(vel_cmd)
time.sleep(1)
vel_cmd = Twist()
pub_cmd_vel.publish(vel_cmd)