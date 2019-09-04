#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from gazebo_utils import  deleteModel
import  time

rospy.init_node('insert_actor')

initial_pose = Pose()
initial_pose.position.x = 1
initial_pose.position.y = 1
initial_pose.position.z = 0

# loc='/home/qinjielin/RL_Ws/turtleBot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/model.sdf'
loc='/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/actor/actor.sdf'

f = open(loc,'r')

sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("actor0", sdff, " ", initial_pose, "world")

# MB_sdf = open('/home/qinjielin/RL_Ws/turtleBot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model-1_4.sdf','r')
# sdff_MB = MB_sdf.read()
# spawn_model_prox("my_MB", sdff_MB, " ", initial_pose, "world")
time.sleep(10)
deleteModel("actor0")
print("spawn successfully")
