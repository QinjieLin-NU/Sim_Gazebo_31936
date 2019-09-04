#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ipa_building_msgs.msg import RoomExplorationActionGoal
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path

class cpp_solver(object):
    def __init__(self):
        self.pub = rospy.Publisher('/room_exploration/room_exploration_server/goal', RoomExplorationActionGoal, queue_size=10)
        self.sub = rospy.Subscriber("/image", Image, self.callback)
        self.mapImage = None
        self.MapRecFlag =False
        self.GoalSengFlag = False


    def callback(self,msg):
        if(not self.MapRecFlag):
            self.mapImage = msg
            self.MapFlag = True
        return 

    def talker(self):
        if(self.MapRecFlag and not self.GoalSengFlag):
            pub_msg = RoomExplorationActionGoal()
            pub_msg.goal.input_map = self.mapImage
            pub_msg.goal.map_resolution = 0.05
            self.pub.publish(pub_msg)
            GoalSengFlag = True
        return

class path_cpp(object):
    def __init__(self):
        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        self.RecFlag =False 
        self.cppPath =None
        rospy.Subscriber("/room_exploration_server/coverage_path", Path, self.callback)



    def talker(self,data):
        numPath = len(data.poses)
        for i in range(numPath):
            pub_msg = MoveBaseActionGoal()
            pub_msg.goal.target_pose = data.poses[i]
            pub_msg.header = data.header
            pub_msg.goal.target_pose.header.frame_id = 'map'
            pub_msg.header.frame_id  = 'map'
            # pub_msg.goal.target_pose.header.frame_id = data.poses[i].header.frame_id
            pub_msg.goal.target_pose.pose.position.x = data.poses[i].pose.position.x-2
            pub_msg.goal.target_pose.pose.position.y = data.poses[i].pose.position.y-2
            pub_msg.goal.target_pose.pose.position.z = data.poses[i].pose.position.z
            # pub_msg.goal.target_pose.pose.orientation = data.poses[i].pose.orientation
            if(self.RecFlag):
                    self.pub.publish(pub_msg)
                    rospy.sleep(2)
                    print(pub_msg)
                    # flag=True                
        self.RecFlag=False

    def callback(self, data):
        self.RecFlag = True
        self.cppPath=data
        self.talker(data)
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        


if __name__ == '__main__':
    rospy.init_node('test_cpp', anonymous=True)
    p = path_cpp()
    rospy.spin()
