#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from cStringIO import StringIO
from gazebo_utils import deleteModel
import time

class FancyFurniture(object):
    def __init__(self,file=''):
        self.name=''
        self.pos=Point()#pos of s,y,z
        self.angle=0#angle of yaw
        self.sdfFileLoc=file
        self.xmlLines=self.readXML(self.sdfFileLoc)

    def readXML(self,fileLoc):
        f = open(fileLoc, 'rw+')
        f.seek(0)
        lines = []
        for line in f:
            lines.append(line.rstrip('\n'))
        f.close()
        return lines

    def writeXML(self,fileLoc,lines):
        f = open(fileLoc, 'rw+')
        f.seek(0)
        for line in lines:
            print line
            f.writelines(line + '\n')
        f.close()

    def spawnXML(self,pos,angle,name):
        lines=[]
        self.name=name
        self.angle=angle
        self.pos=pos
        # < origin        xyz = "0 0 0.0"        rpy = "0 0 0" / >
        poseline='<origin xyz= \"'+str(pos.x)+' '+str(pos.y)+' '+str(pos.z)+ "\"  rpy= \""+' 0.0 0.0 '+str(angle)+ '\" />'
        nameline='<robot name=\"'+str(name)+'\" static= '+  '\"true\">'
        for line in self.xmlLines:
            if (line.find('origin') is not -1) :
                line=poseline
            if (line.find('robot name') is not -1):
                line = nameline
            lines.append(line)
            # print line
        # print  lines
        return lines

    def spawnModel(self,pos,angle,name):
        ##pos is the x,y,z postion of the model, angel is the yaw of the model, angle is from x cordinate
        ## size is the length of x,y,z,
        ## name is the name of model
        lines = self.spawnXML(pos,angle,name)
        # self.writeXML(self.sdfFileLoc,lines)

        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0
        # f = open(self.sdfFileLoc, 'r')
        sdf = StringIO(''.join(lines)).read()

        rospy.wait_for_service('gazebo/spawn_urdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        spawn_model_prox(self.name, sdf, " ", initial_pose, "world")

        # f.close()
        return

if __name__ == '__main__':
    rospy.init_node('insert_furniture')

    trashLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/trash.sdf'
    bookshelfLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/bookshelf/model.sdf'
    cafeTableLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/cafe_table/model.sdf'
    cudeLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/cube_20k/model.sdf'
    tableLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table/model.sdf'
    tableMarbleLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table_marble/model.sdf'
    tableMarbleLoc2 = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table_marble/model-1_4.sdf'

    bed_urdf="/home/qinjielin/RL_Ws/turtleBot3_ws/src/cob_gazebo_objects/objects/bed.urdf"


    nameBefore="my_fur"
    pos=Point()
    pos.x=3
    pos.y=3
    pos.z=0
    angle=0.68

    fm=FancyFurniture(bed_urdf)
    fm.spawnModel(pos,angle,nameBefore)

    time.sleep(5)

    deleteModel(nameBefore)