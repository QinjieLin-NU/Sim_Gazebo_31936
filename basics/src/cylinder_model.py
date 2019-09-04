#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from cStringIO import StringIO
from gazebo_utils import deleteModel
import time

class CylinderModel(object):
    def __init__(self,file=''):
        self.name=''
        self.pos=Point()#pos of s,y,z
        self.size=Point()#size x,y,z, size of surface
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

    def spawnXML(self,pos,angle,size,name):
        lines=[]
        FirstPoseFlag = True
        self.name=name
        self.angle=angle
        pos.z=size.z/2.0
        self.pos=pos
        self.size=size
        poseline='      <pose frame=\'\'>'+str(pos.x)+' '+str(pos.y)+' '+str(pos.z)+ ' 0 0 '+str(angle)+ ' </pose>'
        nameline='  <model name=\''+str(name)+'\'>'
        legRadLine='            <radius>'+str(size.x)+' </radius>'
        legLenLine='            <length>'+str(size.z)+' </length>'

        for line in self.xmlLines:
            if (line.find('pose') is not -1) & FirstPoseFlag:
                FirstPoseFlag=False
                line=poseline
            if (line.find('model name') is not -1):
                line = nameline
            if (line.find('radius') is not -1):
                line = legRadLine
            if (line.find('length') is not -1):
                # print legLenLine
                line = legLenLine

            lines.append(line)
            # print line
        return lines

    def spawnModel(self,pos,angle,size,name):
        ##pos is the x,y,z postion of the model, angel is the yaw of the model, angle is from x cordinate
        ## size is the length of x,y,z,
        ## name is the name of model
        lines = self.spawnXML(pos,angle,size,name)
        # self.writeXML(self.sdfFileLoc,lines)

        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0
        # f = open(self.sdfFileLoc, 'r')
        sdf = StringIO(''.join(lines)).read()

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(self.name, sdf, " ", initial_pose, "world")

        # f.close()
        return

    def getLegPose(self,front,left):
        pos=Point()
        pos.x=front*((self.size.x/2.0)-(self.size.x/20.0))
        pos.y=left*((self.size.y/2.0)-(self.size.x/20.0))
        pos.z=-self.pos.z/2
        poseline='      <pose>'+str(pos.x)+' '+str(pos.y)+' '+str(pos.z)+ ' 0 0 0'+ ' </pose>'
        # print poseline
        return poseline

if __name__ == '__main__':
    rospy.init_node('insert_furniture')

    trashLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/trash.sdf'
    bookshelfLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/bookshelf/model.sdf'
    cafeTableLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/cafe_table/model.sdf'
    cudeLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/cube_20k/model.sdf'
    tableLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table/model.sdf'
    tableMarbleLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table_marble/model.sdf'
    tableMarbleLoc2 = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table_marble/model-1_4.sdf'
    cylinderLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/cylinder/model.sdf'

    nameBefore="my_fur"
    pos=Point()
    pos.x=2.0
    pos.y=2.0
    pos.z=0
    size=Point()
    size.x=0.5
    size.y=0.5
    size.z=2.0
    angle=0

    # fm=FurnitureModel(tableLoc)
    # fm.spawnModel(pos,angle,nameBefore)
    # tm=TableModel(tableLoc)
    # tm.spawnModel(pos,angle,size,nameBefore)

    cm=CylinderModel(cylinderLoc)
    cm.spawnModel(pos,angle,size,nameBefore)
    time.sleep(10)

    deleteModel(nameBefore)