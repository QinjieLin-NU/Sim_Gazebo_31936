#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from cStringIO import StringIO
from gazebo_utils import deleteModel
import time
from furniture_model import FurnitureModel
from wall_model import  WallModel
from table_model import  TableModel
from cylinder_model import  CylinderModel
from opencv_apps.msg import ContourArrayStamped, Point2D
import  math
from shapely.geometry import Polygon
from model_fancyFurniture import FancyFurniture
from table_model import  TableModel


class SpawnFancyRoom(object):
    def __init__(self):
        self.models = []
        self.wallWidth = 0.1
        self.wallHeight = 1
        self.wallPosZ = self.wallHeight/2


    def spawnFurs(self,file,pos,angle):
        name = self.getNextName()
        model = FancyFurniture(file)
        model.spawnModel(pos,angle,name)
        self.models.append(model)
        return

    def spawnWalls(self,file,length,pos,angle):
        size=Point()
        size.x=length
        size.y=self.wallWidth
        size.z=self.wallHeight
        pos.z=self.wallPosZ

        name=self.getNextName()

        model = WallModel(file)
        model.spawnModel(pos,angle,size,name)
        self.models.append(model)
        return

    def spawnTables(self,file,size,pos,angle):
        name=self.getNextName()
        model=TableModel(file)
        model.spawnModel(pos,angle,size,name)
        self.models.append(model)
        return


    def getNextName(self):
        numModels = len(self.models)
        name = "element_"+ str(numModels)
        return name

    def deleteModel(self):
        numModels = len(self.models)
        for i in range(0,numModels):
            deleteModel(self.models[i].name)
        return



if __name__ == '__main__':
    wallFile='/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/wall.sdf'
    tableFile = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table/model.sdf'
    bedFile='/home/qinjielin/RL_Ws/turtleBot3_ws/src/cob_gazebo_objects/objects/bed.urdf'
    chairFile='/home/qinjielin/RL_Ws/turtleBot3_ws/src/cob_gazebo_objects/objects/chair_wicker.urdf'
    tableLRFile='/home/qinjielin/RL_Ws/turtleBot3_ws/src/cob_gazebo_objects/objects/table_living_room.urdf'
    tableLittleFile='/home/qinjielin/RL_Ws/turtleBot3_ws/src/cob_gazebo_objects/objects/table_bedside.urdf'
    shelfFile='/home/qinjielin/RL_Ws/turtleBot3_ws/src/cob_gazebo_objects/objects/shelf.urdf'
    chairIkFile='/home/qinjielin/RL_Ws/turtleBot3_ws/src/cob_gazebo_objects/objects/chair_ikea_borje.urdf'




    rospy.init_node("spawn_Fancyroom")
    test=SpawnFancyRoom()

    test.spawnWalls(wallFile,8-0.15,pos=Point(4,0,0),angle=0)
    test.spawnWalls(wallFile,5-0.15,pos=Point(5,-2.5,0),angle=1.57)
    test.spawnWalls(wallFile,5-0.15,pos=Point(7.5,-5.0,0),angle=0)
    test.spawnWalls(wallFile,10-0.15,pos=Point(10,0,0),angle=1.57)
    test.spawnWalls(wallFile,10-0.15,pos=Point(5.0,5.0,0),angle=0)
    test.spawnWalls(wallFile,5-0.15,pos=Point(0,2.5,0),angle=1.57)
    test.spawnWalls(wallFile,2-0.15,pos=Point(5.0,4.0,0),angle=1.57)

    test.spawnFurs(bedFile,pos=Point(1.3,4.3,0),angle=-3.14)
    test.spawnTables(tableFile,size=Point(3.0,1.0,0.03),pos=Point(2.5,1,1.0),angle=0)
    test.spawnFurs(chairFile,pos=Point(2.5,2,0),angle=0)

    test.spawnFurs(chairIkFile,pos=Point(6.5,-3.5,0),angle=-1.57)
    test.spawnFurs(bedFile,pos=Point(8.8,-3.9,0),angle=0)
    test.spawnFurs(tableLRFile,pos=Point(5.5,-4,0),angle=0)
    test.spawnFurs(tableLittleFile,pos=Point(9.5,-3,0),angle=0)


    test.spawnTables(tableFile,size=Point(2.0,2.0,0.03),pos=Point(7.5,2.5,1.0),angle=0)
    test.spawnFurs(chairIkFile,pos=Point(7.5,4,0),angle=0)
    test.spawnFurs(chairIkFile,pos=Point(7.5,1,0),angle=3.14)
    test.spawnFurs(chairIkFile,pos=Point(9,2.5,0),angle=-1.57)
    test.spawnFurs(chairIkFile,pos=Point(6,2.5,0),angle=1.57)


    # time.sleep(1500)
    # test.deleteModel()

