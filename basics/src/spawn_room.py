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


class SpawnRoom(object):
    def __init__(self):
        rospy.Subscriber("/convex_hull/hulls", ContourArrayStamped, self.hullCallback, queue_size=1)
        # rospy.Subscriber("/find_contours/contours", ContourArrayStamped, self.hullCallback, queue_size=1)
        self.wallLoc = rospy.get_param("wall_file",'/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/wall.sdf')
        self.models = []
        self.subscribedMsg = ContourArrayStamped()
        self.wallWidth = rospy.get_param("wall_width",2.5) #self set
        self.wallHeight = rospy.get_param("wall_height",10.0)#self set
        self.wallPosZ = self.wallHeight/2
        self.resolution = rospy.get_param("map_resolutions",0.1)#self set
        self.msgFlag=False
        self.mapFlag=False
        self.cylinderLoc= rospy.get_param("cylinder_file",'/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table/model.sdf')


    def hullCallback(self,msg):
        # print(msg.contours)
        if(not self.msgFlag):
            self.msgFlag = True
            self.subscribedMsg = msg
        return

    def spawnMap(self):
        if(self.msgFlag):
            msg = self.subscribedMsg
            for contour in msg.contours:
                points = contour.points
                numP = len(points)
                if(numP>2):
                    coords = ()
                    for point in points:
                        coords = coords + ((point.x * self.resolution, point.y * self.resolution),)
                    polygon = Polygon(coords)
                    print ("area is: "+str(polygon.area)+" centorid is: "+str(polygon.centroid.x)+", "+str(polygon.centroid.y))
                    if(polygon.area>30.0 ):
                        for i in range(1, numP):
                            self.spawnWalls(points[i - 1], points[i])
                        self.spawnWalls(points[numP - 1], points[0])
                    else:
                        self.spawnCylinder(polygon.centroid,math.sqrt(polygon.area))
                else:
                    for i in range(1, numP):
                        self.spawnWalls(points[i - 1], points[i])
                    self.spawnWalls(points[numP - 1], points[0])
            self.mapFlag=True


    def spawnCylinder(self,cen,rad):
        name=''
        size = Point()
        pos = Point()
        angle = 0

        name = self.getNextName()
        print ("name: "+name+'')

        size.x = rad
        size.y = rad
        size.z = 1.0
        print ("size: "+str(size.x)+' '+str(size.y)+' '+str(size.z))

        pos.x = cen.x
        pos.y = cen.y
        pos.z = 0.0
        print ("pose: "+str(pos.x)+' '+str(pos.y)+' '+str(pos.z))


        angle = 0
        print ("angle: "+str(angle))

        cylindermodel = CylinderModel(self.cylinderLoc)
        cylindermodel.spawnModel(pos, angle, size, name)
        self.models.append(cylindermodel)
        print ("***********spawned model: "+self.models[-1].name+" successfully***************")

        return

    def spawnWalls(self,point1,point2):
        name=''
        size = Point()
        pos = Point()
        angle = 0

        name = self.getNextName()
        print ("name: "+name+'')

        size.x = self.computelength(point1,point2) * self.resolution
        size.y = self.wallWidth * self.resolution
        size.z = self.wallHeight * self.resolution
        print ("size: "+str(size.x)+' '+str(size.y)+' '+str(size.z))

        pos.x = ((point1.x+point2.x)/2) * self.resolution
        pos.y =((point1.y+point2.y)/2) * self.resolution
        pos.z = self.wallPosZ * self.resolution
        print ("pose: "+str(pos.x)+' '+str(pos.y)+' '+str(pos.z))


        angle = self.computeAngle(point1,point2)
        print ("angle: "+str(angle))

        wallmodel = WallModel(self.wallLoc)
        wallmodel.spawnModel(pos, angle, size, name)
        self.models.append(wallmodel)
        print ("***********spawned model: "+self.models[-1].name+" successfully***************")

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

    def computelength(self,point1,point2):
        length = 0
        deltaX = (point1.x-point2.x)
        deltaY = (point1.y-point2.y)
        length = math.sqrt(deltaX*deltaX+deltaY*deltaY)
        return length

    def computeAngle(self,point1,point2):
        angle=0
        deltaX = (point2.x - point1.x)
        deltaY = (point2.y - point1.y)
        # print("deltax: "+str(deltaX)+" dealtaY: "+str(deltaY))
        if(math.fabs(deltaX) <0.00001):
            # angle=90
            angle = math.pi/2
        else:
            tan = deltaY/deltaX
            # angle = 180*math.atan(tan)/math.pi
            angle = math.atan(tan)
        return angle

if __name__ == '__main__':
    rospy.init_node("spawn_room")
    test = SpawnRoom()
    while(not test.mapFlag):
        test.spawnMap()
    time.sleep(50)
    test.deleteModel()
    rospy.spin()

