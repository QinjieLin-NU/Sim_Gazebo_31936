#!/usr/bin/env python

import rospy
import  sensor_msgs.msg
import nav_msgs.msg
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

class MapImagePub(object):
    def __init__(self):
        self.image_pub = rospy.Publisher("image", Image, queue_size=1)
        self.res_image = Image()
        rospy.Subscriber("/map", OccupancyGrid, self.occmap_to_image, queue_size=1)
        self.MapFlag = False

    def occmap_to_image(self,msg):
        self.res_image.height = msg.info.height
        self.res_image.width = msg.info.width
        self.res_image.encoding = 'mono8'  # waiting
        self.res_image.is_bigendian = 0  # waiting
        self.res_image.step = msg.info.width  # waiting
        i = 0
        self.res_image.data = range(0, msg.info.height * msg.info.width)
        for grid in msg.data:
            if (grid == 100):
                self.res_image.data[i] = 0
            elif(grid ==0):
                self.res_image.data[i] = 175
            else:
                self.res_image.data[i] = 255
            i = i + 1
        # print(res_image.data)
        self.image_pub.publish(self.res_image)
        self.MapFlag=True
        # print("succcessfully")
        return

    def pub_mapImage(self):
        if(self.MapFlag):
            self.image_pub.publish(self.res_image)


def occmap_to_mat(src_map):
    return

def mat_to_image(src_mat):
    return


if __name__ == '__main__':
    rospy.init_node("occmap_to_image")
    map_pub=MapImagePub()
    while(True):
        map_pub.pub_mapImage()
    rospy.spin()

