#!/usr/bin/env python
import os
import numpy as np
import random
import time
import sys
import rospy
from os import listdir
from os.path import isfile, join
import glob

def getTeleopCMd():
    data = None
    a = np.zeros(shape=(2,))
    while data is None:
        try:
            data = rospy.wait_for_message('humanCmd', Twist, timeout=1)
        except:
            # print("getting scan pass")
            pass
    a[0]=data.angular.z
    a[1]=data.linear.x
    max_linear_vel = 0.3
    a[1]=(data.linear.x/max_linear_vel) *4 - 2 
    return a

def writeData(fileLoc,dirName,ep,r,successTimes):
    # filename = os.path.dirname(os.path.abspath(__file__))+"/saved_model/"+dirName+"/data.txt"
    filename = fileLoc+dirName+"/data.txt"
    dirname =  os.path.dirname(filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    with open(filename, 'a') as filehandle:  
        filehandle.write('ep: %d reward: %d successful time: %d\n' % (ep,r,successTimes))

def writeInfo(fileLoc,dirName,info):
    # filename = os.path.dirname(os.path.abspath(__file__))+"/saved_model/"+dirName+"/data.txt"
    filename = fileLoc+dirName+"/data.txt"
    dirname =  os.path.dirname(filename)
    print("file Loc:",fileLoc)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    with open(filename, 'a') as filehandle:   
        filehandle.write('model info: %s \n' % (info))

def saveModelInfo(fileLoc,fileName):
    argNames =  ["dis_weight","lin_weight","obs_weight","ori_weight","col_weight","goal_weight","rot_weight","laser_step","ep_len","complex_nn","step_time"]
    argValues = getArgInfo(argNames)
    info = ""
    numParam = len(argNames)
    for i in range(numParam):
        info = info + " "+str(argNames[i])+":="+str(argValues[i])
    writeInfo(fileLoc,fileName,info)

def getArgInfo(argNames):
    res = []
    for argName in argNames:
        weight = rospy.get_param('/%s'%argName,0.0)
        res.append(weight)
    return res

def launch_stageWorlds(dirName="d",gui="false",rosURI=11311):
    if(dirName == "d"):
        dirName = os.path.dirname(os.path.abspath(__file__))+"/../worlds/stage_map"
    else:
        dirName = os.path.dirname(os.path.abspath(__file__))+"/../worlds/%s"%dirName
    myList = glob.glob("%s/*.png"%dirName)
    for pngName in myList:
        cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
        cmd_stage = "roslaunch turtlebot3_autorl turtlebot3_stage_world.launch gui:=%s png_loc:=\"%s\""%(gui,pngName)
        commandLine = cmd_rosURI +";"+cmd_stage
        print(commandLine)
        os.system("gnome-terminal -e 'bash -c \"%s; exec bash\" '"%commandLine)
        rosURI = rosURI +1
        time.sleep(1)

if __name__ == "__main__":
    launch_stageWorlds(gui="true")
