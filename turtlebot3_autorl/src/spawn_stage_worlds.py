#!/usr/bin/env python
import os
import rospy

def getPngLoc():
    res = rospy.get_param('/png_loc',"room0.png")
    return res

def spawn_world(pngLoc,fileLoc):
    f = open(fileLoc, 'rw+')
    f.seek(0)
    lines = []
    for line in f:
        lines.append(line.rstrip('\n'))
    f.close()

    for i in range(len(lines)):
        if (lines[i].find('bitmap') is not -1):
            lines[i] = "  bitmap \"%s\""%pngLoc
          

    f = open(fileLoc, 'rw+')
    f.seek(0)
    for line in lines:
        f.writelines(line + '\n')
    f.close()

def start_stage(fileLoc):
    gui = rospy.get_param('/gui',False)
    commandLine = ""
    if(gui):
        commandLine = "rosrun stage_ros_add_pose_and_crash stageros %s"%fileLoc
    else:
        commandLine = "rosrun stage_ros_add_pose_and_crash stageros -g %s"%fileLoc
    os.system(commandLine)

if __name__ == "__main__":
    rospy.init_node('spawn_stage_world')
    fileLoc = os.path.dirname(os.path.abspath(__file__))+"/../worlds/stage_world.world"
    pngLoc = getPngLoc()
    spawn_world(pngLoc,fileLoc)
    start_stage(fileLoc)
    

##get the png location 

##then spawn the world file, and launch the stage