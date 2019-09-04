#!/usr/bin/env python
import os
import time
# os.system("gnome-terminal -e 'bash -c \"export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_RL turtlebot3_stage_2_envRL.launch; exec bash\"'")

def generate_argCmd(argName,argValue):
    cmd = " %s:=%f"%(argName,argValue)
    return cmd

def generate_argCmds(argNames,argValues):
    num = len(argNames)
    cmds = " "
    for i in range(num):
        cmd = generate_argCmd(argNames[i],argValues[i])
        cmds = cmds + cmd
    return cmds

def generate_fileName(id):
    cmd = " filename:=\"autoRL_%d\""%id
    return cmd

def generate_Cmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId):
    cmd_args = generate_argCmds(argNames,argValues)
    cmd_args =cmd_args + generate_fileName(fileId)
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gzURI)
    cmd_turtleType = "export TURTLEBOT3_MODEL=%s"%turType
    cmd_gpuId = "export CUDA_VISIBLE_DEVICES='%d'"%gpuID
    cmd_launchWorld = "roslaunch turtlebot3_autorl turtlebot3_stage_2_envRL.launch" 
    cmd_launchRL = "roslaunch turtlebot3_autorl turtlebot3_PPO_house_v1.launch"+ cmd_args
    cmd_bash = "exec bash"
    cmds_world = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_turtleType+";"+cmd_launchWorld+";"+cmd_bash
    cmds_RL = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_gpuId+";"+cmd_turtleType+";"+cmd_launchRL+";"+cmd_bash

    return cmds_world,cmds_RL

def generate_params():
    # param_init = [-1,1,-1,1,1,1,60]
    allparams = []
    params_steps = [-0.5,-1]
    params_distance = [0.5,1]
    params_collisions = [-1,-2]
    params_turnings = [1]
    params_clearance = [1]
    params_goal = [1,2]
    params_laser = [60]
    for param1 in params_steps:
        for param2 in params_distance:
            for param3 in params_collisions:
                for param4 in params_turnings:
                    for param5 in params_clearance:
                        for param6 in params_goal:
                            for param7 in params_laser:
                                params=[]
                                params.append(param1)
                                params.append(param2)
                                params.append(param3)
                                params.append(param4)
                                params.append(param5)
                                params.append(param6)
                                params.append(param7)
                                allparams.append(params)
    return allparams


if __name__ == "__main__":
    rosURI = 11318
    gzURI = 11358
    numTrains = 16
    params = generate_params()
    print(params)
    for i in range(numTrains):
        turType = "burger"
        argNames =  ["theta_step","theta_distance","theta_collision","theta_turning","theta_clearance","theta_goal","laser_step"]
        argValues =  params[i]#[-0.2,1,-1,1,1,1,60]
        gpuID = (i/5) # every tf process take 20% of the gpu to train
        fileId = i

        cmds_world,cmds_RL = generate_Cmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId)
        # print("cmd_world",cmds_world)
        # print("cmd_RL",cmds_RL)
        print("param %d:",i,params[i])
        os.system("gnome-terminal -e 'bash -c \"%s\" '"%cmds_world)
        time.sleep(3)
        os.system("gnome-terminal -e 'bash -c \"%s\" '"%cmds_RL)

        rosURI +=1
        gzURI +=1


