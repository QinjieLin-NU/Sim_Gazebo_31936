#!/usr/bin/env python
import os
import time
import sys
import re
import matplotlib.pyplot as plt
import glob
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

def generate_RLcmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,stageId):
    cmd_args = generate_argCmds(argNames,argValues)
    cmd_args =cmd_args + generate_fileName(fileId)
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gzURI)
    cmd_turtleType = "export TURTLEBOT3_MODEL=%s"%turType
    cmd_gpuId = "export CUDA_VISIBLE_DEVICES='%d'"%gpuID
    cmd_launchWorld = "roslaunch turtlebot3_autorl %s"%gzfileName 
    cmd_stage = " stage:=%d"%stageId
    cmd_launchRL = "roslaunch turtlebot3_autorl %s %s %s "%(rlfileName,cmd_args,cmd_stage)
    cmd_bash = "exec bash"
    cmds_world = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_turtleType+";"+cmd_launchWorld+";"+cmd_bash
    cmds_RL = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_gpuId+";"+cmd_turtleType+";"+cmd_launchRL+";"+cmd_bash

    return cmds_RL

def generate_Gzcmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,stageId,worldName,gui):
    cmd_args = generate_argCmds(argNames,argValues)
    cmd_args =cmd_args + generate_fileName(fileId)
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gzURI)
    cmd_turtleType = "export TURTLEBOT3_MODEL=%s"%turType
    cmd_gpuId = "export CUDA_VISIBLE_DEVICES='%d'"%gpuID
    cmd_launchWorld = "roslaunch turtlebot3_autorl %s  gui:=%s world_file:=\"%s\" "%(gzfileName,gui,worldName) 
    cmd_stage = " stage:=%d"%stageId
    cmd_launchRL = "roslaunch turtlebot3_autorl %s %s %s "%(rlfileName,cmd_args,cmd_stage)
    cmd_bash = "exec bash"
    cmds_world = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_turtleType+";"+cmd_launchWorld+";"+cmd_bash
    return cmds_world

def generate_StageCmd(gui="false",rosURI=11311,pngName="stage_map"):
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
    cmd_stage = "roslaunch turtlebot3_autorl turtlebot3_stage_world.launch gui:=%s png_loc:=\"%s\""%(gui,pngName)
    commandLine = cmd_rosURI +";"+cmd_stage
    # time.sleep(1)
    return commandLine

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

def generate_params_v2():
    # allparams = []
    # params_dis = [1,2]#[1.5,2]
    # params_lin = [0.5,1]
    # params_obs = [0.5]
    # params_ori = [1]
    # params_col = [2,4]
    # params_goal = [2,4]
    # params_rot =[1]
    # params_laser = [60,45]
    # params_eplen = [200]
    # params_nn = [0,1]
    # params_stepTime = [0.5,0.2]#0.5,0.1
    allparams = []
    params_dis = [2,5,10]#[1.5,2]
    params_lin = [1]
    params_obs = [2,5,10]#[1.5,2]
    params_ori = [2]
    params_col = [5,10]#[2,1.5,3]
    params_goal = [5,8,10]#[2,3,4]
    params_rot =[0,1]
    params_laser = [60]
    params_eplen = [1000]
    params_nn = [0]
    params_stepTime = [0.5]#0.5,0.1
    for param1 in params_dis:
        for param2 in params_lin:
            for param3 in params_obs:
                for param4 in params_ori:
                    for param5 in params_col:
                        for param6 in params_goal:
                            for param7 in params_rot:
                                for param8 in params_laser:
                                    for param9 in params_eplen:
                                        for param10 in params_nn:
                                            for param11 in params_stepTime:
                                                params=[]
                                                params.append(param1)
                                                params.append(param2)
                                                params.append(param3)
                                                params.append(param4)
                                                params.append(param5)
                                                params.append(param6)
                                                params.append(param7)
                                                params.append(param8)
                                                params.append(param9)
                                                params.append(param10)
                                                params.append(param11)
                                                allparams.append(params)
    return allparams

def getArgs():
    argvs = sys.argv
    rosStartPort = int(argvs[1])
    gzStartPort = int(argvs[2])
    numPort = int(argvs[3])
    ParamStartId = int(argvs[4])
    fileStartId = int(argvs[5])
    gpuId = int(argvs[6])
    gzfileName = argvs[7]
    rlfileName =argvs[8]
    StageId = int(argvs[9])
    simulatorNames = str(argvs[10])
    dirName = str(argvs[11])
    gui = str(argvs[12])
    return gzStartPort,rosStartPort,numPort,ParamStartId,fileStartId,gpuId,gzfileName,rlfileName,StageId,simulatorNames,dirName,gui

def readData(dirName):
    rList = []
    sList=[]
    model = ""
    ignore = True
    dirPath = os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/saved_model_lin/"+dirName
    lastEp = -1
    if not os.path.exists(dirPath):
        return rList,sList,model
    with open('%s/data.txt'%dirPath, 'r') as filehandle:  
        for line in filehandle:
            if ignore:
                ignore =False
                model=line[:-1]
            else:
                line = line[:-1].split(" ")
                if(line[0]!="model"):
                    ep = int(line[1])
                    lastEp = ep
                    r = int(line[3])
                    s = int(line[6])
                    rList.append(r)
                    sList.append(s)
    return rList,sList,model

def getModelParams(model):
        p = re.compile(r'\d+\.\d+')  # Compile a pattern to capture float values
        param = [float(i) for i in p.findall(model)]  # Convert strings to float
        return param

def getGoodParam():
    idList = range(814,962)
    successTimeList = []
    params = []
    modelIds = []
    for id in idList:
        rList,sList,model = readData("autoRL_%d"%id)
        if(len(sList)>0):
            successTimeList.append(sList[-1])
        if(len(rList)>0):
            if(rList[-1]>200):
                print("model_%d"%id,model)
                param = getModelParams(model)
                params.append(param)
    return params


if __name__ == "__main__":
    gzStartPort,rosStartPort,numPort,ParamStartId,fileStartId,gpuId,gzfileName,rlfileName,StageId,simulatorName,dirName,gui = getArgs()
    rosURI = rosStartPort
    gzURI = gzStartPort
    numTrains = numPort
    fileId = fileStartId
    gpuID = gpuId
    LoadFlag = False
    argNames =  ["dis_weight","lin_weight","obs_weight","ori_weight","col_weight","goal_weight","rot_weight","laser_step","ep_len","complex_nn","step_time"]
    # stageId = 5
    # gzfileName = "turtlebot3_office_envRL.launch" #"turtlebot3_house_envRL.launch"#"turtlebot3_stage_2_envRL.launch"#"turtlebot3_house_envRL.launch"
    # rlfileName = "turtlebot3_DDPG_v1.launch"
    params = []
    modelIds = []
    # params = getGoodParam()
    params = generate_params_v2()
    numParam = len(params)
    print(params)
    dirName = os.path.dirname(os.path.abspath(__file__))+"/../worlds/%s"%dirName

    for i in range(numTrains):
        paramID = i + ParamStartId
        if(paramID >= numParam):
            break
        argValues =  params[paramID]
        turType = "waffle_pi"
        # cmds_world,cmds_RL = generate_Cmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,StageId)
        cmds_world = None
        if(simulatorName == "gazebo"):
            # dirName = os.path.dirname(os.path.abspath(__file__))+"/../worlds/%s"%dirName
            myList = glob.glob("%s/*.world"%dirName)
            for worldName in myList:
                cmds_RL = generate_RLcmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,StageId)
                cmds_world = generate_Gzcmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,StageId,worldName,gui)
                print("cmd_world",cmds_world)
                print("cmd_RL",cmds_RL)
                print("param %d:"%i,params[paramID])
                os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_world)
                time.sleep(3)
                os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_RL)
                rosURI +=1
                gzURI +=1
                fileId +=1
        else:
            # dirName = os.path.dirname(os.path.abspath(__file__))+"/../worlds/%s"%dirName
            myList = glob.glob("%s/*.png"%dirName)
            for pngName in myList:
                cmds_RL = generate_RLcmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,StageId)
                cmds_world = generate_StageCmd(gui,rosURI,pngName)
                print("cmd_world",cmds_world)
                print("cmd_RL",cmds_RL)
                print("param %d:"%i,params[paramID])
                os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_world)
                time.sleep(3)
                os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_RL)
                rosURI +=1
                gzURI +=1
                fileId +=1





