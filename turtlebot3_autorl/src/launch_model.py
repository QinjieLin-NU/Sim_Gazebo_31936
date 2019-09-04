#!/usr/bin/env python
import os
import time
import sys
import re
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

def generate_Cmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,loadEp,gui,stageId,realWorldFlag='false'):
    cmd_args = generate_argCmds(argNames,argValues)
    cmd_args =cmd_args + generate_fileName(fileId)
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gzURI)
    cmd_turtleType = "export TURTLEBOT3_MODEL=%s"%turType
    cmd_gpuId = "export CUDA_VISIBLE_DEVICES='%d'"%gpuID
    cmd_launchWorld = "roslaunch turtlebot3_autorl %s gui:=\"%s\" "%(gzfileName,gui) 
    cmd_load = " train:=\"false\" load:=\"true\" load_ep:=%d gui:=\"%s\" "%(loadEp,gui)
    cmd_stage = " stage:=%d"%stageId
    cmd_worldFlag = " real_world:=%s"%realWorldFlag
    cmd_launchRL = "roslaunch turtlebot3_autorl %s %s %s"%(rlfileName,cmd_args,cmd_stage) + cmd_load + cmd_worldFlag
    cmd_bash = "exec bash"
    cmds_world = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_turtleType+";"+cmd_launchWorld+";"+cmd_bash
    cmds_RL = cmd_gpuId+";"+cmd_launchRL+";"+cmd_bash
    if(realWorldFlag == 'false'):
        cmds_RL = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_gpuId+";"+cmd_turtleType+";"+cmd_launchRL+";"+cmd_bash

    return cmds_world,cmds_RL

# def generate_StageCmds(rosURI,gzURI,turType,gpuID,fileId,gzfileName,rlfileName,loadEp,gui,stageId,realWorldFlag='false')
#     cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
#     cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gzURI)
#     cmd_turtleType = "export TURTLEBOT3_MODEL=%s"%turType
#     cmd_gpuId = "export CUDA_VISIBLE_DEVIES='%d'"%gpuID
#     cmd_launchWorld = "roslaunch turtlebot3_autorl %s gui:=\"%s\" "%(gzfileName,gui) 
#     cmd_stage = " stage:=%d"%stageId
#     cmd_bash = "exec bash"
#     cmds_world = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_turtleType+";"+cmd_launchWorld+";"+cmd_bash
#     cmds_RL = cmd_gpuId+";"+cmd_launchRL+";"+cmd_bash
#     cmd_launchRL = "roslaunch turtlebot3_autorl %s %s"%(rlfileName,cmd_stage) + cmd_load + cmd_worldFlag
#     if(realWorldFlag == 'false'):
#         cmds_RL = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_gpuId+";"+cmd_turtleType+";"+cmd_launchRL+";"+cmd_bash
#     return cmds_world,cmds_RL

def get_params(fileId):
    fileLoc = os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/saved_model_lin/autoRL_%d"%fileId+"/data.txt"
    ignore = True
    with open(fileLoc, 'r') as filehandle:  
        for line in filehandle:
            if ignore:
                ignore =False
                model=line[:-1]
                p = re.compile(r'\d+\.\d+')  # Compile a pattern to capture float values
                model = [float(i) for i in p.findall(line)]  # Convert strings to float
            else:
                break
    return model

def get_params_v2(dirName,fileId):
    fileLoc = os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/%s/autoRL_%d"%(dirName,fileId)+"/data.txt"
    ignore = True
    if not os.path.isfile(fileLoc):
        print("not exist parameter file")
        return False
    with open(fileLoc, 'r') as filehandle:  
        for line in filehandle:
            if ignore:
                ignore =False
                model=line[:-1]
                p = re.compile(r'\d+\.\d+')  # Compile a pattern to capture float values
                model = [float(i) for i in p.findall(line)]  # Convert strings to float
            else:
                break
    return model

def test_model(rosURI,gzURI,fileLoc,fileId,loadEp,gzfileName,stageId,rlfileName,realWorldFlag):
    argNames =  ["dis_weight","lin_weight","obs_weight","ori_weight","col_weight","goal_weight","rot_weight","laser_step","ep_len","complex_nn","step_time"]
    params = get_params_v2(fileLoc,fileId)
    if(params == False):
        argNames = []
        params = []
    argValues =  params
    gpuID = 1
    gui = "true"
    turType = "waffle_pi"
    cmds_world,cmds_RL = generate_Cmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,loadEp,gui,stageId,realWorldFlag)

    print("cmd_world",cmds_world)
    print("cmd_RL",cmds_RL)
    if(realWorldFlag == 'false'):
        print("simulation world")
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_world)
    time.sleep(3)
    os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_RL)

def test_guoModel(rosURI,gzURI,fileLoc,fileId,loadEp,gzfileName,stageId,rlfileName,realWorldFlag):
    gpuID = 1
    gui = "true"
    turType = "waffle_pi"
    fileName = "Stage1_%d"%fileId
    cmds_world,cmds_rl = generate_StageCmds(rosURI,gzURI,turType,gpuID,fileId,gzfileName,rlfileName,loadEp,gui,stageId,realWorldFlag)
    print(cmds_world,cmds_rl)
    if(realWorldFlag == 'false'):
        print("simulation world")
        # os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_world)
    time.sleep(3)
    # os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_rl)

if __name__ == "__main__":
    rosURI = 11315
    gzURI = 11355
    fileId = 815
    loadEp = 400
    gpuID = 1
    gui = "true"
    stageId = 5
    argNames =  ["dis_weight","lin_weight","obs_weight","ori_weight","col_weight","goal_weight","rot_weight","laser_step","ep_len","complex_nn","step_time"]
    gzfileName = "turtlebot3_office_envRL1.launch"
    rlfileName = "turtlebot3_PPO_v2.launch"
    params = get_params(fileId)
    argValues =  params
    turType = "waffle_pi"
    cmds_world,cmds_RL = generate_Cmds(rosURI,gzURI,turType,argNames,argValues,gpuID,fileId,gzfileName,rlfileName,loadEp,gui,stageId)

    print("cmd_world",cmds_world)
    print("cmd_RL",cmds_RL)
    os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_world)
    time.sleep(3)
    os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_RL)

