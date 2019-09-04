#!/usr/bin/env python
import os
import time
import sys
import re
# os.system("gnome-terminal -e 'bash -c \"export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_RL turtlebot3_stage_2_envRL.launch; exec bash\"'")

def generate_argCmd(argName,argValue):
    flag = isinstance(argValue, float) 
    cmd = None
    if(flag):
        cmd = " %s:=%f"%(argName,argValue)
    else:
        cmd = " %s:=\"%s\""%(argName,argValue)
    return cmd

def generate_argCmds(argNames,argValues):
    num = len(argNames)
    cmds = " "
    for i in range(num):
        cmd = generate_argCmd(argNames[i],argValues[i])
        cmds = cmds + cmd
    return cmds

def getArgs():
    argvs = sys.argv
    rosStartPort = int(argvs[1])
    gzStartPort = int(argvs[2])
    numPort = int(argvs[3])
    ParamStartId = int(argvs[4])
    fileStartId = int(argvs[5])
    gpuId = int(argvs[6])
    return gzStartPort,rosStartPort,numPort,ParamStartId,fileStartId,gpuId

def readData(fileName,dirName):
    rList = []
    sList=[]
    model = ""
    ignore = True
    dirPath = dirName + fileName
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
    modelId = -1
    if(lastEp >0):
        modelId = lastEp - (lastEp % 25)
    return rList,sList,model,modelId

def getModelParams(model):
        p = re.compile(r'\d+\.\d+')  # Compile a pattern to capture float values
        param = [float(i) for i in p.findall(model)]  # Convert strings to float
        return param

def getGoodParam(DirName,startId,endId):
    idList = range(startId,endId)
    successTimeList = []
    params = []
    modelIds = []
    fileIds = []
    for id in idList:
        rList,sList,model,modelId = readData("autoRL_%d"%id,DirName)
        if(len(sList)>0):
            successTimeList.append(sList[-1])
        if(len(rList)>0):
            if(rList[-1]>500):
                param = getModelParams(model)
                params.append(param)
                modelIds.append(modelId)
                fileIds.append(id)
    return params,modelIds,fileIds

def generate_Cmds(rosURI,gzURI,turType,gpuID,gzfileName,rlfileName,argNames,argValues):
    cmd_args = generate_argCmds(argNames,argValues)
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gzURI)
    cmd_turtleType = "export TURTLEBOT3_MODEL=%s"%turType
    cmd_gpuId = "export CUDA_VISIBLE_DEVICES='%d'"%gpuID
    cmd_launchWorld = "roslaunch turtlebot3_autorl %s"%gzfileName 
    cmd_launchRL = "roslaunch turtlebot3_autorl %s %s "%(rlfileName,cmd_args)
    cmd_bash = "exec bash"
    cmds_world = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_turtleType+";"+cmd_launchWorld+";"+cmd_bash
    cmds_RL = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_gpuId+";"+cmd_turtleType+";"+cmd_launchRL+";"+cmd_bash
    return cmds_world,cmds_RL

def generate_params(DirName,startId,endId,stageId):
    params,modelIds,fileIds=getGoodParam(DirName,startId,endId) 
    for id in range(len(params)):
        params[id].append(stageId)
        params[id].append("True")
        params[id].append("True")
        params[id].append("autoRL_%d"%fileIds[id])
        params[id].append(modelIds[id])
    return params
    



if __name__ == "__main__":
    gzStartPort,rosStartPort,numPort,ParamStartId,fileStartId,gpuId = getArgs()
    rosURI = rosStartPort
    gzURI = gzStartPort
    numTrains = numPort
    fileId = fileStartId
    gpuID = gpuId
    ######################define your launch file ########################################
    gzfileName = "turtlebot3_office_envRL1.launch"#"turtlebot3_stage_2_envRL.launch"#"turtlebot3_house_envRL.launch"
    rlfileName = "turtlebot3_PPO_v2.launch"
    argNames =  ["dis_weight","lin_weight","obs_weight","ori_weight","col_weight","goal_weight","rot_weight","laser_step","ep_len","complex_nn","step_time","stage","train","load","filename","load_ep"]
    #####################define the file range anf stage ID####################################
    DirName = os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/saved_model_lin/"
    startId = 770
    endId = 834
    stageId = 5
    params = generate_params(DirName,startId,endId,stageId)
    numParam = len(params)
    print(numParam)
    print(params)

    for i in range(numTrains):
        paramID = i + ParamStartId
        if(paramID >= numParam):
            break
        argValues =  params[paramID]
        turType = "burger"
        cmds_world,cmds_RL = generate_Cmds(rosURI,gzURI,turType,gpuID,gzfileName,rlfileName,argNames,argValues)

        print("cmd_world",cmds_world)
        print("cmd_RL",cmds_RL)
        print("param %d:",i,params[paramID])
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_world)
        time.sleep(3)
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmds_RL)

        rosURI +=1
        gzURI +=1
        fileId +=1


