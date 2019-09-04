#!/usr/bin/env python
import sys
import os

rosBeginPort = (11316 +200)
gzBeginPort = (11356 + 500)
numPort = 2# RL occupy 10% GPU
numGpu = 1
ParamStartID = 0
fileBeginId = 10000
GPUId = 0
continueRL = False
gzfileName = "turtlebot3_office_envRL.launch" #"turtlebot3_house_envRL.launch"#"turtlebot3_stage_2_envRL.launch"#"turtlebot3_house_envRL.launch"
rlfileName = "turtlebot3_PPO_v2.launch"
StageId = 5
for i in range(numGpu):
    if(continueRL):
        cmd_terminal = "python turtlebot3_continueRL.py %d %d %d %d %d %d"%(rosBeginPort,gzBeginPort,numPort,ParamStartID,fileBeginId,GPUId)
    else:
        cmd_terminal = "python turtlebot3_autoRL_v2.py %d %d %d %d %d %d %s %s %d"%(rosBeginPort,gzBeginPort,numPort,ParamStartID,fileBeginId,GPUId,gzfileName,rlfileName,StageId)
    print(cmd_terminal) 
    os.system("gnome-terminal -e 'bash -c \"%s; exec bash\" '"%cmd_terminal)
    rosBeginPort +=numPort
    gzBeginPort += numPort
    ParamStartID += numPort
    fileBeginId += numPort
    GPUId +=1