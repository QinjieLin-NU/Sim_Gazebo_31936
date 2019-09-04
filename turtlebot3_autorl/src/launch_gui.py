#!/usr/bin/env python
from tkinter import *
from tkinter import ttk
import sys
import os
from tkinter import messagebox
import tkinter as tk
from plot_result import plotData
from launch_model import test_model
from util_io import launch_stageWorlds
import glob
class LaunchGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Training Management")
        self.mainframe = ttk.Frame(master, padding="3 3 12 12")
        self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)
        # self.create_label(1,1,"RosPort")
        # self.rosPort = self.create_text(2,1)
        # self.create_button(3,2,"launch",self.launch_termial)
        self.create_label(1,1,"Train")
        endR,endC = self.create_launch(2,2)
        self.create_label(endR+1,1,"Inspect")
        endR,endC = self.create_inspect(endR+2,2)
        self.create_label(endR+1,1,"Test")
        endR,endC = self.create_test(endR+2,2)

        for child in self.mainframe.winfo_children(): child.grid_configure(padx=5, pady=5)

    def create_label(self,row,col,text):
        ttk.Label(self.mainframe, text=text).grid(column=col, row=row, sticky=W)

    def create_text(self,row,col,tmp):
        tmp_entry = ttk.Entry(self.mainframe, width=25, textvariable=tmp)
        tmp_entry.grid(row=row, column=col, sticky=(W, E))

    def create_button(self,row,col,text,func):
        ttk.Button(self.mainframe, text=text, command=func).grid(column=col, row=row, sticky=W)

    def launch_termial(self):
        rosBeginPort = int(self.rosPort.get())
        gzBeginPort = int(self.gazeboPort.get())
        numPort = int(self.numPort.get())
        numGpu = int(self.gpuNum.get())
        ParamStartID = int(self.ParamBeginId.get())
        fileBeginId = int(self.fileBeginId.get())
        GPUId = int(self.GpuId.get())
        continueRL = False
        gzfileName = str(self.envFile.get()) #"turtlebot3_house_envRL.launch"#"turtlebot3_stage_2_envRL.launch"#"turtlebot3_house_envRL.launch"
        rlfileName = str(self.RLFile.get()) #"turtlebot3_PPO_v2.launch"
        StageId = int(self.stageId.get())
        simName = "gazebo"
        dirName = self.gazebomapDir.get()
        worldDirName = os.path.dirname(os.path.abspath(__file__))+"/../worlds/%s"%dirName
        numWorld = len(glob.glob("%s/*.world"%worldDirName))
        gui = self.gui.get()
        print("************* numWorld:%d ***************"%numWorld)
        for i in range(numGpu):
            if(continueRL):
                cmd_terminal = "python turtlebot3_continueRL.py %d %d %d %d %d %d"%(rosBeginPort,gzBeginPort,numPort,ParamStartID,fileBeginId,GPUId)
            else:
                cmd_terminal = "python turtlebot3_autoRL_v2.py %d %d %d %d %d %d %s %s %d %s %s %s"%(rosBeginPort,gzBeginPort,numPort,ParamStartID,fileBeginId,GPUId,gzfileName,rlfileName,StageId,simName,dirName,gui)
            print(cmd_terminal) 
            os.system("gnome-terminal -e 'bash -c \"%s; exec bash\" '"%cmd_terminal)
            rosBeginPort +=numPort * numWorld
            gzBeginPort += numPort * numWorld
            ParamStartID += numPort
            fileBeginId += numPort * numWorld
            GPUId +=1
        self.rosPort.set(str(rosBeginPort))
        self.gazeboPort.set(str(gzBeginPort))

    def create_launch(self,row,col):
        deltaC =col 
        deltaR = row 
        space = -1 
        self.create_label(deltaR,deltaC,"Gazebo Setting:")

        self.envFile = StringVar(value="turtlebot3_office_envRL.launch")
        self.create_label(deltaR+1+space,deltaC+1,"env name:")
        self.create_text(deltaR+1+space,deltaC+2,self.envFile)

        self.create_label(deltaR+1+space,deltaC+3,"  ")

        self.gazebomapDir = StringVar(value="gazebo_world")
        self.create_label(deltaR+1+space,deltaC+4,"world_dataset:")
        self.create_text(deltaR+1+space,deltaC+5,self.gazebomapDir)

        self.create_label(deltaR+1+space,deltaC+6,"  ")

        self.gazeboPort = StringVar(value="11856")
        self.create_label(deltaR+1+space,deltaC+7,"gazebo_port:")
        self.create_text(deltaR+1+space,deltaC+8,self.gazeboPort)

        self.create_label(deltaR+2,deltaC,"Stage Setting:")

        self.stageEnvFile = StringVar(value="stage_world.world")
        self.create_label(deltaR+3+space,deltaC+1,"env_name:")
        self.create_text(deltaR+3+space,deltaC+2,self.stageEnvFile)

        self.create_label(deltaR+3+space,deltaC+3,"  ")

        self.stagemapDir = StringVar(value="stage_map")
        self.create_label(deltaR+3+space,deltaC+4,"world_dataset:")
        self.create_text(deltaR+3+space,deltaC+5,self.stagemapDir)


        self.create_label(deltaR+4,deltaC,"RL Setting:")

        self.RLFile = StringVar(value="turtlebot3_PPO_v2.launch")
        self.create_label(deltaR+5+space,deltaC+1,"rl_algorithm:")
        self.create_text(deltaR+5+space,deltaC+2,self.RLFile)

        self.create_label(deltaR+5+space,deltaC+3,"  ")

        self.stageId = StringVar(value="5")
        self.create_label(deltaR+5+space,deltaC+4,"stage_id:")
        self.create_text(deltaR+5+space,deltaC+5,self.stageId)

        self.create_label(deltaR+5+space,deltaC+6,"  ")

        self.fileBeginId = StringVar(value="10000")
        self.create_label(deltaR+5+space,deltaC+7,"saved_fielBeiginId:")
        self.create_text(deltaR+5+space,deltaC+8,self.fileBeginId)


        self.create_label(deltaR+6,deltaC,"ROS Setting:")

        self.rosPort = StringVar(value="11516")
        self.create_label(deltaR+7+space,deltaC+1,"ros_beginPort:")
        self.create_text(deltaR+7+space,deltaC+2,self.rosPort)

        self.create_label(deltaR+8,deltaC,"GPU Setting:")

        self.GpuId = StringVar(value="0")
        self.create_label(deltaR+9+space,deltaC+1,"GPU_beginId:")
        self.create_text(deltaR+9+space,deltaC+2,self.GpuId)

        self.create_label(deltaR+9+space,deltaC+3,"  ")

        self.gpuNum = StringVar(value="4")
        self.create_label(deltaR+9+space,deltaC+4,"GPU_num:")
        self.create_text(deltaR+9+space,deltaC+5,self.gpuNum)

        self.create_label(deltaR+9+space,deltaC+6,"  ")

        self.numPort = StringVar(value="0")
        self.create_label(deltaR+9+space,deltaC+7,"num_param/GPU:")
        self.create_text(deltaR+9+space,deltaC+8,self.numPort)

        self.ParamBeginId = StringVar(value="0")

        self.create_label(deltaR+9,deltaC,"GUI Setting:")

        self.gui = StringVar(value="false")
        self.create_label(deltaR+10+space,deltaC+1,"gui:")
        self.create_text(deltaR+10+space,deltaC+2,self.gui)

        # self.create_label(deltaR+4,deltaC+2,"ParamBeginId:")
        # self.create_text(deltaR+4,deltaC+3,self.ParamBeginId)


        # self.rosPort = StringVar(value="11516")
        # self.create_label(row+1,col+1,"RosPort:")
        # self.create_text(row+1,col+2,self.rosPort)

        # self.create_label(row+1,col+3,"  ")

        # self.gazeboPort = StringVar(value="11856")
        # self.create_label(row+1,col+4,"GazeboPort:")
        # self.create_text(row+1,col+5,self.gazeboPort)

        # self.create_label(row+1,col+6,"  ")

        # self.stagemapDir = StringVar(value="stage_map")
        # self.create_label(row+1,col+7,"WorldDirectory:")
        # self.create_text(row+1,col+8,self.stagemapDir)

        # self.GpuId = StringVar(value="0")
        # self.create_label(deltaR+3,deltaC+2,"GPUBeginId:")
        # self.create_text(deltaR+3,deltaC+3,self.GpuId)

        # self.create_label(deltaR+3,deltaC+4,"  ")

        # self.gpuNum = StringVar(value="4")
        # self.create_label(deltaR+3,deltaC+5,"GpuNum:")
        # self.create_text(deltaR+3,deltaC+6,self.gpuNum)

        # self.create_label(deltaR+3,deltaC+7,"  ")

        # self.numPort = StringVar(value="0")
        # self.create_label(deltaR+3,deltaC+8,"numParam:")
        # self.create_text(deltaR+3,deltaC+9,self.numPort)

        # self.ParamBeginId = StringVar(value="0")
        # self.create_label(deltaR+4,deltaC+2,"ParamBeginId:")
        # self.create_text(deltaR+4,deltaC+3,self.ParamBeginId)

        # self.create_label(deltaR+4,deltaC+4,"  ")

        # self.fileBeginId = StringVar(value="10000")
        # self.create_label(deltaR+4,deltaC+5,"fileBeginId:")
        # self.create_text(deltaR+4,deltaC+6,self.fileBeginId)

        # self.create_label(deltaR+5,deltaC+1,"File Setting:")

        # self.envFile = StringVar(value="turtlebot3_office_envRL.launch")
        # self.create_label(deltaR+6,deltaC+2,"env name:")
        # self.create_text(deltaR+6,deltaC+3,self.envFile)

        # self.create_label(deltaR+6,deltaC+4,"  ")

        # self.RLFile = StringVar(value="turtlebot3_PPO_v2.launch")
        # self.create_label(deltaR+6,deltaC+5,"rl name:")
        # self.create_text(deltaR+6,deltaC+6,self.RLFile)

        # self.create_label(deltaR+6,deltaC+7,"  ")

        # self.stageId = StringVar(value="5")
        # self.create_label(deltaR+6,deltaC+8,"stageId:")
        # self.create_text(deltaR+6,deltaC+9,self.stageId)

        self.create_button(deltaR+11+space,deltaC+2,"launch gazebo",self.launchGz_confirm)

        self.create_button(deltaR+11+space,deltaC+5,"launch stage",self.launchStage_confirm)


        return deltaR+11,deltaC+5

    def launchGz_confirm(self):
        MsgBox = tk.messagebox.askquestion('launch termial', 'Are you sure?',
                                            icon='warning')
        if MsgBox == 'yes':
            self.launch_termial()
        else:
            # tk.messagebox.showinfo('Return', 'You will now return')
            return

    def launchStage_confirm(self):
        MsgBox = tk.messagebox.askquestion('launch termial', 'Are you sure?',
                                            icon='warning')
        if MsgBox == 'yes':
            self.launch_stage()
        else:
            # tk.messagebox.showinfo('Return', 'You will now return')
            return
    
    def launch_stage(self):
        # launch_stageWorlds(dirName=dirName,gui="true")
        rosBeginPort = int(self.rosPort.get())
        gzBeginPort = int(self.gazeboPort.get())
        numParam = int(self.numPort.get())
        numGpu = int(self.gpuNum.get())
        ParamStartID = 0#int(self.ParamBeginId.get())
        fileBeginId = int(self.fileBeginId.get())
        GPUId = int(self.GpuId.get())
        continueRL = False
        gzfileName = str(self.envFile.get()) #"turtlebot3_house_envRL.launch"#"turtlebot3_stage_2_envRL.launch"#"turtlebot3_house_envRL.launch"
        rlfileName = str(self.RLFile.get()) #"turtlebot3_PPO_v2.launch"
        StageId = int(self.stageId.get())
        simName = "stage"
        dirName = self.stagemapDir.get()
        worldDirName = os.path.dirname(os.path.abspath(__file__))+"/../worlds/%s"%dirName
        numWorld = len(glob.glob("%s/*.png"%worldDirName))
        gui = self.gui.get()
        for i in range(numGpu):
            if(continueRL):
                cmd_terminal = "python turtlebot3_continueRL.py %d %d %d %d %d %d"%(rosBeginPort,gzBeginPort,numParam,ParamStartID,fileBeginId,GPUId)
            else:
                cmd_terminal = "python turtlebot3_autoRL_v2.py %d %d %d %d %d %d %s %s %d %s %s %s"%(rosBeginPort,gzBeginPort,numParam,ParamStartID,fileBeginId,GPUId,gzfileName,rlfileName,StageId,simName,dirName,gui)
            print(cmd_terminal) 
            os.system("gnome-terminal -e 'bash -c \"%s; exec bash\" '"%cmd_terminal)
            rosBeginPort +=numParam * numWorld
            gzBeginPort += numParam * numWorld
            ParamStartID += numParam
            fileBeginId += numParam * numWorld
            GPUId +=1
        self.rosPort.set(str(rosBeginPort))

    
    def create_inspect(self,row,col):
        self.create_label(row,col,"Plot Reward:")

        self.plotFileLoc = StringVar(value="saved_model_ddpg")
        self.create_label(row+1,col+1,"model_location:")
        self.create_text(row+1,col+2,self.plotFileLoc)

        self.plotBeginId = StringVar(value="0")
        self.create_label(row+2,col+1,"model_beginId:")
        self.create_text(row+2,col+2,self.plotBeginId)

        self.create_label(row+2,col+3,"  ")

        self.plotEndId = StringVar(value="100")
        self.create_label(row+2,col+4,"model_endId:")
        self.create_text(row+2,col+5,self.plotEndId)

        self.create_label(row+2,col+6,"  ")

        self.successThredshold = StringVar(value="0")
        self.create_label(row+3,col+1,"success_thredshold:")
        self.create_text(row+3,col+2,self.successThredshold)

        self.create_label(row+3,col+3,"  ")

        self.rewardThredshold = StringVar(value="0")
        self.create_label(row+3,col+4,"reward_thshold:")
        self.create_text(row+3,col+5,self.rewardThredshold)

        self.create_button(row+4,col+4,"plot",self.plot_data)

        self.create_label(row+5,col,"Inspect Training:")

        self.inspectRosPort = StringVar(value="11311")
        self.create_label(row+6,col+1,"ros_port:")
        self.create_text(row+6,col+2,self.inspectRosPort)

        self.create_label(row+6,col+3,"  ")

        self.inspectGazeboPort = StringVar(value="11345")
        self.create_label(row+6,col+4,"gazebo_port:")
        self.create_text(row+6,col+5,self.inspectGazeboPort)

        self.create_button(row+7,col+4,"inspect",self.inspect_gazebo)


        return (row+7,col+4)

    def test(self):
        print("successfully")

    def plot_data(self):
        beginId = int(self.plotBeginId.get())
        endId = int(self.plotEndId.get())
        fileLoc =str(self.plotFileLoc.get())
        rT = int(self.rewardThredshold.get())
        sT = int(self.successThredshold.get())
        plotData(beginId,endId,fileLoc,rT,sT)
    
    def inspect_gazebo(self):
        rosURI = int(self.inspectRosPort.get())
        gzURI = int(self.inspectGazeboPort.get())
        cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosURI)
        cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gzURI)
        cmd_launchWorld = "roslaunch gazebo_ros empty_world.launch"
        cmd_bash = "exec bash"
        cmd = cmd_rosURI+";"+cmd_gazeboURI+";"+cmd_launchWorld+";"+cmd_bash 
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd)

    def create_test(self,row,col):
        self.create_label(row,col,"launch_model")

        self.testFileLoc = StringVar(value = "saved_model_ddpg")
        self.create_label(row+1,col+1,"fileLoc:")
        self.create_text(row+1,col+2,self.testFileLoc)

        self.create_label(row+1,col+3,"  ")

        self.testModelId = StringVar(value = "10000")
        self.create_label(row+1,col+4,"model_id:")
        self.create_text(row+1,col+5,self.testModelId)

        self.create_label(row+1,col+6,"  ")

        self.testEpId = StringVar(value = "0")
        self.create_label(row+1,col+7,"laod_episode:")
        self.create_text(row+1,col+8,self.testEpId)

        self.testEnv =StringVar(value = "turtlebot3_office_envRL.launch")
        self.create_label(row+2,col+1,"env_file:")
        self.create_text(row+2,col+2,self.testEnv)

        self.create_label(row+2,col+3,"  ")

        self.testStageId = StringVar(value = "5")
        self.create_label(row+2,col+4,"stage_id:")
        self.create_text(row+2,col+5,self.testStageId)

        self.create_label(row+2,col+6,"  ")

        self.testRL = StringVar(value = "turtlebot3_DDPG_v1.launch")
        self.create_label(row+2,col+7,"rl_file:")
        self.create_text(row+2,col+8,self.testRL)

        self.testRosPort = StringVar(value="11311")
        self.create_label(row+3,col+1,"ros_port:")
        self.create_text(row+3,col+2,self.testRosPort)

        self.create_label(row+3,col+3,"  ")

        self.testGazeboPort = StringVar(value="11345")
        self.create_label(row+3,col+4,"gazebo_post:")
        self.create_text(row+3,col+5,self.testGazeboPort)

        self.create_label(row+3,col+6,"  ")

        self.realWorldFlag = StringVar(value="false")
        self.create_label(row+3,col+7,"real_world:")
        self.create_text(row+3,col+8,self.realWorldFlag)

        self.create_button(row+4,col+4,"test",self.launch_testModel)

        return(row+4,col+4)

    def launch_testModel(self):
        rosURI = int(self.testRosPort.get())
        gzURI = int(self.testGazeboPort.get())
        fileLoc = str(self.testFileLoc.get())
        fileId = int (self.testModelId.get())
        loadEp = int (self.testEpId.get())
        gzfileName = str(self.testEnv.get())
        stageId = int (self.testStageId.get())
        rlfileName = str(self.testRL.get())
        realWorldFlag = str(self.realWorldFlag.get())
        test_model(rosURI,gzURI,fileLoc,fileId,loadEp,gzfileName,stageId,rlfileName,realWorldFlag)


if __name__ == '__main__':
    root = Tk()
    LaunchGUI(root)
    root.mainloop()