#!/usr/bin/env python
import os
import time
import sys
import matplotlib.pyplot as plt

def savePng(id):
    plt.title("model %s"%id)
    plt.xlabel('Episode')  
    plt.ylabel('Moving averaged episode reward')
    filename = os.path.dirname(os.path.abspath(__file__))+"/../../../../data_ws/plot_autorl1/"+"%d.png"%id
    plt.savefig(filename)

def readData(dirName):
    rList = []
    sList=[]
    model = ""
    ignore = True
    dirPath = os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/"+dirName
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
                    r = int(line[3])
                    s = int(line[6])
                    rList.append(r)
                    sList.append(s)
    return rList,sList,model

def plotData(beginId,endId,fileLoc,reward_thred,succesTime_thred):
    idList = range(beginId,endId)
    successTimeList = []
    for id in idList:
        rList,sList,model = readData(fileLoc+"/autoRL_%d"%id)
        if(len(sList)>0):
            successTimeList.append(sList[-1])
            print("%d: %d"%(id,sList[-1]))
        if( (len(rList)>0) and (len(sList)>0) ):
            if(sList[-1]>succesTime_thred):
                if(rList[-1]>reward_thred):
                    plt.plot(rList,label=id)
                    print("model_%d"%id,model)
    plt.xlabel('Episode')  
    plt.ylabel('Moving averaged episode reward')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    idList = range(1518,1614)
    fileLoc = "saved_model_lin"
    successTimeList = []
    for id in idList:
        rList,sList,model = readData(fileLoc+"/autoRL_%d"%id)
        if(len(sList)>0):
            successTimeList.append(sList[-1])
            print("%d: %d"%(id,sList[-1]))
        if(len(rList)>0):
            if sList[-1]>10:
                if(rList[-1]!=0):
                    plt.plot(rList,label=id)
                    print("model_%d"%id,model)
    plt.xlabel('Episode')  
    plt.ylabel('Moving averaged episode reward')
    plt.legend()
    plt.show()
