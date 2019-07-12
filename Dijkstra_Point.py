'''
#  Copyright 2019 Koyal Bhartia
#  @file    Dijkstra-POint Robot.py
#  @author  Koyal Bhartia
#  @version 1.0
#
#  @brief This is the code to solve the Dijkstra algorithm for a point robot
#
# @Description This code has functions which returns all the possible paths that can be
# traversed given any goal matrix. Given any input matrix it gives the path to the goal matrix
#
'''
import argparse
import numpy as np
import os, sys
from numpy import linalg as LA
from numpy import linalg as la
from matplotlib import pyplot as plt
import math
from PIL import Image
import random

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2

# Function to create map using half planes
def createMap():

    for x in range(0,image_width):
        for y in range(0,image_height):

            #Rectangle Obstacle
            # 4 half planes needed for the rectangle
            #V1---V2
            # -   -
            #V4---V3
            rectV1x,rectV1y=int((50-1)*resolution),int((112-1)*resolution)
            rectV2x,rectV2y=int((100-1)*resolution),int((112-1)*resolution)
            rectV3x,rectV3y=int((100-1)*resolution),int((67-1)*resolution)
            rectV4x,rectV4y=int((50-1)*resolution),int((67-1)*resolution)
            if x in range(rectV1x,rectV2x+1):
                if y in range(rectV4y,rectV1y+1):
                    white_image[image_height-1-y,x]=[0,0,0]

            #Elliptical Obstacle
            # 1 elliptical half plane needed for ellipse obstable
            Ellipx,Ellipy=int((140-1)*resolution),int((120-1)*resolution)
            EllipMaj=int(15*resolution)
            EllipMin=int(6*resolution)

            EllipObstacle=math.pow(x-Ellipx,2)/math.pow(EllipMaj,2)+math.pow(y-Ellipy,2)/math.pow(EllipMin,2)
            if EllipObstacle<=1:
                white_image[image_height-1-y,x]=[0,0,0]

            #Circular Obstacle
            # 1 circular half plane needed for circle obstable
            Circlex=int((190-1)*resolution)
            Circley=int((130-1)*resolution)
            CircleRad=int(15*resolution)

            CircleObstacle=math.pow(x-Circlex,2)+math.pow(y-Circley,2)
            if CircleObstacle<=math.pow(CircleRad,2):
                white_image[image_height-1-y,x]=[0,0,0]

        # 6 half planes needed for the polygon obstacle
            def calcIntercept(x,y,m):
                c=y-m*x
                return c
            PolyV0x,PolyV0y=int((150-1)*resolution),int((15-1)*resolution)
            PolyV1x,PolyV1y=int((125-1)*resolution),int((56-1)*resolution)
            PolyV2x,PolyV2y=int((163-1)*resolution),int((52-1)*resolution)
            PolyV3x,PolyV3y=int((170-1)*resolution),int((90-1)*resolution)
            PolyV5x,PolyV5y=int((173-1)*resolution),int((15-1)*resolution)
            PolyV4x,PolyV4y=int((193-1)*resolution),int((52-1)*resolution)

            Slope1=(PolyV1y-PolyV0y)/(PolyV1x-PolyV0x)
            Slope2=(PolyV2y-PolyV1y)/(PolyV2x-PolyV1x)
            Slope3=(PolyV3y-PolyV2y)/(PolyV3x-PolyV2x)
            Slope4=(PolyV4y-PolyV3y)/(PolyV4x-PolyV3x)
            Slope5=(PolyV5y-PolyV4y)/(PolyV5x-PolyV4x)
            Slope6=(PolyV0y-PolyV5y)/(PolyV0x-PolyV5x)

            Interc1=calcIntercept(PolyV1x,PolyV1y,Slope1)
            Interc2=calcIntercept(PolyV1x,PolyV1y,Slope2)
            Interc3=calcIntercept(PolyV3x,PolyV3y,Slope3)
            Interc4=calcIntercept(PolyV3x,PolyV3y,Slope4)
            Interc5=calcIntercept(PolyV5x,PolyV5y,Slope5)
            Interc6=calcIntercept(PolyV5x,PolyV5y,Slope6)

            ObsLine1x=(y-Interc1)/Slope1
            ObsLine2y=Slope2*x+Interc2
            ObsLine3x=(y-Interc3)/Slope3
            ObsLine4x=(y-Interc4)/Slope4
            ObsLine5x=(y-Interc5)/Slope5
            if(x>ObsLine1x and (y<ObsLine2y or x>ObsLine3x) and x<ObsLine4x and x<ObsLine5x and y>Interc6):
                white_image[image_height-1-y,x]=[0,0,0]

#Updating the initial values of the matrix conatining info of all ndoes
def allNodesInfoMatCreate():
    for x in range(0,image_height):
        for y in range(0,image_width):
            allNodesInfo[image_width*x+y,0]=x
            allNodesInfo[image_width*x+y,1]=y
            if white_image[x,y][0]+white_image[x,y][1]+white_image[x,y][2]==0:
                allNodesInfo[image_width*x+y,5]=np.inf
            else:
                allNodesInfo[image_width*x+y,5]=10000
    return allNodesInfo

#Fucntion to get the neighbours of any parent node
def fetchCoordinates(parentx,parenty):
    global LeftX,LeftY,RightX,RightY,UpX,UpY,DownX,DownY,UpLeftX,UpLeftY,UpRightX,UpRightY,DownLeftX,DownLeftY,DownRightX,DownRightY
    LeftX,LeftY=parentx,parenty-1
    RightX,RightY=parentx,parenty+1
    UpX,UpY=parentx-1,parenty
    DownX,DownY=parentx+1,parenty
    UpLeftX,UpLeftY=parentx-1,parenty-1
    UpRightX,UpRightY=parentx-1,parenty+1
    DownLeftX,DownLeftY=parentx+1,parenty-1
    DownRightX,DownRightY=parentx+1,parenty+1

#Function to update the matrix conataiing all node Info
#The node, cost and the parent of each node
def updatecostnPrevNode(allNodesInfo,parentx,parenty):
    img_array=[]
    count=0
    while(allNodesInfo[image_width*goalx+goaly,5]!=50000):
        fetchCoordinates(parentx,parenty)
        print("Parent:", parentx,parenty)
        #if count!=0 and parentx!=goalx and parenty!=goaly:
        white_image[parentx,parenty]=[0,255,255]

        if UpX>-1 and (white_image[UpX,UpY][0]+white_image[UpX,UpY][1]+white_image[UpX,UpY][2])!=0:
                newupcost=allNodesInfo[image_width*parentx+parenty,2]+costUp
                if newupcost<allNodesInfo[image_width*UpX+UpY,2]:
                    print(UpX,UpY,"up")
                    allNodesInfo[image_width*UpX+UpY,2]=newupcost
                    allNodesInfo[image_width*UpX+UpY,5]=newupcost
                    allNodesInfo[image_width*UpX+UpY,3]=parentx
                    allNodesInfo[image_width*UpX+UpY,4]=parenty
                    white_image[UpX,UpY]=[102,0,51]
            #print(allNodesInfo[image_width*UpX+UpY,5],"up")
        if DownX<image_height and (white_image[DownX,DownY][0]+white_image[DownX,DownY][1]+white_image[DownX,DownY][2])!=0:# and white_image[DownX,DownY][2]!=153:
                newdowncost=allNodesInfo[image_width*parentx+parenty,2]+costDown
                if newdowncost<allNodesInfo[image_width*DownX+DownY,2]:
                    print(DownX,DownY,"down")
                    print(white_image[UpX,UpY][2])
                    allNodesInfo[image_width*DownX+DownY,2]=newdowncost
                    allNodesInfo[image_width*DownX+DownY,5]=newdowncost
                    allNodesInfo[image_width*DownX+DownY,3]=parentx
                    allNodesInfo[image_width*DownX+DownY,4]=parenty
                    white_image[DownX,DownY]=[102,0,51]
            #print(allNodesInfo[image_width*DownX+DownY,5],"down")
        if LeftY>-1 and (white_image[LeftX,LeftY][0]+white_image[LeftX,LeftY][1]+white_image[LeftX,LeftY][2])!=0:# and white_image[LeftX,LeftY][2]!=153:
                newleftcost=allNodesInfo[image_width*parentx+parenty,2]+costLeft
                if newleftcost<allNodesInfo[image_width*LeftX+LeftY,2]:
                    print(LeftX,LeftY,"left")
                    print(white_image[UpX,UpY][2])
                    allNodesInfo[image_width*LeftX+LeftY,2]=newleftcost
                    allNodesInfo[image_width*LeftX+LeftY,5]=newleftcost
                    allNodesInfo[image_width*LeftX+LeftY,3]=parentx
                    allNodesInfo[image_width*LeftX+LeftY,4]=parenty
                    white_image[LeftX,LeftY]=[102,0,51]

        if RightY<image_width and (white_image[RightX,RightY][0]+white_image[RightX,RightY][1]+white_image[RightX,RightY][2])!=0:# and white_image[RightX,RightY][2]!=153:
                newrightcost=allNodesInfo[image_width*parentx+parenty,2]+costRight
                if newrightcost<allNodesInfo[image_width*RightX+RightY,2]:
                    print(RightX,RightY,"right")
                    allNodesInfo[image_width*RightX+RightY,2]=newrightcost
                    allNodesInfo[image_width*RightX+RightY,5]=newrightcost
                    allNodesInfo[image_width*RightX+RightY,3]=parentx
                    allNodesInfo[image_width*RightX+RightY,4]=parenty
                    print(allNodesInfo[image_width*RightX+RightY,5],"right")
                    white_image[RightX,RightY]=[102,0,51]

        if UpLeftX>-1 and UpLeftY>-1 and RightY<image_width and (white_image[UpLeftX,UpLeftY][0]+white_image[UpLeftX,UpLeftY][1]+white_image[UpLeftX,UpLeftY][2])!=0:# and white_image[UpLeftX,UpLeftY][2]!=153:
                newupleftcost=allNodesInfo[image_width*parentx+parenty,2]+costUpLeft
                if newupleftcost<allNodesInfo[image_width*UpLeftX+UpLeftY,2]:
                    print(UpLeftX,UpLeftY,"upleft")
                    allNodesInfo[image_width*UpLeftX+UpLeftY,2]=newupleftcost
                    allNodesInfo[image_width*UpLeftX+UpLeftY,5]=newupleftcost
                    allNodesInfo[image_width*UpLeftX+UpLeftY,3]=parentx
                    allNodesInfo[image_width*UpLeftX+UpLeftY,4]=parenty
                    white_image[UpLeftX,UpLeftY]=[102,0,51]

        if UpRightX>-1 and UpRightY<image_width and (white_image[UpRightX,UpRightY][0]+white_image[UpRightX,UpRightY][1]+white_image[UpRightX,UpRightY][2])!=0: #and white_image[UpRightX,UpRightY][2]!=153:
                newuprightcost=allNodesInfo[image_width*parentx+parenty,2]+costUpRight
                if newuprightcost<allNodesInfo[image_width*UpRightX+UpRightY,2]:
                    print(UpRightX,UpRightY,"upright")
                    allNodesInfo[image_width*UpRightX+UpRightY,2]=newuprightcost
                    allNodesInfo[image_width*UpRightX+UpRightY,5]=newuprightcost
                    allNodesInfo[image_width*UpRightX+UpRightY,3]=parentx
                    allNodesInfo[image_width*UpRightX+UpRightY,4]=parenty
                    white_image[UpRightX,UpRightY]=[102,0,51]

        if DownLeftY>-1 and DownLeftX<image_height and (white_image[DownLeftX,DownLeftY][0]+white_image[DownLeftX,DownLeftY][1]+white_image[DownLeftX,DownLeftY][2])!=0:# and white_image[DownLeftX,DownLeftY][2]!=153:
                newdownleftcost=allNodesInfo[image_width*parentx+parenty,2]+costDoLeft
                if newdownleftcost<allNodesInfo[image_width*DownLeftX+DownLeftY,2]:
                    print(DownLeftX,DownLeftY,"downleft")
                    allNodesInfo[image_width*DownLeftX+DownLeftY,2]=newdownleftcost
                    allNodesInfo[image_width*DownLeftX+DownLeftY,5]=newdownleftcost
                    allNodesInfo[image_width*DownLeftX+DownLeftY,3]=parentx
                    allNodesInfo[image_width*DownLeftX+DownLeftY,4]=parenty
                    white_image[DownLeftX,DownLeftY]=[102,0,51]

        if DownRightY<image_width and DownLeftX<image_height and (white_image[DownRightX,DownRightY][0]+white_image[DownRightX,DownRightY][1]+white_image[DownRightX,DownRightY][2])!=0:# and white_image[DownRightX,DownRightY][2]!=153:
                newdownrightcost=allNodesInfo[image_width*parentx+parenty,2]+costDoRight
                if newdownrightcost<allNodesInfo[image_width*DownRightX+DownRightY,2]:
                    print(DownRightX,DownRightY,"downright")
                    allNodesInfo[image_width*DownRightX+DownRightY,2]=newdownrightcost
                    allNodesInfo[image_width*DownRightX+DownRightY,5]=newdownrightcost
                    allNodesInfo[image_width*DownRightX+DownRightY,3]=parentx
                    allNodesInfo[image_width*DownRightX+DownRightY,4]=parenty
                    white_image[DownRightX,DownRightY]=[102,0,51]

        allNodesInfo[image_width*parentx+parenty,5]=50000
        #white_image[parentx,parenty]=[102,0,51]
        white_image1=white_image.copy()


        minIndex=allNodesInfo[:,5].argmin()
        parentx=int(allNodesInfo[minIndex,0])
        parenty=int(allNodesInfo[minIndex,1])
        img_array.append(white_image1)
        count+=1
    return allNodesInfo,img_array

def NodePath(img_array,allNodesInfo,startx,starty,goalx,goaly):
    startindex=image_width*startx+starty
    goalindex=image_width*goalx+goaly
    totcost=allNodesInfo[goalindex,2]
    print("Total cost from start to goal node:",totcost)
    position=goalindex
    node_path=[]
    node_path.append(position)
    path_array=[]
    while(position!=startindex):
        #totcost=totcost+allNodesInfo[position,2]
        positionx=int(allNodesInfo[position,3])
        positiony=int(allNodesInfo[position,4])
        position=image_width*positionx+positiony
        node_path.append(position)
    node_path.reverse()
        #node_path.append(node_position)
    for i in range(len(node_path)):
        position=node_path[i]
        positionx=int(allNodesInfo[position,0])
        positiony=int(allNodesInfo[position,1])
        white_image[positionx,positiony]=[153,0,0]
        white_image1=white_image.copy()
        img_array.append(white_image1)

def video(img_array):
    video=cv2.VideoWriter('Dijkstra_Point.avi',cv2.VideoWriter_fourcc(*'DIVX'), 38,(image_width,image_height))
    for i in range(len(img_array)):
        video.write(img_array[i])
    video.release()


if __name__ == '__main__':

    #user inputs - default
    resolution=1
    RobotDiameter=0
    startx=0
    starty=0
    goalx=149
    goaly=249
    Clearance=0
    #--------------------------------------------------------------------
    #Taking user inputs
    print("Dijkstra Algorithm - Point Robot")
    print("Assume the follwoing axis")
    print("*--->y")
    print("`")
    print("`")
    print("x")
    flag=0
    while(flag!=1):
        resolution=input("Enter expected image resolution")
        print(resolution)
        try:
            resolution=float(resolution)
            if resolution>0:
                flag=1
                break
            else:
                print("Please enter a postive value for resolution")
        except ValueError:
            print("Please enter valid resolution")
    #----------------------------------------------------------------------
    #Create Map
    print("Creating map of above resolution")
    resolution=np.round(1/resolution,1)
    image_height=int(150*resolution)
    image_width=int(250*resolution)
    white_image=255*np.ones((image_height,image_width,3),dtype=np.uint8)
    createMap()
    print("Map created")
    cv2.namedWindow('Map_Created',cv2.WINDOW_NORMAL)
    cv2.imshow("Map_Created",white_image)
    print("Please press 0 to close the image")
    cv2.waitKey(0)
    #--------------------------------------------------------------------
    #Other user inputs - start
    flag=0
    while(flag!=1):
        print("Please enter the start coordinates in the range 0-149 for X and 0-249 for Y")
        print("It will be scaled according to the resolution accordingly")
        startx=input("Start node, X coordinate:")
        starty=input("Start node, Y coordinate:")
        try:
            startx=int(startx)
            starty=int(starty)
            if(startx in range(0,150) and starty in range(0,250)):
                startx=int(startx*resolution)
                starty=int(starty*resolution)
                if(white_image[startx,starty][0]+white_image[startx,starty][1]+white_image[startx,starty][2]==0):
                    raise ValueError
            else:
                raise ValueError
            flag=1
        except ValueError:
            print("Please enter valid coordinates which do not lie on obstacle")
    #-------------------------------------------------------------------------
    #Other user inputs - goal
    flag=0
    while(flag!=1):
        print("Similarly enter the goal node of the robot:")
        goalx=input("Goal node of the robot, X coordinate:")
        goaly=input("Goal node of the robot, Y coordinate:")
        try:
            goalx=int(goalx)
            goaly=int(goaly)
            if(goalx in range(0,150) and goaly in range(0,250)):
                goalx=int(goalx*resolution)
                goaly=int(goaly*resolution)
                if(white_image[startx,starty][0]+white_image[startx,starty][1]+white_image[startx,starty][2]==0):
                    raise ValueError
            else:
                raise ValueError
            flag=1
        except ValueError:
            print("Please enter valid coordinates which do not lie on obstacle")
    #-------------------------------------------------------------------------
    # Dijkstra algorithm initializations
    parentx=startx
    parenty=starty
    white_image[parentx,parenty]=[0,100,0]
    white_image[goalx,goaly]=[0,0,255]
    Total=image_height*image_width
    allNodesInfo=np.zeros((Total,6)) #Creating matrix of nodes to update cost and previous nodes
    allNodesInfo[:,2]=np.inf
    allNodesInfo[image_width*startx+starty,2]=0 # Initializing G-cost of start node as 0
    costUp,costDown,costLeft,costRight=1,1,1,1
    costUpLeft,costDoLeft,costUpRight,costDoRight=np.sqrt(2),np.sqrt(2),np.sqrt(2),np.sqrt(2)
    #-------------------------------------------------------------------
    # The Dikhstra algorithm
    allNodesInfo=allNodesInfoMatCreate() # Exploration of nodes
    allNodesInfo,img_array=updatecostnPrevNode(allNodesInfo,parentx,parenty)
    NodePath(img_array,allNodesInfo,startx,starty,goalx,goaly) #Backtrace to get the optimum path
    video(img_array)
