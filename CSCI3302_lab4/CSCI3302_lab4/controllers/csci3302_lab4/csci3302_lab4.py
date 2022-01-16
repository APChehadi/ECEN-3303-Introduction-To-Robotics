"""csci3302_lab4 controller."""
# Copyright (2021) Brad Hayes <bradley.hayes@colorado.edu>
# University of Colorado Boulder CSCI 3302: Introduction to Robotics

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
import random
import copy
#import numpy as np
from controller import Robot, Motor, DistanceSensor

state = "line_follower" # Change this to anything else to stay in place to test coordinate transform functions

LIDAR_SENSOR_MAX_RANGE = 3 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

# These are your pose values that you will update by solving the odometry equations
pose_x = 0.197
pose_y = 0.678
pose_theta = 0 

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28

# create the Robot instance.
robot=Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Initialize the Display    
display = robot.getDevice("display")

# get and enable lidar 
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()

##### DO NOT MODIFY ANY CODE ABOVE THIS #####

##### Part 1: Setup Data structures
#
# Create an empty list for your lidar sensor readings here,
# as well as an array that contains the angles of each ray 
# in radians. The total field of view is LIDAR_ANGLE_RANGE,
# and there are LIDAR_ANGLE_BINS. An easy way to generate the
# array that contains all the angles is to use linspace from
# the numpy package.

lidar_sensor_readings=[]
lidar_offsets=[0]*LIDAR_ANGLE_BINS
angle_step=LIDAR_ANGLE_RANGE/LIDAR_ANGLE_BINS
lidar_offsets[10]=0
for i in range(1,LIDAR_ANGLE_BINS-10):
    lidar_offsets[10+i]=lidar_offsets[10+i-1]+angle_step
    lidar_offsets[10-i]=lidar_offsets[10-i+1]-angle_step
    

#### End of Part 1 #####
 
# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:     
    
    #####################################################
    #                 Sensing                           #
    #####################################################

    # Read ground sensors
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read Lidar           
    lidar_sensor_readings = lidar.getRangeImage()
    
    
    ##### Part 2: Turn world coordinates into map coordinates
    #
    # Come up with a way to turn the robot pose (in world coordinates)
    # into coordinates on the map. Draw a red dot using display.drawPixel()
    # where the robot moves.
    
    #map robot
    x_map = int(pose_x * 300)
    y_map = int(pose_y * 300)
    # print("MAP", x_map, y_map)
    display.setColor(0xFF0000) #Red - Robot pose 
    display.drawPixel(x_map, y_map) #draw a pixel
    
    #From lab report:
    #The display has 300x300 pixels. Come up with a way to convert 
    #from world coordinates in meters to display coordinates in pixels so that the 300x300 display 
    #covers the entire 1m x 1m floor.  
    #Hint: You can use the (display.drawLine(int x1, int y1, int x2, y2)) to plot 
    #beams from the LIDAR 
    # Please use red for drawing the robot pose, blue for the obstacles detected by the 
    # LiDAR, and white for the free space. Pixels in unmapped areas should remain black.  
    #display.setColor(0xFFFFFF) #White - Free space 
    #display.setColor(0x0000FF) #Blue - Obstacle 
    
    # map the lidar white 
    # print(lidar_sensor_readings) #lengths of the lidar lines
    
    #use the length and the offset angle to find the end of the lidar reading
    lidar_map_x = 0
    lidar_map_y = 0 
    
    for i in range(0, len(lidar_sensor_readings)):
        #offest is angle and lidar_sensor_readins are h
        if(lidar_sensor_readings[i] == math.inf):
            lidar_sensor_readings[i] = 1
            
        print("Length lidar_sensor_readings: " + str(len(lidar_sensor_readings)))
        
        rx = math.cos(lidar_offsets[i])*lidar_sensor_readings[i] #point in robot frame of reference
        ry = math.sin(lidar_offsets[i])*lidar_sensor_readings[i]
        
        wx = math.sin(pose_theta)*rx - math.cos(pose_theta)*ry + pose_x
        wy = math.cos(pose_theta)*rx + math.sin(pose_theta)*ry + pose_y
        
        lidar_map_x = int(wx * 300)
        lidar_map_y = int(wy * 300)
        
        display.setColor(0xFFFFFF) #White - Free space
        display.drawLine(x_map, y_map, lidar_map_x, lidar_map_y)
        display.setColor(0x0000FF) #Blue - Obstacle 
        display.drawPixel(lidar_map_x, lidar_map_y) #draw a pixel

    display.setColor(0xFF0000) #Red - Robot pose 
    display.drawPixel(x_map, y_map) #draw a pixel
    
    ##### Part 3: Convert Lidar data into world coordinates
    #
    # Each Lidar reading has a distance rho and an angle alpha.
    # First compute the corresponding rx and ry of where the lidar
    # hits the object in the robot coordinate system. Then convert
    # rx and ry into world coordinates wx and wy. 
    # The arena is 1x1m2 and its origin is in the top left of the arena. 
    

    
    
    ##### Part 4: Draw the obstacle and free space pixels on the map
 
    
          

    
 

    
    # DO NOT MODIFY THE FOLLOWING CODE
    #####################################################
    #                 Robot controller                  #
    #####################################################

    if state == "line_follower":
            if(gsr[1]<350 and gsr[0]>400 and gsr[2] > 400):
                vL=MAX_SPEED*0.3
                vR=MAX_SPEED*0.3                
            # Checking for Start Line          
            elif(gsr[0]<500 and gsr[1]<500 and gsr[2]<500):
                vL=MAX_SPEED*0.3
                vR=MAX_SPEED*0.3
                # print("Over the line!") # Feel free to uncomment this
                display.imageSave(None,"map.png") 
            elif(gsr[2]<650): # turn right
                vL=0.2*MAX_SPEED
                vR=-0.05*MAX_SPEED
            elif(gsr[0]<650): # turn left
                vL=-0.05*MAX_SPEED
                vR=0.2*MAX_SPEED
             
    else:
        # Stationary State
        vL=0
        vR=0   
    
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
    
    #####################################################
    #                    Odometry                       #
    #####################################################
    
    EPUCK_MAX_WHEEL_SPEED = 0.11695*SIM_TIMESTEP/1000.0 
    dsr=vR/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    dsl=vL/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    ds=(dsr+dsl)/2.0
    
    pose_y += ds*math.cos(pose_theta)
    pose_x += ds*math.sin(pose_theta)
    pose_theta += (dsr-dsl)/EPUCK_AXLE_DIAMETER
    
    # Feel free to uncomment this for debugging
    print("X: %f Y: %f Theta: %f " % (pose_x,pose_y,pose_theta))