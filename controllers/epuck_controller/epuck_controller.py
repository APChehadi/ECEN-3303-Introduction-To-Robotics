"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
from matplotlib import colors as colors

LIDAR_SENSOR_MAX_RANGE = 3.5 # Meters
LIDAR_ANGLE_BINS = 360 
LIDAR_ANGLE_RANGE = math.radians(360)
 
# These are your pose values that you will update by solving the odometry equations
pose_x = 0 
pose_y = 0  
pose_theta = 0

vL = 0
vR = 0

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize the Display
# We are using x*y, 385*645 pixels to map the 7.7m*12.86m break room    
display = robot.getDevice("display")

# get and enable lidar 
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()

#initialize distance sensors
ps = []
psNames = [
    'ds0', 'ds1', 'ds2'
]

for i in range(len(psNames)):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(SIM_TIMESTEP)

# Create an empty list for your lidar sensor readings here,
# as well as an array that contains the angles of each ray 
# in radians. The total field of view is LIDAR_ANGLE_RANGE,
# and there are LIDAR_ANGLE_BINS. An easy way to generate the
# array that contains all the angles is to use linspace from
# the numpy package.
lidar_sensor_readings=[]
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)


# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(SIM_TIMESTEP)

map = None

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
# mode = 'manual' 
# mode = 'planner' 
mode = 'autonomous'



###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = (0, 0) # (Pose_X, Pose_Z) in meters
    end_w = (-3.2, -4.4) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = (int(250 + (start_w[0]*50)),int(335 + (start_w[1]*50))) # (x, y) 
    end = (int(250 + (end_w[0]*50)), int(335 + (end_w[1]*50))) # (x, y) 

    # print(start_w, end_w)
    print(start, end)

    class Node:
        """
        Node for A* Algorithm. 
        """
        def __init__(self, point, parent=None):
            self.point = point # n-Dimensional point
            self.parent = parent # Parent node
            
            self.g = 0
            self.h = 0
            self.f = 0
    
    # Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    def path_planner(map, start, end):
        
        '''
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''

        start_node = Node(start, None)
        start_node.g = 0 #initialize to 0
        start_node.h = 0
        start_node.f = 0
        end_node = Node(end, None)
        end_node.g = 0
        end_node.h = 0
        end_node.f = 0
        open_list=[]
        open_list.append(start_node)
        closed_list=[]
        
        point2=np.array(end_node.point) #variable to calcualte distance from end_node
        
        while(len(open_list)>0):
            if (len(open_list)>1000000): #if length becomes too long, path cannot be found
                print('No possible path')
                return 0
                
            current_node = open_list[0]
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
            
            open_list.remove(current_node) 
            closed_list.append(current_node)
            
            point1=np.array(current_node.point) 

            buffer = 10
            if np.linalg.norm(point1-point2)<buffer:
                path = []
                curr = current_node
                while curr is not None:
                    path.append(curr.point)
                    curr = curr.parent
                return path[::-1] # Return reversed path
                
            children=[]
            
            for new_position in [(0, -buffer), (0, buffer), (-buffer, 0), (buffer, 0), (-buffer, -buffer), (-buffer, buffer), (buffer, -buffer), (buffer, buffer)]:    
                node_position = (current_node.point[0] + new_position[0], current_node.point[1] + new_position[1])
                
                if node_position[0] > (len(map) - 1) or node_position[0] < 0 or node_position[1] > (len(map[len(map)-1]) -1) or node_position[1] < 0:
                    continue
                if map[int(node_position[0])][int(node_position[1])] == 1: #1
                    continue
                new_node = Node(node_position, current_node)
                children.append(new_node)
            
            for child in children:
                # if child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue
                
                child.g = current_node.g + 1 #calcualte g, h, and f values
                child.h=((child.point[0] - end_node.point[0]) ** 2) + ((child.point[1] - end_node.point[1]) ** 2) #Euclidean Distance
                child.f = child.g + child.h
                
                # If child is already in the open list
                for open_child in open_list:
                    if child == open_child and child.g > open_child.g: #If g of child is greater than that of in the open list
                        continue #next child

                # Add the child to the open list
                open_list.append(child)

    # Part 2.1: Load map (map.npy) from disk and visualize it
    map = np.load("map.npy")
    plt.imshow(np.fliplr(map))  
    plt.imshow(map) 
    #plt.show() 


    # Part 2.2: Compute an approximation of the “configuration space”
    new_map=map.copy()
    for _ in range(5):
        for i in range(1, 384):
            for j in range(1, 644):
                if (map[i][j]==1):
                    new_map[i-1][j-1]=1
                    new_map[i-1][j-1]=1
                    new_map[i+1][j-1]=1
                    new_map[i-1][j]=1
                    new_map[i+1][j]=1
                    new_map[i-1][j+1]=1
                    new_map[i][j+1]=1
                    new_map[i+1][j+1]=1
                    new_map[i][j]=1
        map=new_map.copy()
    plt.imshow(np.fliplr(new_map))  
    plt.imshow(new_map)
    #plt.show()
    map=new_map
            
    

    # Part 2.3 continuation: Call path_planner
    path=path_planner(map, start, end)
    # print("path", path)

    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it            
    waypoints = [] # in world space
    for (x, y) in path:
        waypoints.append(((x-250)/50, (y-335)/50))
        map[x][y] = 1
    print("waypoints", waypoints)
    np.save("path.npy", waypoints) 
    plt.imshow(map)
    # plt.show()

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization
# Initialize your map data structure here as a 2D floating point array
map = np.zeros((385,645)) # Replace None by a numpy 2D floating point array
waypoints = []

if mode == 'autonomous':
    # Load path from disk and visualize it
    waypoints = np.load("path.npy")
    print("Waypoints loaded")
    print("First Waypoint: " + str(waypoints[0]))
    print("First Waypoint: " + str(waypoints[-1]))
    print("Waypoints", waypoints)
    print("End Waypoint State", len(waypoints) - 1)

state = 0 # use this to iterate through your path

while robot.step(SIM_TIMESTEP) != -1 and mode != 'planner':
    psValues = []
    for i in range(len(ps)):
        psValues.append(ps[i].getValue())

    ###################
    #
    # Mapping
    #
    ###################

    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()

    # Draw the robot's current pose on the 360x360 display
    x_map_puck = int(335 + (pose_y*50))
    y_map_puck = int(250 + (pose_x*50))
    display.setColor(int(0xFF0000)) #red
    display.drawPixel(y_map_puck, x_map_puck) #display (1,1) upper left hand corner

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho
        
        # Convert detection from robot coordinates into world coordinates
        wx = (math.cos(pose_theta)*rx - math.sin(pose_theta)*ry) # + pose_x
        wy = -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) # + pose_y


        if rho < LIDAR_SENSOR_MAX_RANGE:
            x_map = int(x_map_puck - (wx *50)) 
            y_map = int(y_map_puck + (wy *50))

            if(x_map >= 645 or x_map<0 or y_map >= 385 or y_map < 0):
                continue

            g = map[y_map, x_map]
            if g>1:
                g = 1
                
            color = (g*256**2+g*256+g)*255
            color = int(color)
            display.setColor( color ) 
            
            display.drawPixel(y_map, x_map)
            
            # Add to map
            map[y_map, x_map] += 0.005


    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED*0.05
            vR = MAX_SPEED*0.2
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED*0.2
            vR = -MAX_SPEED*0.05
        elif key == keyboard.UP:
            vL = MAX_SPEED #*0.8
            vR = MAX_SPEED #*0.8
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED #*0.8
            vR = -MAX_SPEED #*0.8
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            display.imageSave(None,"map.png")
            for i in range(0, 360):
                for j in range(0, 360):
                    if( map[i, j]  < 0.5):
                        map[i, j] = 0

            save_map = map > 0.5
            save_map = save_map*1
            np.save("map.npy", save_map)
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
        # Feedback controller
        # Calculate the error
        # Euclid Error
        rho = (((pose_x-waypoints[state][0])**2 + (pose_y-waypoints[state][1])**2)**(0.5))
        print("rho", rho)

        # Bearing Error
        if(state < 23):
            alpha = -(math.atan2(waypoints[state][1] - pose_y, waypoints[state][0] - pose_x*2) + pose_theta)
        else:
            alpha = -(math.atan2(waypoints[state][1] - pose_y, waypoints[state][0] - pose_x) + pose_theta)
        print("alpha", alpha)

        # Controller

        # x dot
        dX = 1*rho
        if (rho > 0.8):
            dTheta = -1*alpha
            

        # theta dot
        dTheta = 10*alpha #10
        print("dTheta", dTheta)

        # Compute wheelspeeds
        vL = (2*dX - dTheta * EPUCK_AXLE_DIAMETER) / 2
        vR = (2*dX + dTheta * EPUCK_AXLE_DIAMETER) / 2 

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
        max_val = max(abs(vL), abs(vR), MAX_SPEED)
        vL += max_val
        vR += max_val
        
        pctL = (vL / max(vL, vR) - 0.5) * 2
        pctR = (vR / max(vL, vR) - 0.5) * 2
        
        vL = pctL
        vR = pctR

        # Clamp wheel speeds
        if (vR > MAX_SPEED):
            vR = MAX_SPEED
        if (vR < -1*MAX_SPEED):
            vR = -1*MAX_SPEED
        if (vL > MAX_SPEED):
            vL = MAX_SPEED
        if (vL < -1*MAX_SPEED):
            vL = -1*MAX_SPEED

        #Changing Waypoints 
        if ( abs(rho) < 0.5 and state < len(waypoints)-2): 
            state = state+1
        elif (abs(rho)< 0.34 and state != len(waypoints)-1):
            state = state+1        
        
        
        print("Current Pose: [" + str(pose_x) + ", " + str(pose_y) + "]")
        print("Current Waypoint:", state, " [" + str(waypoints[state][0]) + ", " + str(waypoints[state][1]) + "]")
        print("Distance: ", pose_x-waypoints[state][0], pose_y-waypoints[state][1])
        print("Distance: ", waypoints[state][1] - pose_y, waypoints[state][0] - pose_x*2)
        print("   ")

        front_obstacle = psValues[2]>3
        left_obstacle = psValues[1]>3
        right_obstacle = psValues[0]>3
        
        if (front_obstacle):
            vL=0
            vR=0
            print('obstacle') #for testing
        elif (right_obstacle or left_obstacle):
            vL=5*vL
            vR=5*vR
            print('obstacle') #for testing
        print(psValues[2], psValues[0], psValues[1])
        
    
    #####################################################
    #                    Odometry                       #
    #####################################################
    
    EPUCK_MAX_WHEEL_SPEED = 0.11695*SIM_TIMESTEP/1000.0 
    dsr=vR/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    dsl=vL/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    ds=(dsr+dsl)/2.0
    
    pose_y -= ds*math.sin(pose_theta)
    pose_x += ds*math.cos(pose_theta)
    pose_theta += (dsr-dsl)/EPUCK_AXLE_DIAMETER
    
    # Feel free to uncomment this for debugging
    #print("X: %f Y: %f Theta: %f " % (pose_x,pose_y,pose_theta))

    ## Stopping Criteria
    if (state == len(waypoints)-1):
        leftMotor.setVelocity(0) 
        rightMotor.setVelocity(0)
        break

    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)