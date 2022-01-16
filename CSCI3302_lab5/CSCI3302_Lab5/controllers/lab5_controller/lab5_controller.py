"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
from matplotlib import colors as colors

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
# mode = 'autonomous'




###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = (4.47, 8.06) # (Pose_X, Pose_Z) in meters
    end_w = (10.0, 7.0) # test value: (7.0, 5.5) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = (360-int(start_w[1]*30), int(start_w[0]*30)) # (x, y) in 360x360 map
    end = (360-int(end_w[1]*30), int(end_w[0]*30)) # (x, y) in 360x360 map

    # print(start_w, end_w)
    # print(start, end)

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
            current_index = 0
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
                # print(node_position, map[int(node_position[0])][int(node_position[1])])
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
    # plt.show() 


    # Part 2.2: Compute an approximation of the “configuration space”
    new_map=map.copy()
    for _ in range(5):
        for i in range(1, 359):
            for j in range(1, 359):
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
    # plt.show()
    map=new_map
            
    

    # Part 2.3 continuation: Call path_planner
    path=path_planner(map, start, end)
    # print("path", path)

    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it            
    waypoints = [] # in 12x12m space
    for (x, y) in path:
        waypoints.append( (y/30, (-x+360)/30) ) 
        map[x][y] = 1
    # print("waypoints", waypoints)
    np.save("path.npy", waypoints) 
    plt.imshow(map)
    plt.show()
    

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization
# Initialize your map data structure here as a 2D floating point array
map = np.zeros((360,360)) # Replace None by a numpy 2D floating point array
waypoints = []

if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    waypoints = np.load("path.npy")
    print("Waypoints loaded")

state = 0 # use this to iterate through your path

while robot.step(timestep) != -1 and mode != 'planner':

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
    
        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
            g = map[360-int(wy*30), int(wx*30)]
            if g>1:
                g = 1
                
            color = (g*256**2+g*256+g)*255
            color = int(color)
            
            display.setColor( color ) 
            display.drawPixel(360-int(wy*30),int(wx*30))
            # Add to map
            map[360-int(wy*30), int(wx*30)] += 0.005
            # print(map)

    # Draw the robot's current pose on the 360x360 display
    display.setAlpha(1)
    display.setColor(int(0xFF0000))
    display.drawPixel(360-int(pose_y*30),int(pose_x*30))



    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
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
        # Part 3.2: Feedback controller
        #STEP 1: Calculate the error

        # Euclid Error
        rho = (((pose_x-waypoints[state][0])**2+(pose_y-waypoints[state][1])**2)**(0.5))

        # Bearing Error
        alpha = -(math.atan2(waypoints[state][1]-pose_y,waypoints[state][0]-pose_x) + pose_theta)

        #STEP 2: Controller

        # x dot
        dX = 1*rho

        # theta dot
        dTheta = 10*alpha

        #STEP 3: Compute wheelspeeds
        vL = (2*dX - dTheta * AXLE_LENGTH) / 2
        vR = (2*dX + dTheta * AXLE_LENGTH) / 2 

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
        if ( rho < 0.5 and state != len(waypoints)-1): 
            state = state+1
        
        # print("Positions", pose_x, pose_y)
        # print("goal state", state, waypoints[state])
        # print("   ")
        

    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    ## Stopping Criteria
    if (state == len(waypoints)-1):
        robot_parts[MOTOR_LEFT].setVelocity(0) 
        robot_parts[MOTOR_RIGHT].setVelocity(0)
        break

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)