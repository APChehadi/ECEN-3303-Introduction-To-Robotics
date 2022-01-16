"""lab3 controller."""
# Copyright Prof. Bradley Hayes <bradley.hayes@colorado.edu> 2021
# University of Colorado Boulder CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67 # [rad/s]      #possibly use motor.getMaxVelocity()? message from Hayes
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.160 # [m] 

MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
target_pos = ('inf', 'inf')
robot_parts = []
goal_pos_array = [(4.79 - 3.29, 8.72 - 7.72, 0),
                  (5.99 - 3.29, 8.72 - 5.75, 0),
                  (4.17 - 3.29, 8.72 - 4.96, 0),
                  (3.93 - 3.29, 8.72 - 3.24, 0)] # (x,y,theta) from x= 4.78854 y=0 z=7.71861 in webots subtract

goal_pos = goal_pos_array[0]

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))
        print(robot_parts[i].getMaxVelocity())

# Odometry
pose_x = 0 
pose_y = 0 
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0 
vR = 0 
print(goal_pos_array[0])
i=0

while robot.step(timestep) != -1:
        # test target: position of yellow dot x= 4.78854 y=0 z=7.71861
        # goal zone: position of green dot  x = 3.92854 y=0 z = 3.23861
    #STEP 2: Calculate sources of error\

    #STEP 2.1: Position Error
    # p
    euclid_error = (((pose_x-goal_pos[0])**2+(pose_y-goal_pos[1])**2)**(0.5))
    print("euclid_error", euclid_error)
    
    
    #STEP 2.2: Bearing Error
    # math.atan2(y, x) for atan(y/x)
    # alpha, a
    bearing_error = math.atan2((goal_pos[1]-pose_y), (goal_pos[0]-pose_x))-pose_theta
    if bearing_error < -3.1415: 
        bearing_error += 6.283 
    print("bearing_error", bearing_error)


    #STEP 2.3: Heading Error
    # n
    heading_error = goal_pos[2] - pose_theta
    print("heading_error", heading_error)
    
    
    #STEP 2.4: Feedback Controller
    x_dot= 1*euclid_error
    theta_dot= 10*bearing_error
   
    
    #STEP 1.2: Inverse Kinematics Equations 
    # vL = phi*r = (2x_R - theta d) / 2r
    vL = (2*x_dot - theta_dot * AXLE_LENGTH) / 2
    print("vL", vL)

    # vR = phi*r =(2x_R + theta d) / 2 # r goes away because these are proportional
    vR = (2 * x_dot + theta_dot * AXLE_LENGTH) / 2 
    print("vR", vR)
    
    
    #STEP 2.9: Changing Waypoints 
    if (abs(euclid_error) < 0.5 and i != 3):
        i = i+1
        goal_pos=goal_pos_array[i]
       
    print("goal number" , i)
    print(goal_pos)
    
    
    # STEP 2.5: Compute wheel velocities (vL, vR)
    # v_L and v_R are equal to phi_L and phi_R. Can basically ignore. Did in 1.2


    #STEP 2.7: Proportional velocities
    max_val = max(abs(vL), abs(vR), MAX_SPEED)
    vL += max_val
    vR += max_val
    
    pctL = (vL / max(vL, vR) - 0.5) * 2
    pctR = (vR / max(vL, vR) - 0.5) * 2
    
    vL = pctL
    vR = pctR


    # STEP 2.6: Clamp wheel speeds
    if (vR > MAX_SPEED):
        vR = MAX_SPEED
    if (vR < -1*MAX_SPEED):
        vR = -1*MAX_SPEED
    if (vL > MAX_SPEED):
        vL = MAX_SPEED
    if (vL < -1*MAX_SPEED):
        vL = -1*MAX_SPEED
        
        
    # Odometry code. Don't change speeds (vL and vR) after this line
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)
    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)
    pose_theta += (distR-distL)/AXLE_LENGTH
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28

    ## Stopping Criteria
    if (i==3 and euclid_error<.2):
        robot_parts[MOTOR_LEFT].setVelocity(0) # pctL ) 
        robot_parts[MOTOR_RIGHT].setVelocity(0)
        break
        
    # TODO: Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity( vL) # pctL ) 
    robot_parts[MOTOR_RIGHT].setVelocity( vR) # pctR ) 

    print("pose_x", pose_x, "pose_y", pose_y, "pose_theta", pose_theta)
 