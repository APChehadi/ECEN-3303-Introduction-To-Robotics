"""part2EPuck controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor, PositionSensor
import enum
import math 

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

# State Definitions
class States(enum.Enum):
    drive_forward_init = 1
    avoid_first_obstacle = 2
    drive_forward_1 = 3
    rotate_clockwise = 4
    drive_forward_2 = 5
    stop = 6
    
current_state = States.drive_forward_init


for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    # *** fix this shiz ***
    front_obstacle = psValues[0] > 80 or psValues[7] > 80 #68.0 #72
    left_obstacle = psValues[5] > 80 or psValues[6] > 80
    
#     Remove before submission
    print("loop")
    print(front_obstacle, psValues[0], psValues[7])
    print(left_obstacle, psValues[5], psValues[6])
    
    print(current_state)
    if current_state == States.drive_forward_init:
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        if front_obstacle:
            leftSpeed  = 0.0 * MAX_SPEED
            rightSpeed = 0.0 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            current_state = States.avoid_first_obstacle
            
            # *** Figure out PAUSE ***
    elif current_state == States.avoid_first_obstacle:
        # turn left 180 degrees using volocity controls
        leftSpeed  = -1 * MAX_SPEED
        rightSpeed = 1 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        robot.step(790) #12.5*TIME_STEP
        
        #drive forward
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        current_state = States.drive_forward_1
    elif current_state == States.drive_forward_1:
        if front_obstacle:
            current_state = States.rotate_clockwise
    elif current_state == States.rotate_clockwise:
        # rotate clockwise until the left distance sensor (ps5) reads <0.05m
        if left_obstacle == False:
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
        elif left_obstacle == True:
            current_state = States.drive_forward_2
    elif current_state == States.drive_forward_2:
        if left_obstacle:
            #drive forward
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
        else:
            leftSpeed  = 0.0 * MAX_SPEED
            rightSpeed = 0.0 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            current_state = States.stop
      	
    elif current_state == States.stop:
        leftSpeed  = 0.0 * MAX_SPEED
        rightSpeed = 0.0 * MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
    # else:
      	# blah