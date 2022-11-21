"""ur5_webot_ctrl controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot, DistanceSensor, Motor
import math
import time

# create the Robot instance.
robot = Robot()


# get the time step of the current world.
pi = math.pi
timestep = 2*int(robot.getBasicTimeStep())
gripper = []
arm = []

state = 'start'

hand_joints = ['finger_1_joint_1','finger_2_joint_1','finger_middle_joint_1']
arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
base_joints = ['front_left_wheel_joint','front_right_wheel_joint','back_left_wheel_joint','back_right_wheel_joint']
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
for i in range(len(hand_joints)):
    gripper.append(robot.getDevice(hand_joints[i]))

for k in range(len(arm_joints)):
    arm.append(robot.getDevice(arm_joints[k]))
# Main loop:
# - perform simulation steps until Wem[0].setVelocity(0.1)bots is s,topping the controller

class MoveTheRobot(object):
    
    def __init__(self):
        self.state = 'start'
       
    def close_gripper(self):
        gripper[0].setPosition(1)
        gripper[1].setPosition(1)
        gripper[2].setPosition(1)
        
    def open_gripper(self):
        gripper[0].setPosition(0)
        gripper[1].setPosition(0)
        gripper[2].setPosition(0)
        
    def go_to_pose(self):
        arm[0].setPosition(1)
        arm[1].setPosition(-pi/2)
        arm[2].setPosition(pi/2)
        arm[3].setPosition(-pi)
        arm[4].setPosition(-pi/2)
        arm[5].setPosition(pi)     
        arm[0].setVelocity(1)
        arm[1].setVelocity(1)
        arm[2].setVelocity(1)
        arm[3].setVelocity(1)
        arm[4].setVelocity(1)
        arm[5].setVelocity(1)  

        
command = MoveTheRobot()
    
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    # Enter here functions to send actuator commands, like:
    time = robot.getTime()
    print(time)
    if time >= 0:
        command.go_to_pose() 
    if time >= 3.2:
        command.close_gripper()

    pass