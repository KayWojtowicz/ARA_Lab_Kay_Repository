#! /usr/bin/env python3

import rospy
import math
import time
from std_msgs.msg import * # imports all std_msgs
from geometry_msgs.msg import Twist # needed to move the base
# from client_custom_msgs.msg import * # imports everything from the custom msgs created
import stretch_body.robot as robot
import os

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

state = None
#robot= stretch_body.robot.Robot()

#####################################################################################

def joint_states_callback(joint_state_msg) :
    global state
    state = joint_state_msg    

#####################################################################################

def subscriber_callback(given_string) :
    message = given_string.data
    
    if 'position' in message :
        move_position(message)

    elif 'base' in message :
        move_base(message)
        
    elif 'arm' in message :
        move_arm(message)
        
    elif 'lift' in message :
        move_lift(message)
        
    elif 'gripper' in message :
        move_gripper(message)

    elif 'wrist' in message :
        move_wrist(message)
    
        
#####################################################################################

def move_position(message) : 
    
    if 'arm' in message:
        if 'stow' in message :
            os.system("rosservice call /stow_the_robot")
#            time.sleep(5)
#            robot.arm.move_to(0)
#            robot.lift.move_to(0.2)
#            robot.end_of_arm.move_to('wrist_yaw',0)
#            robot.end_of_arm.move_to('wrist_pitch',0)
#            robot.end_of_arm.move_to('wrist_roll',0)
#            robot.push_command()
#            time.sleep(5)

            

        elif 'grasping' in message :
            robot.startup()

            robot.arm.move_to(0)
            robot.lift.move_to(0.2)
            robot.end_of_arm.move_to('wrist_yaw',0)
            robot.end_of_arm.move_to('wrist_pitch',0)
            robot.end_of_arm.move_to('wrist_roll',0)
            robot.push_command()
            time.sleep(5)

            robot.stop()

#####################################################################################

def move_base(message) :

    base_movement = Twist()

    if 'degrees' in message:
        if 'ninety' in message:
            rad = 90 * (math.pi)/180

        if 'left' in message:
            robot.startup()

            robot.base.rotate_by(rad)
            robot.push_command()
            robot.sleep(2)

            robot.stop()

        if 'right' in message:
            robot.startup()

            robot.base.rotate_by(-rad)
            robot.push_command()
            robot.sleep(2)

            robot.stop()

    elif 'forward' in message :
        base_movement.linear.x = 1
        base_movement.angular.z = 0
        
    elif 'backwards' in message :
        base_movement.linear.x = -1
        base_movement.angular.z = 0
        
    elif 'left' in message :
        base_movement.linear.x = 0
        base_movement.angular.z = 1
        
    elif 'right' in message :
        base_movement.linear.x = 0
        base_movement.angular.z = -1
        
    base_pub.publish(base_movement)

#####################################################################################

def move_arm(message) :

    endclient = arm_trajectoryClient

    if 'extend' in message :
        command = {'joint': 'wrist_extension', 'delta': 0.05}
         
    elif 'collapse' in message :
        command = {'joint': 'wrist_extension', 'delta': -0.05}   
    
    send_command(endclient, command)

#####################################################################################

def move_lift(message) :

    endclient = arm_trajectoryClient
    
    if 'up' in message :
        command = {'joint': 'joint_lift', 'delta': 0.08}
    elif 'down' in message :
        command = {'joint': 'joint_lift', 'delta': -0.08}
        
    send_command(endclient, command)
    
#####################################################################################

def move_gripper(message) :

    endclient = gripper_trajectoryClient

    if 'open' in message :
        command = {'joint': 'joint_gripper_finger_right', 'delta': 0.1}
    elif 'close' in message :
        command = {'joint': 'joint_gripper_finger_right', 'delta': -0.1}
    
    elif 'rotate' in message :
        if 'right' in message:
            command = {'joint': 'joint_wrist_roll', 'delta': 0.1}
        elif 'left' in message:
            command = {'joint': 'joint_wrist_roll', 'delta': -0.1}
    
    elif 'tilt' in message:
        if 'up' in message:
            command = {'joint': 'joint_wrist_pitch', 'delta': 0.1}
        elif 'down' in message:
            command = {'joint': 'joint_wrist_pitch', 'delta': -0.1}

    send_command(endclient, command)

#####################################################################################

def move_wrist(message) :

    endclient = arm_trajectoryClient

    if 'left' in message :
        command = {'joint': 'joint_wrist_yaw', 'delta': 0.2}
    elif 'right' in message :
        command = {'joint': 'joint_wrist_yaw', 'delta': -0.2}

    send_command(endclient, command)

#####################################################################################

def send_command(endclient, command) :

    joint_state = state

    if (joint_state is not None) and (command is not None):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.1)
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            
        joint_name = command['joint']

        if joint_name in ['joint_lift', 'joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt', 'joint_wrist_roll', 'joint_wrist_pitch', 'joint_gripper_finger_right']:
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
        elif joint_name in ["wrist_extension"]:
                trajectory_goal.trajectory.joint_names = ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']
            
        positions = []

        for j_name in trajectory_goal.trajectory.joint_names:
            joint_index = joint_state.name.index(j_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta/len(trajectory_goal.trajectory.joint_names)
            positions.append(new_value)
            
        point.positions = positions

        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        endclient.send_goal(trajectory_goal)

#####################################################################################

if __name__ == "__main__":
    rospy.init_node('client_interface_msgs_node') #initializes server node
    
    
    command = None
    
    base_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes to stretch base
    
    command_sub = rospy.Subscriber('/client_command', String, subscriber_callback) #subscribes to lidar values
    joint_state_sub = rospy.Subscriber('/joints', JointState, joint_states_callback) #subscribes to joint state topic
    arm_trajectoryClient = actionlib.ActionClient('/arm_AC', FollowJointTrajectoryAction) #allows for goals to be sent to the arm
    gripper_trajectoryClient = actionlib.ActionClient('/gripper_AC', FollowJointTrajectoryAction) #allows for goals to be sent to the gripper
    #base_trajectoryClient = actionlib.ActionClient('/base_AC', FollowJointTrajectoryAction) #allows for goals to be sent to the base
    
    os.system("rosservice call /switch_to_navigation_mode \"{}\"")

    rospy.spin()
