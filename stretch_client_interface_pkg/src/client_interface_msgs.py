#! /usr/bin/env python3

import rospy
from std_msgs.msg import * # imports all std_msgs
from geometry_msgs.msg import Twist # needed to move the base
from client_custom_msgs.msg import * # imports everything from the custom msgs created

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

state = None
#####################################################################################

def joint_states_callback(joint_state_msg) :
    global state
    state = joint_state_msg    

#####################################################################################

def subscriber_callback(given_string) :
    message = given_string.command
    
    if 'position' in message :
        move_position(given_string)

    elif 'base' in message :
        move_base(given_string)
        
    elif 'arm' in message :
        move_arm(given_string)
        
    elif 'lift' in message :
        move_lift(given_string)
        
    elif 'gripper' in message :
        move_gripper(given_string)

    elif 'wrist' in message :
        move_wrist(given_string)
    
        
#####################################################################################
#needs work!!!!
def move_position(given_string) : 
    message = given_string.command
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(0.1)
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
    
    if 'arm' in message:
        if 'stow' in message :
            trajectory_goal.trajectory.joint_names = ['joint_lift', 'joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3', 'joint_wrist_yaw']
            point.positions[0,0,0,0,0,0]
            endclient = arm_trajectoryClient

        elif 'home' in message :
            trajectory_goal.trajectory.joint_names = ['joint_lift', 'joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3', 'joint_wrist_yaw']
            point.positions[0.5, 0.2, 0.2, 0.2, 0.2, 10]
            endclient = arm_trajectoryClient
            

    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

    endclient.send_goal(trajectory_goal)

#####################################################################################

def move_base(given_string) :
    message = given_string.command
    base_movement = Twist()

    if 'forward' in message :
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

def move_arm(given_string) :
    message = given_string.command
    endclient = arm_trajectoryClient

    if 'extend' in message :
        command = {'joint': 'wrist_extension', 'delta': 0.05}
         
    elif 'collapse' in message :
        command = {'joint': 'wrist_extension', 'delta': -0.05}   
    
    send_command(endclient, command)

#####################################################################################

def move_lift(given_string) :
    message = given_string.command
    endclient = arm_trajectoryClient
    
    if 'up' in message :
        command = {'joint': 'joint_lift', 'delta': 0.08}
    elif 'down' in message :
        command = {'joint': 'joint_lift', 'delta': -0.08}
        
    send_command(endclient, command)
    
#####################################################################################

def move_gripper(given_string) :
    message = given_string.command
    endclient = gripper_trajectoryClient

    if 'open' in message :
        command = {'joint': 'joint_gripper_finger_right', 'delta': 0.1}
    elif 'close' in message :
        command = {'joint': 'joint_gripper_finger_right', 'delta': -0.1}
    
    send_command(endclient, command)

#####################################################################################

def move_wrist(given_string) :
    message = given_string.command
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

        if joint_name in ['joint_lift', 'joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt']:
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

    rospy.spin()
