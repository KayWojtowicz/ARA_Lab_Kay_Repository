#!/usr/bin/env python3

import rospy #always need
import sys #needed for exit
from std_msgs.msg import * #imports all std_msgs, needed to send string to client interface
from geometry_msgs.msg import Twist #needed to move the base
from sensor_msgs.msg import LaserScan #needed for lidar and camera
from collision_prevention_pkg.srv import * #includes service


def driver_callback(data):

    #previous rows are just for testing. Add in your own code!!!
    
    index = 0
    while index < 1999: #is checking all every index for the lidar, acts as a for loop
        if data.ranges[index] < 0.5:
            rospy.wait_for_service('collision_prevention_service') #waits for collision service to respond
            try:
                prevention_service_call = rospy.ServiceProxy('collision_prevention_service', collision_prevention_service)
                answer = prevention_service_call(index)
                return answer # answer is empty
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e) #if service isn't running this will pop up
                sys.exit(1)
        
        index+=1

if __name__ == '__main__':
    rospy.init_node('collision_node',anonymous=True)
    sub = rospy.Subscriber('/laser_scan', LaserScan, driver_callback) #subscribes to lidar values
    
    rospy.spin() #This is needed
