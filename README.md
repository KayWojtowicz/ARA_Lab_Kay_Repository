# This repository contains everything I worked on while working in the UMass Lowell ARA Lab

## Summary
There are currently two different ROS packages in the repository. These packages are used on Hello Robot Stretch along with the stretch_ros repository 

//
The stretch_ros repository can be found here: https://github.com/hello-robot/stretch_ros.git
//

## stretch_robot_client_interface_pkg
The purpose of this package is to contain all the needed files for the client interface.

This package also includes a launch file which launches a client interface application of the created collision prevention package as well as a server on a rosbridge on port 2022.

## collision_prevention_pkg
This package is currently able avoid obsticals using the lidar of the stretch robot. 

There is a python shell file for a client and a server that can be adjusted based on the application.

Using the current server the teleop twist python file provided by hello_robot was adjusted to help with collision prevention.

All default topics are for stretch robot in gazebo!
