# Autonomous Path Planning Under Cyberattacks

The Autonomous Path Planning Under Cyberattacks Major Qualifying Project (MQP), as part of an undergraduate degree requirement at Worcester Polytechnic Institute. 

## Getting Started

These instructions will get you a copy of the project running on a local machine, in which the robot attempts to autonomously drive a "figure eight" trajectory while there is a simulated attack on the robot's LIDAR sensor, injecting it with false position data. The software uses a mitigation method in order to reduce the effect of the simulated attack, and allows for the robot to travel the pre-planned trajectory with little disruption to desired operation. 

### Prerequisites

This project runs code on a remote computer using Ubuntu Linux 16.04 with ROS Kinetic Installed. This remote computer communicates to a Turtlebot3 Burger robot via Wi-Fi connection. 

### Installing

First, in order to run the simulation, ROS Kinetic must be installed. Installation instructions for ROS Kinetic can be found on the [ROS Documentation Website](http://wiki.ros.org/kinetic/Installation). Once ROS is installed, create a [Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in which the project package can be created. Once the workspace directory is created, the workspace must be created with catkin via
```
catkin_make
```
and the workspace must be sourced with
```
source devel/setup.bash
```
The entire repository can then be placed as a folder within the workspace directory. 

## Running the simulation

The project consists of two tests, the LQG Controller which allows for the robot to travel the preplanned trajectory, and the LQG Controller with the migitation method under a simulated false data injection attack. 

### LQG Controller Test

In order to run the LQG Controller test, roscore must first be started in a terminal window on a remote computer:
```
roscore
```
Then, connect to the Turtlebot3 from the remote computer in a second terminal window and bringup the basic ROS packages on the turtlebot (must be run on the Turtlebot3 itself via SSH, not on the remote computer)
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
Once roscore is up and the bringup packages have been run on the robot, navigate to the pathplanning package folder within the catkin workspace using a third terminal window. An easy way to navigate to this folder within ros is:
```
roscd pathplanningmqp
```
Once in the pathplanningmqp directory, run the test using:
```
rosrun pathplanningmqp FBL_LQD_ros.py
```

### False Data Injection Test
Explain what these tests test and why

```
Give an example
```

## Additional Notes

If the Turtlebot3 model is not set within ROS, an error may occur when trying to run either of the tests. In order to set the model, use:
```
export TURTLEBOT3_MODEL=burger
```
Within the same terminal window in which the python file is trying to be ran
If the python file cannot be executed, make sure that the python file is given proper executable user permissions:
```
chmod +x FILE_NAME_HERE.py
``` 

## Built With

* [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) - An open-source autonomous robot platform
* [Robot Operating System (ROS)](http://wiki.ros.org/) - Robot Middleware Package used on Turtlebot3
* [NumPy](https://numpy.org/) - Mathematical Library for Python in order to complete calculations using matricies
* [CVXPy](https://www.cvxpy.org/) - Convex Optimization solver library for Python 


## Authors

* **Minh Le** 
* **Christopher Letherbarrow** 

## Acknowledgments

We would like to thank our advisor, Professor Andrew Clark for his guidance and feedback throughout the course of this project. Also, we would like to thank graduate students Hongchao Zhang and Zhouchi Li, who were always available to offer us advice and feedback with our progress whenever we needed it. 
