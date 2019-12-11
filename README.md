# Autonomous Path Planning Under Cyberattacks

The Autonomous Path Planning Under Cyberattacks Major Qualifying Project (MQP), as part of an undergraduate degree requirement at Worcester Polytechnic Institute. 

## Getting Started

These instructions will get you a copy of the project running on a local machine, in which the robot attempts to drive a "figure eight" trajectory while there is a simulated attack on the robot's LIDAR sensor, injecting it with false position data. The software uses a mitigation method in order to reduce the effect of the simulated attack, and allows for the robot to travel the pre-planned trajectory with little disruption to desired operation. 

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

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

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
