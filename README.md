
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# ROS beginner_tutorials
This repository contains basic concepts of ROS nodes, topics and messages by creating a ROS Publisher and Subscriber in C++. It contains a package named beginner_tutorials which has src, CMakeLists.txt and package.xml. The src directory has Publisher(talker.cpp) and Subscriber(listener.cpp).

## Dependencies
For this project, you require following dependencies:

- Ubuntu 16.04
- ROS kinetic
- catkin

ROS can be installed from the http://wiki.ros.org site, click on following link to navigate to the installation guide:
For ROS, [here](http://wiki.ros.org/kinetic/Installation)

## Building this package(beginner_tutorials)
To build the given package, first you need to create a catkin workspace, the steps of which are as below:
```
mkdir -p ~/your_workspace/src
cd ~/your_workspace/
catkin_make
source devel/setup.bash
```
After making a catkin workspace and building the workspace using catkin_make, you now need to follow these below steps to install and build the package:

```
cd src/
git clone --recursive https://github.com/nakulpatel94/beginner_tutorials.git
cd ..
catkin_make
```
In above steps, we have again executed catkin_make command to rebuild our workspace with the freshly installed package(beginner_tutorials)


## Running this package(beginner_tutorials)
After successfully building the workspace with the package, we have already created the executables for our nodes:

- talker is the executable for talker.cpp
- listener is the executable for listener.cpp

Now, first step is to run roscore to start the ROS system

1. In a new terminal, execute roscore command as follows:

```
roscore
```

Next, ensure that you source your workspace using following command:

2. In a new teminal, navigate to your workspace and source the setup.bash file:
```
cd ~/your_workspace/
source devel/setup.bash
```
Now, in two terminals, run the nodes, with publisher(talker) node being run first.

3. Running the publisher node using rosrun 
```
rosrun beginner_tutorials talker
```

Now, run the subscriber(listener) node.

4. Running the subscriber node using rosrun 
```
rosrun beginner_tutorials listener
```

 
