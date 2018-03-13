# Operation-BrightStar
This repository contains packages for IranOpen 2018 virtual rescue robot .

### Preparing your system
It assumes that you have already installed  Gazebo 7.0 and ROS-kinetic on your system .

* Create a catkin workspace  
```
mkdir ~/workspace/src 
cd ~/workspace/src 
```
* Clone this repository 
```
git clone https://github.com/Delara-Forghani/Operation-BrightStar.git

```
* Now it's the time to make the catkin package 
```
catkin_make

```
* source the shell and variables :
```
source ./devel/setup.sh

```
## Running the tests


run the following commands:
* $ roslaunch setup robot_world.launch
* $ roslaunch communication_node registration_server.launch
* $ roslaunch setup spawn_4_pioneers.launch
* $ roslaunch communication_node update_info.launch
* $ roslaunch setup robot_navigation.launch
* $ roslaunch burgard burgard.launch
* $ roslaunch setup rviz.launch 

## Affiliation

CRLab at AmirKabir university of Technology.
