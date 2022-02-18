# F1tenth Lab 3 - Wall Following


This is my implementation of the [F1tenth Lab 3 - Wall Following](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab3.html)

## Getting Started

### Dependencies and Installing Guides

* [ROS](https://wiki.ros.org/ROS/Installation) - I used ROS Noetic
* [F1tenth Simulator](https://f1tenth.readthedocs.io/en/latest/going_forward/simulator/sim_install.html)

### Executing program

* Clone this repo to your catkin workspace source folder
```
cd catkin_ws/src
git clone https://github.com/AlexFigas/F1tenth-Lab3-Wall-Following.git
```
* Rename the cloned folder to "wall_following"
```
mv F1tenth-Lab3-Wall-Following  wall_following
```
* Compile the code
```
cd ..
catkin_make
```
* Run the ROS node
```
roslaunch wall_following wall_following.launch
```