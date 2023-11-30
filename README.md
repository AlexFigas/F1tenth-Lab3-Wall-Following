# F1tenth Lab 3 - Wall Following

This repository contains my implementation of [F1tenth Lab 3 - Wall Following](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab3.html).

## Getting Started

### Prerequisites

Make sure you have the following dependencies installed:

- [ROS (Robot Operating System)](https://wiki.ros.org/ROS/Installation) - I used ROS Noetic.
- [F1tenth Simulator](https://f1tenth.readthedocs.io/en/latest/going_forward/simulator/sim_install.html)

### Installation

1. Clone this repository to your catkin workspace source folder:

    ```bash
    cd catkin_ws/src
    git clone https://github.com/AlexFigas/F1tenth-Lab3-Wall-Following.git wall_following
    ```

2. Compile the code:

    ```bash
    cd ..
    catkin_make
    ```

### Usage

Run the ROS node with the following command:

```bash
roslaunch wall_following wall_following.launch
```