# motor_controller

## Description
This package takes heights from [landing_optimizer](https://github.com/ME75Team1/landing_optimizer) and moves Dynamixel motors accordingly.

## Getting Started
### Prerequisites
- Ubuntu 20.04
- [Stereolabs zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [landing_optimizer](https://github.com/ME75Team1/landing_optimizer)

### Build the Repository
The `motor_controller` is a catkin package. It depends on the following ROS packages:
- `roscpp`
- `rospy`
- `std_msgs`
- `dynamixel_sdk`

Open a terminal, clone the respository, update the dependencies and build the packages:
```
cd ~/catkin_ws/src
git clone https://github.com/ME75Team1/motor_controller.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source ./devel/setup.bash
```

### Update the local repository
To update the repository to the latest release you must use the following command to retrieve the latest commits of `motor_controller`.

```
git checkout main # if you are not on the main branch
git pull
```
Remember to always clean the cache of your catkin workspace before compiling with the catkin_make command to be sure that everything will work as expected:
```
roscd
cd ..
rm -rf build devel
catkin_make
```

## Run the motor_controller
Launch the ZED node from the [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)

Run the leg height calculator from the [landing_optimizer](https://github.com/ME75Team1/landing_optimizer).

Then run the following node:

### Motor Controller Node
#### Setting Motors to Zero
- Before running the node, use `rosrun motor_controller zero_motors.py` to make all the legs jam into the housing.
#### Initial Launch
- This node can be launched by `roslaunch motor_controller initial_motors.launch`.
- It reads the `/leg_heights` message from the leg height calculator in [landing_optimizer](https://github.com/ME75Team1/landing_optimizer) and moves the Dynamixel motors based on the message.
#### Subsequent Launches
- For subsequent launches, the node should be launched with `roslaunch motor_controller second_motors.launch`.
- This launch file uses parameters saved by the initial launch that store the initial location of each leg.

## Modifications to Original Code
This repository is a modified version of the Dynamixel SDK examples at https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/ros/dynamixel_sdk_examples. 
A copy of the original Apache License 2.0 is included in this repository. 

The changes we made are listed below:
- The original .msg files in msg were deleted and replaced with a legHeights.msg file.
- The original .cpp files in src were deleted.
- The original read_write_node.py in src was modified to accept legHeights messages, move 4 motors instead of 1, and change the motors to extended position control mode.
- A script called zero_motors.py was added to src.
- The BulkGetItem.srv and SyncGetPosition.srv files were removed. GetPosition.srv was modified to change the sizes of variables.
- The CMakeLists.txt and package.xml were updated with the files we added and removed.
