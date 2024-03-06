# motor_controller

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
