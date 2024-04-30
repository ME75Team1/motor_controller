#!/usr/bin/env python

# run using roslaunch motor_controller motors.launch to ensure params are set
# can test setting position using:
# rostopic pub -1 /leg_heights landing_optimizer/legHeights "{ids: [1,4], heights: [0.01,0.1]}"

# can test getting position using:
# rosservice call /get_position "id: 4"

import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from motor_controller.msg import legHeights

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_OPERATING_MODE     = 11

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
# DXL_IDS                     = rospy.get_param('/dynamixel/running_ids')               # Dynamixel ID : 1
DXL_IDS = [4]
BAUDRATE                    = 57600         # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -1048575               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1048575        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
OPERATING_MODE              = 4               # extended position control mode

ZERO_POSITION_VALUES = {}
METERS_PER_REVOLUTION = rospy.get_param('/dynamixel/meters_per_revolution')
RESOLUTION = 4096 # pulses per revolution
MAX_HEIGHT = rospy.get_param('/dynamixel/max_extension')
METERS_PER_PULSE = METERS_PER_REVOLUTION / RESOLUTION

TWOS_COMPLEMENT_UPPER_LIMIT = 2147483647
TWOS_COMPLEMENT_RANGE = 4294967295

LF_ID = rospy.get_param('/dynamixel_ids/lf')
RF_ID = rospy.get_param('/dynamixel_ids/rf')
LB_ID = rospy.get_param('/dynamixel_ids/lb')
RB_ID = rospy.get_param('/dynamixel_ids/rb')

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

MINIMUM_HEIGHT_DIFFERENCE = rospy.get_param('/dynamixel/minimum_height_difference')
MINIMUM_ALT_DIFFERENCE = rospy.get_param('/dynamixel/minimum_altitude')
current_positions = [-1000,-1000,-1000,-1000]

def set_motor_positions(position_to_set):
    print("Setting Motors to Large Negative Position")
    for id in DXL_IDS:
        print(f"DXL {id} set to position_to_set")
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, position_to_set)
        if dxl_comm_result !=0:
            print("DXL Communication Result: ", dxl_comm_result)
        if dxl_error !=0:
            print("DXL Error: ", dxl_error)

def main():
    # Open port
    try:
        portHandler.openPort()
        print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    print()

    for id in DXL_IDS:
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"ID {id}")
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print(f"ID {id}: Torque has been successfully disabled")

        # enable extended position mode
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, OPERATING_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print(f"ID {id}: Extended Position Control Mode has been successfully set")

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print(f"ID {id}: Torque has been successfully enabled")
        
        print()

    print("Ready to get & set Position.")

    set_motor_positions(100000)


if __name__ == '__main__':
    main()
