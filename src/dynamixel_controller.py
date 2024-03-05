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
import yaml

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
DXL_IDS                     = rospy.get_param('/dynamixel/running_ids')               # Dynamixel ID : 1
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

def set_goal_pos_callback(data, f):
    print("Set Goal Position of ID %s = %s" % (data.ids, data.heights))
    min_height = min(data.heights)
    params_dict = {}
    if min_height > MINIMUM_ALT_DIFFERENCE:
        for i, id in enumerate(data.ids):
            if id in DXL_IDS:
                adjusted_height = data.heights[i] - min_height
                adjusted_height = max(min(adjusted_height, MAX_HEIGHT), 0)
                pulses_from_zero = int(adjusted_height/METERS_PER_PULSE)
                position_to_set = pulses_from_zero + ZERO_POSITION_VALUES[id]
                print(id, ": ", position_to_set)
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, position_to_set)
                if dxl_comm_result !=0:
                    print("DXL Communication Result: ", dxl_comm_result)
                if dxl_error !=0:
                    print("DXL Error: ", dxl_error)
                if id == LF_ID and abs(adjusted_height - current_positions[0]) > MINIMUM_HEIGHT_DIFFERENCE:
                    params_dict['/starting_leg_heights/lf'] = adjusted_height
                    current_positions[0] = adjusted_height
                elif id == RF_ID and abs(adjusted_height - current_positions[1]) > MINIMUM_HEIGHT_DIFFERENCE:
                    params_dict['/starting_leg_heights/rf'] = adjusted_height
                    current_positions[1] = adjusted_height
                elif id == RB_ID and abs(adjusted_height - current_positions[2]) > MINIMUM_HEIGHT_DIFFERENCE:
                    params_dict['/starting_leg_heights/rb'] = adjusted_height
                    current_positions[2] = adjusted_height
                elif id == LB_ID and abs(adjusted_height - current_positions[3]) > MINIMUM_HEIGHT_DIFFERENCE:
                    params_dict['/starting_leg_heights/lb'] = adjusted_height
                    current_positions[3] = adjusted_height
    
    yaml.safe_dump(params_dict, f)
        

def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    present_pos = twos_complement_to_signed_integer(dxl_present_position)
    print("Present Position of ID %s = %s" % (req.id, present_pos))
    return present_pos

def get_present_pos_from_ID(id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)
    return twos_complement_to_signed_integer(dxl_present_position)

def twos_complement_to_signed_integer(val):
    if val > TWOS_COMPLEMENT_UPPER_LIMIT:
        return(-1 * (TWOS_COMPLEMENT_RANGE - val))
    else:
        return val

def read_write_py_node():
    rospy.init_node('read_write_py_node')
    with open('/home/team1/catkin_ws/src/motor_controller/params/params_saved.yaml', 'w') as f:
        rospy.Subscriber('/leg_heights', legHeights, set_goal_pos_callback, f, queue_size=1)
        rospy.Service('get_position', GetPosition, get_present_pos)
        rospy.spin()

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

    lf_initial_height = rospy.get_param('/starting_leg_heights/lf')
    lb_initial_height = rospy.get_param('/starting_leg_heights/lb')
    rb_initial_height = rospy.get_param('/starting_leg_heights/rb')
    rf_initial_height = rospy.get_param('/starting_leg_heights/rf')

    lf_initial_offset = lf_initial_height/METERS_PER_PULSE
    rf_initial_offset = rf_initial_height/METERS_PER_PULSE
    lb_initial_offset = lb_initial_height/METERS_PER_PULSE
    rb_initial_offset = rb_initial_height/METERS_PER_PULSE

    lf_initial_position = get_present_pos_from_ID(LF_ID)
    rf_initial_position = get_present_pos_from_ID(RF_ID)
    lb_initial_position = get_present_pos_from_ID(LB_ID)
    rb_initial_position = get_present_pos_from_ID(RB_ID)

    initial_positions = {}
    initial_positions[LF_ID] = lf_initial_position
    initial_positions[RF_ID] = rf_initial_position
    initial_positions[LB_ID] = lb_initial_position
    initial_positions[RB_ID] = rb_initial_position

    print("Initial Positions Read from Dynamixels: ", initial_positions)

    ZERO_POSITION_VALUES[LF_ID] = int(lf_initial_position - lf_initial_offset)
    ZERO_POSITION_VALUES[RF_ID] = int(rf_initial_position - rf_initial_offset)
    ZERO_POSITION_VALUES[LB_ID] = int(lb_initial_position - lb_initial_offset)
    ZERO_POSITION_VALUES[RB_ID] = int(rb_initial_position - rb_initial_offset)

    print("Position Values at Zero Height: ", ZERO_POSITION_VALUES)

    print("Ready to get & set Position.")

    read_write_py_node()


if __name__ == '__main__':
    main()
