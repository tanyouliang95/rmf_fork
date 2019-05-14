#!/usr/bin/env python3
# license removed for brevity
# This scripts runs dynamixel sdk stuffs, which serial controls the behavior of the gripper
# change motor control params according to the addresses below
#
# Created by Poh Yong Keat 2018, , Tan You Liang
# Maintained by Tan You Liang 2019 (RMF Project)
#
# for low level address control, check: http://support.robotis.com/en/product/actuator/dynamixel/ax_series/ax-18f.htm


import time
import os

from dynamixel_sdk import *
from dynamixel_sdk.port_handler import *
from dynamixel_sdk.packet_handler import *
from termcolor import colored


# Control table address for Dynamixel AX
ADDR_AX_TORQUE_ENABLE       = 24
ADDR_AX_GOAL_POSITION       = 30
ADDR_AX_VELOCITY            = 32
ADDR_AX_PRESENT_POSITION    = 36
ADDR_AX_PRESENT_LOAD        = 40
ADDR_AX_PRESENT_TEMP        = 43
ADDR_AX_TORQUE_LIMIT        = 34
ADDR_AX_MAX_TORQUE          = 14

# Torque Compliance
ADDR_CW_COMPLIANCE_MARGIN   = 26
ADDR_CCW_COMPLIANCE_MARGIN  = 27
ADDR_CW_COMPLIANCE_SLOPE    = 28
ADDR_CCW_COMPLIANCE_SLOPE   = 29

# Protocol version
PROTOCOL_VERSION1           = 1.0               #  Protocol version is used to communicate with Dynamixel

# Default setting
DXLleft_ID                  = 1                 # Left dynamixel id 1, right dynamixel id 2
DXLright_ID                 = 2
BAUDRATE                    = 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

DXL_MOVING_STATUS_THRESHOLD = 5                 # Allowable tolerance between (dynamixel) goal and current position
DXL_TORQUE_LIMIT            = 1024

CW_SLOPE_VALUE              = 2                 # low val represents high gradient
LEFTLOAD_THRESHOLD = 0.3                        # Adjust grip load thresholds accordingly to prevent overloading
RIGHTLOAD_THRESHOLD = 0.3
STEP_RELEASE_ANGLE = 1 #1                        # Rate of step release to reduce load magnitude



################################################################################################################################
################################################################################################################################


class DXL_Controller():

    # initialise dynamixel connection & enable torque
    def __init__(self):
        
        # check if running on windows nt
        try:
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
        except termios.error as e:
            print("pass error \n\n")
            pass

        # Initialize PortHandler instance and set port path
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance, with dxl protocal version 
        self.packetHandler = PacketHandler(PROTOCOL_VERSION1)

        # Try to use serial connect to dxl, then open port
        try:
            if self.portHandler.openPort():
                print(" - Succeeded to open the port")
            else:
                print(colored("Error! Failed to open the port", "red"))
                exit(0)
        except serial.serialutil.SerialException:
            print(colored(" Error! Gripper is not connected, pls check connection and chmod", "red"))
            exit(0)

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print(" - Succeeded to change the baudrate")
        else:
            print(colored("Error! Failed to change baud rate", "red"))
            exit(0)

        # Enable left Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
        if (dxl_comm_result == COMM_SUCCESS):
            print("Dynamixel#%d has been successfully connected" % DXLleft_ID)
        else:
            print(colored(" Error! Failed in connecting left dxl motor :( ", "red") )
            exit(0)

        # Enable right Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
        if (dxl_comm_result == COMM_SUCCESS):
            print("Dynamixel#%d has been successfully connected" % DXLright_ID)
        else:
            print(colored(" Error! Failed in connecting right dxl motor :( ", "red") )
            exit(0)

        # -------------------------------------- Edit Torque Configuration -------------------------------------------

        # Read Dynamixel gripper TORQUE LIMIT
        torque_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_TORQUE_LIMIT)

        # Manage Torque Limit
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_TORQUE_LIMIT, DXL_TORQUE_LIMIT)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_TORQUE_LIMIT, DXL_TORQUE_LIMIT)

        ## Compliance settings
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_CW_COMPLIANCE_SLOPE, CW_SLOPE_VALUE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_CCW_COMPLIANCE_SLOPE, CW_SLOPE_VALUE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLright_ID, ADDR_CW_COMPLIANCE_SLOPE, CW_SLOPE_VALUE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLright_ID, ADDR_CCW_COMPLIANCE_SLOPE, CW_SLOPE_VALUE)

        # MaxTorque, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_MAX_TORQUE)
        # print(" # Max Torque: ", MaxTorque)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_MAX_TORQUE, 1000)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_MAX_TORQUE, 1000)


    # Open dynamixel gripper
    # @Non-blocking
    def dxl_open(self, left_open_pos, right_open_pos, servo_speed):

        # Set motor speed
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_VELOCITY, servo_speed)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_VELOCITY, servo_speed)
        # Set torque on
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)

        # Write Dynamixel gripper open position to left and right motors
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, left_open_pos )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, right_open_pos)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))



    # Close dynamixel gripper
    # @Non-blocking
    def dxl_close(self, left_close_pos, right_close_pos, servo_speed):

        # Set motor speed
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_VELOCITY, servo_speed)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_VELOCITY, servo_speed)
        # Set torque on
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)

        # Write Dynamixel gripper closed position to left and right motors
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, left_close_pos)
        if dxl_comm_result != COMM_SUCCESS:
            print("DXL Close Result: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("DXL Close Result: %s" % self.packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, right_close_pos)
        if dxl_comm_result != COMM_SUCCESS:
            print("DXL Close Result: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("DXL Close Result: %s" % self.packetHandler.getRxPacketError(dxl_error))


    '''
    Get current joint position of both servos
    configurable in PARAM above
    @return: list(float)  [ left motor pos, right motor pos ],  -1.0 means error
    '''
    def dxl_get_pos(self):

        try:
            # Read Dynamixel gripper present position
            dxlleft_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            dxlright_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            return [dxlleft_present_position,dxlright_present_position]

        except IndexError:
            print(colored("GetPos ERROR!!! Exception Index Error in Dynamixel SDK...", "red"))
            return [-1.0,-1.0]


    '''
    Get current temperature of both servos
    @return: list(float)  [ left motor temp, right motor temp ],  -1.0 means error
    '''
    def dxl_get_temp(self):

        try:
            # Read Dynamixel gripper present temperature
            dxlleft_present_temp, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_PRESENT_TEMP)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            dxlright_present_temp, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_PRESENT_TEMP)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            return [dxlleft_present_temp,dxlright_present_temp]
        
        except IndexError:
            print(colored("GetTemp ERROR!!! Exception Index Error in Dynamixel SDK...", "red"))
            return [-1.0,-1.0]


    '''
    Checks if dynamixel goal position has been met
    @return: (int) if 1: reaches, if 0: not reaching
    '''
    def dxl_is_reached(self, goal,present_pos):
        if(goal == 'close'):
            goal_pos = [DXLleft_CLOSE_POSITION_VALUE,DXLright_CLOSE_POSITION_VALUE]
        elif(goal == 'open'):
            goal_pos = [DXLleft_OPEN_POSITION_VALUE,DXLright_OPEN_POSITION_VALUE]
        else:
            return 0

        # Checks if current position is within given threshold from goal position
        for i in range(2):
            if (abs(goal_pos[i] - present_pos[i]) > DXL_MOVING_STATUS_THRESHOLD):
                return 0
        return 1


    '''
    Get torque load from each servo
    @return: list(float)  [ left motor load, right motor load ], -1.0 means error
    '''
    def dxl_get_load(self):

        try:
            # Read Dynamixel gripper load
            dxlleft_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_PRESENT_LOAD)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            dxlright_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_PRESENT_LOAD)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Normalise load values
            dxlleft_load = (dxlleft_load & int('1111111111', 2)) / 1024.0
            dxlright_load = (dxlright_load & int('1111111111', 2)) / 1024.0
            
            return [dxlleft_load,dxlright_load]

        except IndexError:
            print(colored("GetLoad ERROR!!! Exception Index Error in Dynamixel SDK...", "red"))
            return [-1.0,-1.0]


    # Fix current pos of gripper, aka lock the torque (stall) on the current pos with 'moving' offset
    # ** +ve offset means close, -ve offset means open
    def dxl_fix_gripper(self, offset=0):
        
        [left_pos,right_pos] = self.dxl_get_pos()

        # Set torque on
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)

        # Write Dynamixel gripper open position to left and right motors
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, left_pos + offset)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, right_pos- offset)


















    # GRAVEYARD  +
    #          _|^|_

    # # Execute stepwise increment/decrement of joint positions based on current load, to reduce servo load
    # # TODO: currently not being used
    # def dxl_step_release(self, is_gripped):

    #     [left_load,right_load] = self.dxl_get_load()
    #     [left_pos,right_pos] = self.dxl_get_pos()

    #     #set torque on
    #     self.packetHandler.write1ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
    #     self.packetHandler.write1ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)


    #     new_left_pos = 0
    #     new_right_pos = 0

    #     if(is_gripped == True):
    #         # gradual but slow release of grip angles to maintain grip torque (using step release)
    #         if(left_load > LEFTLOAD_THRESHOLD or right_load > RIGHTLOAD_THRESHOLD):
    #             new_left_pos = left_pos-STEP_RELEASE_ANGLE
    #             print("Step releasing angle increasing")
    #             new_right_pos = right_pos+STEP_RELEASE_ANGLE

    #             # Write Dynamixel gripper step release
    #             dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, new_left_pos)
    #             if dxl_comm_result != COMM_SUCCESS:
    #                 print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    #                 print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    #             dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, new_right_pos)
    #             if dxl_comm_result != COMM_SUCCESS:
    #                 print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    #             elif dxl_error != 0:
    #                 print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    #             print("High load detected in closed pos, releasing joint angles.")


    #     elif(is_gripped == False):

    #         # gradual but slow release of grip angles to maintain grip torque (using step release)
    #         if(left_load > LEFTLOAD_THRESHOLD):
    #             new_left_pos = left_pos+STEP_RELEASE_ANGLE

    #             # Write Dynamixel gripper step release
    #             dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLleft_ID, ADDR_AX_GOAL_POSITION, new_left_pos)
    #             if dxl_comm_result != COMM_SUCCESS:
    #                 print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    #             elif dxl_error != 0:
    #                 print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    #             print("High load detected in open pos [left], releasing joint angles.")


    #         # gradual but slow release of grip angles to maintain grip torque (using step release)
    #         if(right_load > LEFTLOAD_THRESHOLD):
    #             new_right_pos = right_pos+STEP_RELEASE_ANGLE

    #             # Write Dynamixel gripper step release
    #             dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXLright_ID, ADDR_AX_GOAL_POSITION, new_right_pos)
    #             if dxl_comm_result != COMM_SUCCESS:
    #                 print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    #             elif dxl_error != 0:
    #                 print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    #             print("High load detected in open pos [right], releasing joint angles.")


        