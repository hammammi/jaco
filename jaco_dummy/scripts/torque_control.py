#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Hyemi Jeong

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example :
# All models using Protocol 1.0
# DXL MX-64, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#
import rospy
import os

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
import time

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from jaco_dummy.msg import modechange
from jaco_dummy.msg import angle
from std_msgs.msg import Int8
# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_TCME               = 70
ADDR_MX_GOAL_TORQUE        = 71               # write in bite

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID0                      = 0                 # Dynamixel ID setting
DXL_ID1                      = 1
DXL_ID2                      = 2
DXL_ID3                      = 3
DXL_ID4                      = 4
DXL_ID5                      = 5
DXL_ID                       = [0,1,2,3,4,5]
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

dxl_home_position = [1024, 2048, 0, 2560, 4095, 0]         # Home position
dxl_present_position = [0,0,0,0,0,0]



# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)



# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque



    # time.sleep(1)



def callback(msg):
    # rospy.loginfo(msg)
    global jn
    jn = msg.data



def readposition():
    global jn
    jn = 10
    count = 0

    for i in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_TCME, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        else:
            print("Dynamixel %03d has been successfully connected" % i )

    rospy.init_node('get_joint_angle')
    pub = rospy.Publisher('joint_angle',angle,queue_size=10)
    rospy.Subscriber('command',Int8,callback)
    r = 1  # Hz
    rate = rospy.Rate(r)


    while not rospy.is_shutdown():
        # Read present position
        for i in DXL_ID:
            dxl_present_position[i], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_PRESENT_POSITION)
            dxl_present_position[i] = dxl_present_position[i] - dxl_home_position[i]
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

        # print("[Present position] joint#0 %03d joint#1 %03d joint#2 %03d joint#3 %03d joint#4 %03d joint#5 %03d " % (dxl_present_position[0], dxl_present_position[1], dxl_present_position[2], dxl_present_position[3], dxl_present_position[4],dxl_present_position[5]))
        joints = angle()
        joints.joint0 = dxl_present_position[0] * 0.088
        joints.joint1 = dxl_present_position[1] * 0.088
        joints.joint2 = dxl_present_position[2] * 0.088
        joints.joint3 = dxl_present_position[3] * 0.088
        joints.joint4 = dxl_present_position[4] * 0.088
        joints.joint5 = dxl_present_position[5] * 0.088
        # rospy.loginfo("[Present position] joint#0 %03d joint#1 %03d joint#2 %03d joint#3 %03d joint#4 %03d joint#5 %03d " % (dxl_present_position[0], dxl_present_position[1], dxl_present_position[2], dxl_present_position[3], dxl_present_position[4],dxl_present_position[5]))
        pub.publish(joints)
        rospy.loginfo("[Present position] joint#0 %03d joint#1 %03d joint#2 %03d joint#3 %03d joint#4 %03d joint#5 %03d " % (joints.joint0, joints.joint1, joints.joint2, joints.joint3, joints.joint4, joints.joint5))
        rospy.loginfo("%d %d"%(jn, count))
        if jn != 10 and jn != 6:
            if count == 0 :
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,jn, ADDR_MX_TCME,TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,jn, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                dxl_torque_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, jn, ADDR_MX_TCME)

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                count = count + 1
                print("[ID:%03d] Mode %03d" % (jn, dxl_torque_mode))


            elif count/r == 5.0:
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,jn, ADDR_MX_TCME, TORQUE_DISABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,jn, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                jn = 10
                count = 0
            else:
                count = count + 1

        elif jn == 6 :
            if count == 0 :
                for i in DXL_ID:
                    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,DXL_ID[i], ADDR_MX_TCME,TORQUE_ENABLE)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                    dxl_torque_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_TCME)

                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))

                    print("[ID:%03d] Mode %03d" % (DXL_ID[i], dxl_torque_mode))
                count = count + 1

            elif count/r == 5.0:
                for i in DXL_ID:
                    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,DXL_ID[i], ADDR_MX_TCME, TORQUE_DISABLE)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))

                    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                jn = 10
                count = 0
            else:
                count = count + 1


        rate.sleep()
    # rospy.spin()

# def changemode():
#     rospy.Subscriber('command',modechange,callback)
#     rospy.spin()

if __name__== '__main__':

    try:
        readposition()
        # changemode()

    except rospy.ROSInterruptException: pass




# Close port
portHandler.closePort()
