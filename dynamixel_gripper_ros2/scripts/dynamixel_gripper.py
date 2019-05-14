#!/usr/bin/env python3
# license removed for brevity
# This scripts runs a node that opens and closes the dyanmixel gripper via ros topics
#
# Created by Poh Yong Keat 2018, , Tan You Liang
# Maintained by Tan You Liang 2019 (RMF Project)


import rclpy
import time
import os

from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from dynamixel_gripper_ros2.msg import GripState

from termcolor import colored
from dxl_controller import DXL_Controller



TIMER_CALLBACK_INTERVAL = 0.25   # periodic pub interval
SKIP_PUB_FREQ = 4               # skip pub in timer callback

GRIPPER_STATUS_REACTION_TIME = 3  #in seconds
GRIPPER_LEFT_OPEN_POS        = int(1.3355 * (1023/5.2333))           # Dynamixel will rotate between these values
GRIPPER_LEFT_CLOSE_POS       = int(2.2717 * (1023/5.2333))
GRIPPER_RIGHT_OPEN_POS        = int(3.8009 * (1023/5.2333))           # Dynamixel will rotate between these values
GRIPPER_RIGHT_CLOSE_POS      = int(2.9164 * (1023/5.2333))
GRIPPER_SPEED                = 60                                    # Moving speed set to slow



################################################################################################################################
################################################################################################################################



# Gripper ROS2 Node class (publishes to /gripper/grip_state and subscribes to /gripper/grip_command)
class DXL_Controller_ROS(Node):

    def __init__(self):
        super().__init__('dynamixel_gripper')

        self.get_logger().info(" Starting Dynamixel Gripper ROS2 Node ")
        self.is_gripped = False
        self.pubState = self.create_publisher(GripState, '/gripper/grip_state')
        self.subCmd = self.create_subscription(Bool,'/gripper/grip_command',self.command_callback)
        self.tmr = self.create_timer(TIMER_CALLBACK_INTERVAL, self.timer_callback)

        # initialise dynamixel connection
        self.dxl_gripper =  DXL_Controller()
        self.dxl_gripper.dxl_open( GRIPPER_LEFT_OPEN_POS, GRIPPER_RIGHT_OPEN_POS, GRIPPER_SPEED )
        self.last_grip_state = False
        self.new_command = False
        self.command_time = 0 #TODO
        self.pub_count = 0


    # Periodic callback
    def timer_callback(self):

        # Report open/close state, position, temperature & load of motors
        [left_load,right_load] = self.dxl_gripper.dxl_get_load() # TODO: theres error here
        [left_pos,right_pos] = self.dxl_gripper.dxl_get_pos()


        if(self.pub_count >= SKIP_PUB_FREQ):
            state = GripState()

            # TODO: Temp solution for handling change of gripper state
            if ( (time.time()-self.command_time > GRIPPER_STATUS_REACTION_TIME) and (self.new_command==True) ):
                print( colored("Pub new GripState: {}; to ROS2 Topic".format(self.is_gripped), "yellow") )
                self.dxl_gripper.dxl_fix_gripper(offset = 4 )
                self.last_grip_state = self.is_gripped
                self.new_command = False
            
            state.is_gripped = self.last_grip_state
            state.left_pos = float(left_pos)
            state.right_pos = float(right_pos)
            state.load_left = float(left_load)
            state.load_right = float(right_load)
            state.avg_load = (left_load+right_load)/2.0
            state.avg_temp = sum(self.dxl_gripper.dxl_get_temp())/2

            self.pubState.publish(state)
            self.pub_count = 0
        
        else:
            self.pub_count = self.pub_count + 1



    # Respond to /gripper/grip_command topic
    def command_callback(self,msg):
        gripCommand = msg.data
        state = GripState()
        self.get_logger().info(colored("Received Grip Command: %s from client" %gripCommand, "white", "on_green" ))
        
        self.new_command = True
        self.command_time = time.time()
        self.last_grip_state = self.is_gripped

        if (self.is_gripped != gripCommand):
            # Close gripper
            if(gripCommand == True):
                self.dxl_gripper.dxl_close(GRIPPER_LEFT_CLOSE_POS, GRIPPER_RIGHT_CLOSE_POS, GRIPPER_SPEED)
                self.get_logger().info('Closing gripper...')
                self.is_gripped = True

            # Open gripper
            elif(gripCommand == False):
                self.dxl_gripper.dxl_open(GRIPPER_LEFT_OPEN_POS, GRIPPER_RIGHT_OPEN_POS, GRIPPER_SPEED)
                self.get_logger().info('Opening gripper...')
                self.is_gripped = False

            else:
                self.get_logger().info( colored('Invalid gripper command: {}'.format(gripCommand), "red") )
        else:
            self.get_logger().info("Gripper reached grip's state")



################################################################################################################################
################################################################################################################################


def main(args=None):

    # initialise new gripper node
    rclpy.init(args=args)
    node = DXL_Controller_ROS()
    # Spin continuously to publish gripper state
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()