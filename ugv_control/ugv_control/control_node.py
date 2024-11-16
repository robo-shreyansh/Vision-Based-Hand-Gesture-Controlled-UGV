#! /usr/bin/env python3

import rclpy
import rclpy 
from rclpy.node import Node
import cmd as cmd
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

from ugv_control.ugv import ugv

PIN_LEFT = 7 
PIN_RIGHT = 12
MAX_MOTOR_SPEED = 12 ## CHANGE
GPIO.setmode(GPIO.BOARD)

class UGV_Control(Node):

    def __init__(self, pin_left=PIN_LEFT, pin_right=PIN_RIGHT):
        super().__init__("ugv_controller")
        self.left_pin = pin_left
        self.right_pin = pin_right
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.ugv = ugv()

        # setup the pins to output, set dc = 0, start(0)

    # how about i create a custom message over a custom topic. 
    # FAST AHEAD (make dc 1 for both), AHEAD (make dc = 0.5), ROTATE CW (make dc = 0.5 for one, zero for other), ROTATE ACW, 
    # NO dont write custom message. 

    # ugv_control

    def cmd_vel_callback(self, msg):
        fwd_vel = msg.linear.x
        ang_vel = msg.angular.z
        self.ugv.move(fwd_vel, ang_vel)

def main(args=None):
    rclpy.init(args= args)
    node = UGV_Control()
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__=="__main__":
    main()