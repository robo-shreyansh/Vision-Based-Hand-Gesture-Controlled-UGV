#! /usr/bin/env python3

import rclpy 
from rclpy.node import Node
import cmd as cmd
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO


#CHANGE
PIN_LEFT = 7 
PIN_RIGHT = 12
MAX_MOTOR_SPEED = 12 ## CHANGE
GPIO.setmode(GPIO.BOARD)


def sign(z):
    if z!=0:
        return abs(z)/z
    else:
        return 0

def saturate(speed):
    
    if speed<0:
        # add a flag to return 'reverse'
        return 0
    elif speed>=MAX_MOTOR_SPEED:
        return MAX_MOTOR_SPEED
    return speed

class wheel():
    def __init__(self, pin_num):
        self.pin_num = pin_num
        

    def set_dc(self, dc):
        self.motor.ChangeDutyCycle(abs(dc))

    def initialize(self):
        GPIO.setup(self.pin_num, GPIO.OUT)
        self.motor = GPIO.PWM(self.pin_num, 100)
        self.motor.start(0)


class ugv():
    def __init__(self):
        
        self.left_wheel = wheel(PIN_LEFT)
        self.right_wheel = wheel(PIN_RIGHT)
        self.left_wheel.initialize()
        self.right_wheel.initialize()
        self._fwd_ = 0
        self._angular_ = 0
        

    def move(self, _fwd_, _angular_):

        # Maybe i have complicated it unnecessarily. Just reverse if the wheel speed is negative, so one more pin for reverse
        self._fwd_ += _fwd_
        self._angular_ += _angular_

        # self._fwd_ = self._fwd_ if self._fwd_<MAX_MOTOR_SPEED else MAX_MOTOR_SPEED # REMOVE ABS add reverse
        # self._fwd_ = 0 if self._fwd_<0 else self._fwd_

        # self._angular_ = self._angular_ if self._angular_<MAX_MOTOR_SPEED else MAX_MOTOR_SPEED # REMOVE ABS add reverse
        # self._angular_ = 0 if self._angular_<0 else self._angular_
        
        self.left_wheel_speed = saturate(self._fwd_ + self._angular_)#*sign(_angular_))
        self.right_wheel_speed = saturate(self._fwd_ + self._angular_)#*(-sign(_angular_)))

        ## Maybe tune or saturate to get the ideal dc. this will depend on the fully loaded bot. 
        ## figure out taking reverse! 

        self.left_wheel.set_dc(100*self.left_wheel_speed/MAX_MOTOR_SPEED)
        self.right_wheel.set_dc(100*self.right_wheel_speed/MAX_MOTOR_SPEED)



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