#! /usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class UGV_Cam (Node):
    def __init__(self):
        super().__init__("ugv_cam")

        self.img_pub = self.create_publisher(Image, "/img_data", 10)
        self.create_timer(0.1, self.img_callback_pub)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cv_bridge = CvBridge()
        self.cam = cv2.VideoCapture(0)
    
    def img_callback_pub(self):
        ret, frame = self.cam.read()
        frame = cv2.resize(frame, (255,255))
        imgmsg = self.cv_bridge.cv2_to_imgmsg(frame)
        self.img_pub.publish(imgmsg)


def main(args = None):
    rclpy.init(args = args)
    node = UGV_Cam()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__=="__main__":
    main()