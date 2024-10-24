#!/usr/bin/env python3

from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class PC_Viewer(Node):
    def __init__(self):
        super().__init__("pc_viewer")
        self.cv_bridge = CvBridge()
        self.img_subscription = self.create_subscription(Image, "/img_data", self.img_sub_callback, 10)

    def img_sub_callback(self, data : Image):
        img_frame = self.cv_bridge.imgmsg_to_cv2(data)
        cv2.imshow("Img Feed", img_frame)
        cv2.waitKey(1)


class UGV_Cam (Node):
    def __init__(self):
        super().__init__("ugv_cam")

        self.img_pub = self.create_publisher(Image, "/img_data", 10)
        self.create_timer(0.001, self.img_callback_pub)
        self.cv_bridge = CvBridge()
        self.cam = cv2.VideoCapture(0)
    
    def img_callback_pub(self):
        ret, frame = self.cam.read()
        imgmsg = self.cv_bridge.cv2_to_imgmsg(frame)
        self.img_pub.publish(imgmsg)

