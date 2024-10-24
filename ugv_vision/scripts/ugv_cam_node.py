#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from ugv_vision.vis_nodes import UGV_Cam

def main(args = None):
    rclpy.init(args = args)
    node = UGV_Cam()
    rclpy.spin(node)
    rclpy.shutdown()
# import 

if __name__=="__main__":
    main()