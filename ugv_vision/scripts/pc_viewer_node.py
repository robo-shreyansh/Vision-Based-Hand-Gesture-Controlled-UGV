#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ugv_vision.vis_nodes import PC_Viewer

def main(args=None):
    rclpy.init(args= args)
    node = PC_Viewer()
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__=="__main__":
    main()