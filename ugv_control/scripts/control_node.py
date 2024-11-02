#! /usr/bin/env python3

import rclpy
from ugv_control.ugv import UGV_Control

def main(args=None):
    rclpy.init(args= args)
    node = UGV_Control()
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__=="__main__":
    main()