#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node 
 

class Mynode(Node):

    def __init__(self):
        super().__init__("loop_node")
        self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info("Amma loves me")

def main(args=None):
    rclpy.init(args=args)
    node=Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== '__main__':
    main()
