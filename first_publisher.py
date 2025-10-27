#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class draw_circle(Node):
    def __init__(self):
        super().__init__("first_publisher")
        self.cmd_vel_ = self.create_publisher(Twist, "/turtle1/cmd_vel" , 10 ) 
        self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info("circle has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.1
        self.cmd_vel_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node=draw_circle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__' :
    main()