#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class my_subscriber(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.create_pub_ = self.create_publisher(Twist , "/turtle1/cmd_vel" ,10)
        self.create_sub_ = self.create_subscription( Pose ,"/turtle1/pose",self.pose_callback,10)
        self.get_logger().info("Turtle has been started ")

    def pose_callback(self , msg= Pose):

        cmd =Twist()
        if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.6
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        self.create_pub_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node=my_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()