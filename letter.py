#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class DrawLetter(Node):
    def __init__(self):
        super().__init__("draw_H")
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.pose = None

        
        self.waypoints = [
            (5.4, 5.4), (8.4, 5.4), (8.4, 8.4), (8.4, 2.4),
            (8.4, 5.4), (5.4, 5.4), (2.4, 5.4), (2.4, 8.4),
            (2.4, 2.4), (2.4, 5.4), (5.4, 5.4)
        ]
        self.index = 0

        # thresholds / fixed speeds
        self.reached_threshold = 0.05
        self.angle_tolerance = 0.05

        self.linear_speed = 1.2
        self.angular_speed = 1.8

        self.state = 'rotate'

        # control loop at 20 Hz
        self.timer = self.create_timer(1.0 / 20.0, self.control_loop)
        self.get_logger().info(f"Waiting for /turtle1/pose... {len(self.waypoints)} waypoints loaded")

    def pose_callback(self, msg: Pose):
        self.pose = msg

    def normalize_angle(self, ang):
        # normalize to [-pi, pi]
        while ang > math.pi:
            ang -= 2.0 * math.pi
        while ang < -math.pi:
            ang += 2.0 * math.pi
        return ang

    def control_loop(self):
        if self.pose is None:
            return

        # finished
        if self.index >= len(self.waypoints):
            # ensure stopped
            self.pub.publish(Twist())
            self.get_logger().info("All waypoints visited. Stopping.")
            self.timer.cancel()
            return

        goal_x, goal_y = self.waypoints[self.index]
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.pose.theta)

        twist = Twist()

        # reached current waypoint?
        if distance < self.reached_threshold:
            self.pub.publish(Twist())  # stop briefly
            self.get_logger().info(f"Reached waypoint {self.index+1}/{len(self.waypoints)}: ({goal_x:.2f}, {goal_y:.2f})")
            self.index += 1
            if self.index < len(self.waypoints):
                self.state = 'rotate'
            return

        # rotate-in-place state
        if self.state == 'rotate':
            direction = 1.0 if angle_error > 0.0 else -1.0
            twist.angular.z = direction * self.angular_speed
            twist.linear.x = 0.0

            # if aligned, stop and switch to move
            if abs(angle_error) < self.angle_tolerance:
                self.pub.publish(Twist())
                self.state = 'move'
                return

        # move-straight state (fixed speed)
        elif self.state == 'move':
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

            # if heading drifts too much, go back to rotate
            if abs(angle_error) > (self.angle_tolerance * 1.5):
                self.pub.publish(Twist())
                self.state = 'rotate'
                return

        # publish command
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DrawLetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
