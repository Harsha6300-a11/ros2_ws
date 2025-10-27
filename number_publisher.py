import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int32, 'number', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_number)
        self.number = 3  # fixed number
        self.get_logger().info('Number Publisher Node started')

    def publish_number(self):
        msg = Int32()
        msg.data = self.number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
