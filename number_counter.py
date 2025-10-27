import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class NumberCounter(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Int32, 'number_sum', 10)
        self.counter = 0
        self.get_logger().info('Number Counter Node started')

    def listener_callback(self, msg):
        self.counter += msg.data
        out_msg = Int32()
        out_msg.data = self.counter
        self.publisher_.publish(out_msg)
        self.get_logger().info(f'Current Sum: {out_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
