#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'hello', 10)
        self.timer = self.create_timer(1, self.timer_callback)
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
class Subscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        print(msg.data)
def main(args=None):
    rclpy.init(args=args)
    # publisher = Publisher()
    subscriber = Subscriber() 
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()        # self.get_logger().info("Param simple_int_param has been declared with value %d" % self.get_parameter("simple_int_param").value)
        # self.get_logger().info("Node %s has been started" % self.get_name())
