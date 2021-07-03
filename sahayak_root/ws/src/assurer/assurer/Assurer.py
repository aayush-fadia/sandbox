import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class Assurer(Node):
    def __init__(self):
        super().__init__('assurer')
        self.subscription = self.create_subscription(
            Image,
            'unreliable',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Image, 'reliable', qos_profile_system_default)

    def listener_callback(self, msg):
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Assurer()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
