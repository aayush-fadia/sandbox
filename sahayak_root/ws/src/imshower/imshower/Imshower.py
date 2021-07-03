import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2


class Imshower(Node):
    def __init__(self):
        super().__init__('imshower')
        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.listener_callback,
            qos_profile_sensor_data)

    def listener_callback(self, msg):
        img_np = self.cv_bridge.imgmsg_to_cv2(msg)
        cv2.imshow('img', img_np)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Imshower()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
