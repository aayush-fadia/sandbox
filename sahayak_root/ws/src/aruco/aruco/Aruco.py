import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from transforms3d import quaternions


class Aruco(Node):
    def __init__(self):
        super(Aruco, self).__init__('aruco')
        self.cv_bridge = CvBridge()
        self.tf_broadcast = tf2_ros.TransformBroadcaster(self)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        sub_rgb = Subscriber(
            self,
            Image,
            'image',
            qos_profile=qos_profile_sensor_data
        )
        sub_camera_info = Subscriber(
            self,
            CameraInfo,
            'camera_info',
            qos_profile=qos_profile_sensor_data
        )

        ts = ApproximateTimeSynchronizer([sub_rgb, sub_camera_info],
                                         queue_size=1000, slop=100.0)
        ts.registerCallback(self.image_listener)

        self.publisher_ = self.create_publisher(PoseStamped, 'pose', qos_profile_system_default)

    def image_listener(self, msg_rgb, msg_camera_info):
        bgr = self.cv_bridge.imgmsg_to_cv2(msg_rgb, 'bgr8')
        (corners, ids, rejected) = cv2.aruco.detectMarkers(bgr, self.aruco_dict)
        if len(corners) > 0:
            bgr = cv2.aruco.drawDetectedMarkers(bgr, corners)
            K = np.asarray(msg_camera_info.k).reshape((3, 3))
            t = TransformStamped()
            p = PoseStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "marker_0"
            t.child_frame_id = "robot_per_marker_0"
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 1, K, np.asarray([0, 0, 0, 0]))
            for rv, tv in zip(rvecs, tvecs):
                cv2.aruco.drawAxis(bgr, K, np.zeros((4, 1)), rv, tv, 0.6)
            print(rvecs)
            rvecs = -1 * rvecs
            tvecs = -1 * tvecs
            p.header.frame_id = "marker_0"
            t.transform.translation.x = tvecs[0, 0, 0]
            p.pose.position.x = tvecs[0, 0, 0]
            t.transform.translation.y = tvecs[0, 0, 1]
            p.pose.position.x = tvecs[0, 0, 1]
            t.transform.translation.z = tvecs[0, 0, 2]
            p.pose.position.x = tvecs[0, 0, 2]
            rotation_matrix = np.array([[0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0, 0, 0, 1]],
                                       dtype=float)
            rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvecs[0, 0])
            q = quaternions.mat2quat(cv2.Rodrigues(rvecs[0, 0])[0])
            t.transform.rotation.x = q[0]
            p.pose.orientation.x = q[0]
            t.transform.rotation.y = q[1]
            p.pose.orientation.y = q[1]
            t.transform.rotation.z = q[2]
            p.pose.orientation.z = q[2]
            t.transform.rotation.w = q[3]
            p.pose.orientation.w = q[3]
            self.tf_broadcast.sendTransform(t)
            self.publisher_.publish(p)
        cv2.imshow('aruco', bgr)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    imshower = Aruco()
    rclpy.spin(imshower)

    imshower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
