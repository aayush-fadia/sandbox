import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import matplotlib.pyplot as plt
from ros2_numpy import numpify
from mpl_toolkits.mplot3d import Axes3D

plt.ion()


def features_deepcopy(f):
    return [
        cv2.KeyPoint(
            x=k.pt[0],
            y=k.pt[1],
            _size=k.size,
            _angle=k.angle,
            _response=k.response,
            _octave=k.octave,
            _class_id=k.class_id,
        )
        for k in f
    ]


class VisualOdom(Node):
    def __init__(self):
        super(VisualOdom, self).__init__('imshower')
        self.cv_bridge = CvBridge()
        self.orb = cv2.ORB_create()
        self.dist_threshold = 0.5
        index_params = dict(
            algorithm=6, table_number=6, key_size=12, multi_probe_level=1
        )
        search_params = dict(checks=100)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.draw_params = dict(
            matchColor=(0, 255, 0), flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        self.trajectory = [np.array([0, 0, 0])]
        self.P = np.eye(4)
        self.prev_kp = None
        self.prev_des = None
        self.prev_rgb = None
        self.prev_depth = None
        sub_rgb = Subscriber(
            self,
            Image,
            'image',
            qos_profile=qos_profile_sensor_data
        )
        sub_depth = Subscriber(
            self,
            Image,
            'depth',
            qos_profile=qos_profile_sensor_data
        )

        sub_camera_info = Subscriber(
            self,
            CameraInfo,
            'camera_info',
            qos_profile=qos_profile_sensor_data
        )

        ts = ApproximateTimeSynchronizer([sub_rgb, sub_depth, sub_camera_info],
                                         queue_size=1000, slop=100.0)
        ts.registerCallback(self.image_listener)

    def draw_matches(self, prev_img, img, prev_kp, kp, matches):
        drawn_matches = cv2.drawMatchesKnn(
            prev_img, prev_kp, img, kp, matches, None, **self.draw_params
        )
        return drawn_matches

    def filter_matches_distance(self, match):
        filtered_match = []
        for m, n in match:
            if m.distance < self.dist_threshold * n.distance:
                filtered_match.append(m)
        return filtered_match

    def deproject(self, img, depth, u, v, K):
        u0 = img.shape[1] // 2
        v0 = img.shape[0] // 2
        fx = K[0, 0]
        fy = K[1, 1]
        x = (u - u0) * depth[v, u] / fx
        y = (v - v0) * depth[v, u] / fy
        z = depth[v, u]
        return np.asarray([x, y, z])

    def image_listener(self, msg_rgb, msg_depth, msg_camera_info):
        bgr = self.cv_bridge.imgmsg_to_cv2(msg_rgb, 'bgr8')
        depth = numpify(msg_depth)
        kp, des = self.orb.detectAndCompute(bgr, None)
        image1_points_3d = []
        image2_points = []
        if self.prev_kp is not None:
            matches = self.flann.knnMatch(self.prev_des, des, k=2)
            drawn_matches = self.draw_matches(self.prev_rgb, bgr, self.prev_kp, kp, matches)
            drawn_matches = cv2.cvtColor(drawn_matches, cv2.COLOR_BGR2RGB)
            plt.subplot(2, 1, 1).imshow(drawn_matches)
            plt.axis("off")
            K = np.asarray(msg_camera_info.k).reshape((3, 3))
            filtered_matches = self.filter_matches_distance(matches)
            for m in filtered_matches:
                i1 = m.queryIdx
                i2 = m.trainIdx
                p1_x, p1_y = self.prev_kp[i1].pt
                image1_3d_point = self.deproject(bgr, depth, int(p1_x), int(p1_y), K)
                image1_points_3d.append(image1_3d_point)
                p2_x, p2_y = kp[i2].pt
                image2_points.append([p2_x, p2_y])
            _, rvec, t, _ = cv2.solvePnPRansac(np.asarray(image1_points_3d), np.asarray(image2_points), K, None)
            R, _ = cv2.Rodrigues(rvec)
            t = np.array([t[0, 0], t[1, 0], t[2, 0]])
            P_new = np.eye(4)
            P_new[0:3, 0:3] = R.T
            P_new[0:3, 3] = (-R.T).dot(t)
            self.P = self.P.dot(P_new)
            self.trajectory.append(self.P[:3, 3])
            print(self.P[:3, 3])
            xyz = [[x[i] for x in self.trajectory] for i in range(3)]
            print(np.asarray(self.trajectory).shape)
            plt.subplot(2, 1, 2, projection='3d').plot(xyz[0], xyz[1], zs=xyz[2])
            plt.show()
            plt.pause(0.01)
        self.prev_rgb = bgr.copy()
        self.prev_depth = depth.copy()
        self.prev_kp = features_deepcopy(kp)
        self.prev_des = des.copy()


def main(args=None):
    rclpy.init(args=args)
    imshower = VisualOdom()
    rclpy.spin(imshower)

    imshower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
