"""
Find the 3D pose of a human hand using computer vision.

Using cv_bridge, take messages from the /image_raw topic and convert them
into OpenCV images for mediapipe to use to figure out where a user's hand
is located. Afterwards, figure out the 3D locations of the hand's joints
generated by mediapipe using the depth_image_proc package.

SUBSCRIBERS:
  + /image_raw (Image) - Raw image data from the usb camera.

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge, CvBridgeError
from .mediapipehelper import MediaPipeRos as mps

import mediapipe as mp
import numpy as np
import cv2 as cv


class HandCV(Node):
    def __init__(self):
        super().__init__("handcv")

        # initialize CvBridge object
        self.bridge = CvBridge()

        # initialize MediaPipe Object
        self.mps = mps()

        # create callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()

        # create timer
        self.timer = self.create_timer(
            1/30, self.timer_callback, callback_group=self.timer_callback_group)

        # create subscribers
        self.color_image_raw_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_image_raw_callback, 10)

        self.depth_image_raw_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_image_raw_callback)

        # create publishers
        self.cv_image_pub = self.create_publisher(Image, 'cv_image', 10)
        self.marker_pub = self.create_publisher(
            Marker, 'visualization_marker', 10)
        self.waypoint_pub = self.create_publisher(
                PoseStamped, 'waypoint', 10)

        # intialize other variables
        self.color_image = None
        self.depth_image = None
        self.waypoint = PoseStamped() 
        self.waypoint.pose.orientation.x = 1
        self.waypoint.pose.orientation.w = 0

    def depth_image_raw_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")
        self.depth_image = cv.flip(self.depth_image, 1)

    def color_image_raw_callback(self, msg):
        """Capture messages published on the /image_raw topic, and convert them to OpenCV images."""
        self.color_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")
        self.color_image = cv.flip(self.color_image, 1)

    def process_depth_image(self, annotated_image=None, detection_result=None):
        # first package the data into numpy arrays
        if len(detection_result.hand_landmarks) == 1:
            coords = np.array([[landmark.x * np.shape(annotated_image)[1],
                                landmark.y * np.shape(annotated_image)[0]]
                               for landmark in [detection_result.hand_landmarks[0][0],
                                                detection_result.hand_landmarks[0][1],
                                                detection_result.hand_landmarks[0][2],
                                                detection_result.hand_landmarks[0][5],
                                                detection_result.hand_landmarks[0][9],
                                                detection_result.hand_landmarks[0][14],
                                                detection_result.hand_landmarks[0][17]]])
            # now perform the math on the numpy arrays. I think this is faster?
            length = coords.shape[0]
            sum_x = np.sum(coords[:, 0])
            sum_y = np.sum(coords[:, 1])
            centroid = np.array([sum_x/length, sum_y/length])
            centroid = np.append(
                centroid, self.depth_image[int(centroid[1]), int(centroid[0])])

            self.waypoint.pose.position.x = centroid[0]
            self.waypoint.pose.position.y = centroid[1]
            self.waypoint.pose.position.z = centroid[2]

            text = f"(x: {np.round(centroid[0])}, y: {np.round(centroid[1])}, z: {np.round(centroid[2])})"

            annotated_image = cv.putText(annotated_image, text,
                                         (int(centroid[0])-100,
                                          int(centroid[1])+40),
                                         cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

            annotated_image = cv.circle(
                annotated_image, (int(centroid[0]), int(centroid[1])), 10, (255, 255, 255), -1)

        cv_image = self.bridge.cv2_to_imgmsg(
            annotated_image, encoding="rgb8")

        return cv_image, centroid

    def process_color_image(self):
        try:

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB, data=self.color_image)

            detection_result = self.mps.landmarker.detect(mp_image)
            annotated_image = self.mps.draw_landmarks_on_image(
                rgb_image=self.color_image, detection_result=detection_result)

            return annotated_image, detection_result

        except CvBridgeError:
            self.get_logger().error(CvBridgeError)

    def timer_callback(self):
        if self.color_image is not None and self.depth_image is not None:
            annotated_image, detection_result = self.process_color_image()
            cv_image, _ = self.process_depth_image(
                annotated_image, detection_result)
            self.cv_image_pub.publish(cv_image)
        
        # publish the waypoint
        self.waypoint.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(self.waypoint)


def main(args=None):
    rclpy.init(args=args)

    handcv = HandCV()

    rclpy.spin(handcv)


if __name__ == '__main__':
    main()
