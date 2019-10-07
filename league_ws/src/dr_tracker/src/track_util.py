#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import time
import tf


class TrackUtil():

    def __init__(self, debug_mode=True, temporal_filtering=2, line_contour_cutoff=100, cam_dist_from_ground=1,
                 camera_vertical_fov=80, camera_horizontal_fov=120, track_line_lower=None,
                 track_line_upper=None, blur_param=9, refresh_rate=0):
        if track_line_lower is None:
            track_line_lower = [0, 0, 0]
        if track_line_upper is None:
            track_line_upper = [180, 50, 255]

        self.debug = debug_mode
        self.temporal_filtering = temporal_filtering
        self.line_contour_cutoff = line_contour_cutoff
        self.cam_dist_from_groun = cam_dist_from_ground
        self.camera_vertical_fov = camera_vertical_fov
        self.camera_horizontal_fov = camera_horizontal_fov
        self.track_line_lower = track_line_lower
        self.track_line_upper = track_line_upper
        self.blur_param = blur_param
        self.refresh_rate = refresh_rate

        self.last_image_time = 0
        self.cv_image_filtered = None
        self.cv_image_raw = None
        self.camera_resolution = None
        self.parsed_img = None
        self.bridge = CvBridge()

        self.timer = rospy.Timer(rospy.Duration(0.1), self.is_off_track, oneshot=False)

    def process_image(self, image):  # callback for ROS subscriber with Image message
        """Takes in image from ROS and converts it to open CV form. Begins the process of analyzing the track, the deepracer, etc."""
        # process images at specified rate
        if time.time() - self.last_image_time > self.refresh_rate:
            if self.refresh_rate == 0 and self.cv_image_raw is not None:
                pass
            else:
                try:
                    self.cv_image_raw = self.bridge.imgmsg_to_cv2(image, "bgr8")
                    if self.camera_resolution is None:
                        self.camera_resolution = np.array((self.cv_image_raw.shape[1], self.cv_image_raw.shape[0]))
                    self.cv_image_filtered = cv2.medianBlur(self.cv_image_raw, 9)
                    self.parse_track()
                except CvBridgeError as e:
                    print(e)
                self.last_image_time = time.time()
        if self.debug and self.cv_image_raw is not None:
            cv2.imshow("Raw image window", self.cv_image_raw)
            cv2.imshow("Filtered image window", self.cv_image_filtered)
            cv2.imshow("parsed image window", cv2.bitwise_and(self.cv_image_raw, self.cv_image_raw, mask=self.parsed_img))
            # cv2.imshow("Filtered and parsed", np.hstack([self.cv_image_filtered, self.parsed_img]))
            cv2.waitKey(50)

    def parse_track(self):
        if self.cv_image_filtered is not None:  # make sure that a frame exists
            # create NumPy arrays from the boundaries
            hsv = cv2.cvtColor(self.cv_image_filtered, cv2.COLOR_BGR2HSV)  # convert to hsv for easier color isolation

            # find the colors within the specified boundaries and apply
            mask = cv2.inRange(hsv, np.array(self.track_line_lower), np.array(self.track_line_upper))
            eroded_mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
            mask1 = np.zeros(eroded_mask.shape, dtype=np.uint8)  # outer + inner track
            mask2 = np.zeros(eroded_mask.shape, dtype=np.uint8)  # inner track
            _, contours, _ = cv2.findContours(eroded_mask, 1, 2)  # find contour in mask for white lines
            center_point = (480, 100)  # center for revolving lines around to fill track
            cnts = []
            for i, cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if self.line_contour_cutoff < area < 1500:
                    cnts.append(i)
                    if self.debug:
                        print("Area of contour " + str(i) + ": " + str(area))

            if len(cnts) > 1:
                cv2.drawContours(mask1, contours, cnts[0], 255, -1)  # Draw filled contour in mask
                for x in range(mask1.shape[0]):
                    for y in range(mask1.shape[1]):
                        if mask1[x, y] > 200:
                            cv2.line(mask1, (y, x), center_point, 150, 5)

                cv2.drawContours(mask2, contours, cnts[1], 255, -1)  # Draw filled contour in mask
                for x in range(mask2.shape[0]):
                    for y in range(mask2.shape[1]):
                        if mask2[x, y] > 200:
                            cv2.line(mask2, (y, x), center_point, 150, 5)
                if self.debug:
                    cv2.circle(mask2, center_point, 5, 200, thickness=10)
                    cv2.circle(mask1, center_point, 5, 200, thickness=10)
                self.parsed_img = cv2.bitwise_xor(mask1, mask2)
            else:
                self.cv_image_raw = None  # reload an image due to inadequacy of original

    def is_off_track(self, event):
        pass


if __name__ == "__main__":
    rospy.init_node("track_util")

    # ros parameters for setting up the dr tracking node
    debug_param = rospy.get_param('~debug', False)
    temporal_filtering_param = rospy.get_param('~temporal_filtering', 2)
    line_contour_cutoff = rospy.get_param("~line_contour_cutoff", 100)
    cam_dist_from_ground_param = rospy.get_param("~camera_dist_from_ground", 1)
    camera_vertical_fov_param = rospy.get_param("~camera_vertical_fov", 80)
    camera_horizontal_fov_param = rospy.get_param("~camera_horizontal_fov", 120)
    blur_param = rospy.get_param("~blur_param", 9)
    input_camera_topic_param = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
    white_mask = map(int, rospy.get_param("~white_mask", "0, 0, 0, 180, 50, 255").split(','))
    refresh_param = rospy.get_param('~refresh_rate', 10)

    track = TrackUtil(debug_param, temporal_filtering_param, line_contour_cutoff,
                      cam_dist_from_ground_param, camera_vertical_fov_param, camera_horizontal_fov_param,
                      blur_param=blur_param, refresh_rate=refresh_param,
                      track_line_lower=[white_mask[0], white_mask[1], white_mask[2]],
                      track_line_upper=[white_mask[3], white_mask[4], white_mask[5]])

    rospy.Subscriber(input_camera_topic_param, Image, track.process_image)
    rospy.spin()
