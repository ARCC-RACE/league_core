#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading
import math
from geometry_msgs.msg import Vector3
import time


def dist(x1, y1, x2, y2):
    return math.sqrt(float((y2 - y1) ** 2 + (x2 - x1) ** 2))


def midpoint(x1, y1, x2, y2):
    return (x2 + x1) / 2.0, (y2 + y1) / 2.0


class TrackDR:
    def __init__(self, display_windows=True, temporal_filtering=2, dr_contour_area_cutoff=50, cam_dist_from_ground=1,
                 camera_vertical_fov=80, camera_horizontal_fov=120):
        self.acceleration = None
        self.image_pub = rospy.Publisher("dr_tracker/image_raw", Image, queue_size=1)
        # self.location_vector_pub = rospy.Publish("dr_tracker/location_vector", Vector3, queue_size=1)
        self.bridge = CvBridge()
        self.display_windows = display_windows
        self.cv_image_raw = None
        self.cv_image_filtered = None  # Image with blur or median applied. This will be better for processing contours
        self.dr_centroid = None
        self.dr_box = None  # the bounding box the of DR triangle for determining orientation vector
        self.dr_contour_area_cutoff = dr_contour_area_cutoff
        # self.last_bounding_rect = None
        # self.last_centroid = None
        self.temporal_filtering = temporal_filtering  # the number of dr_location vectors to average (0 = no filtering)
        self.dr_location_vector = np.zeros((self.temporal_filtering + 1, 2,
                                            2))  # two points that represent the orientation and location fo the DeepRacer (first index is front)
        # final state variables for the DeepRacer
        self.dr_velocity = 0.0
        self.dr_acceleration = 0.0
        self.dr_position = 0.0
        self.dr_heading = 0.0

        self.last_update_time = time.time()  # used for finding dx/dt, dv/dt

        # variables for holding information about the cameras in order to compute the velocity and absolute position of the car
        # units = meters, degrees, seconds
        self.dr_height = 0.17
        self.cam_dist_from_ground = cam_dist_from_ground
        self.camera_fov = np.array((camera_horizontal_fov, camera_vertical_fov))
        self.camera_resolution = None

    def process_image(self, image):
        """Takes in image from ROS and converts it to open CV form. Begins the process of analyzing the track, the deepracer, etc."""
        try:
            self.cv_image_raw = self.bridge.imgmsg_to_cv2(image, "bgr8")
            # print(self.cv_image_raw.shape)
            if self.camera_resolution is None:
                self.camera_resolution = np.array((self.cv_image_raw.shape[1], self.cv_image_raw.shape[0]))
        except CvBridgeError as e:
            print(e)
        if self.display_windows:
            cv2.imshow("Raw image window", self.cv_image_raw)
        self.filter_image()
        self.detect_dr_arrow()
        self.compute_state()
        cv2.waitKey(3)

    def filter_image(self):
        # median = cv2.medianBlur(image, 9)
        # blur = cv2.bilateralFilter(image, 9, 75, 75)
        blur = cv2.GaussianBlur(self.cv_image_raw, (15, 15), 0)
        # if self.display_windows:
        #     cv2.imshow("Filter image window", blur)
        #     # cv2.imshow("Blurred image window", blur)
        self.cv_image_filtered = blur

    def detect_dr_arrow(self):
        """Detects the red arrow that indicates the location and direction of the Deepracer"""
        if self.cv_image_filtered is not None:  # make sure that a frame exists
            # create NumPy arrays from the boundaries
            hsv = cv2.cvtColor(self.cv_image_filtered, cv2.COLOR_BGR2HSV)  # convert to hsv for easier color isolation
            low_red = np.array([170, 50, 90])  # lower bound for red mask
            high_red = np.array([200, 255, 255])  # upper bound for red mask

            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(hsv, low_red, high_red)  # create a mask that only looks at the red arrow
            _, contours, _ = cv2.findContours(mask, 1,
                                              2)  # find contour in mask to mark the area, bounding box, and centroid
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > self.dr_contour_area_cutoff:  # make sure that the contour is significant (filter out some noise if it exists)
                    # Find the centroid for the arrow contour on the DeepRacer
                    moments = cv2.moments(cnt)
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    centroid = (cx, cy)
                    # average out the centroid to minimize noise
                    # if self.last_centroid is not None:
                    #     centroid = (
                    #         (centroid[0] + self.last_centroid[0]) / 2.0, (centroid[1] + self.last_centroid[1]) / 2.0)
                    self.dr_centroid = (int(centroid[0]), int(centroid[1]))
                    # self.last_centroid = self.dr_centroid

                    # Find the minimum area bounding rectangle for the DeepRacer arrow
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    # average the bounding rectangle with an last one to remove more noise
                    # if self.last_bounding_rect is not None:
                    #     box = (box + self.last_bounding_rect) / 2.0
                    # self.last_bounding_rect = box
                    self.dr_box = np.int0(box)

                    # Get the dr_location_vector by using the centroid and bounding rectangle
                    # Since the indicator is a triangle we can assume that the back of the DeepRacer is the side of the rectangle closest to the centroid
                    # Start by getting the midpoint of the two closes sides of the rectangle
                    if dist(self.dr_box[0, 0], self.dr_box[0, 1], self.dr_box[1, 0], self.dr_box[1, 1]) < dist(
                            self.dr_box[0, 0], self.dr_box[1, 1], self.dr_box[2, 0], self.dr_box[2, 1]):
                        p1 = midpoint(self.dr_box[0, 0], self.dr_box[0, 1], self.dr_box[1, 0], self.dr_box[1, 1])
                        p2 = midpoint(self.dr_box[2, 0], self.dr_box[2, 1], self.dr_box[3, 0], self.dr_box[3, 1])
                        midpoints = np.int0([[p1[0], p1[1]], [p2[0], p2[1]]])
                    else:
                        p1 = midpoint(self.dr_box[1, 0], self.dr_box[1, 1], self.dr_box[2, 0], self.dr_box[2, 1])
                        p2 = midpoint(self.dr_box[3, 0], self.dr_box[3, 1], self.dr_box[0, 0], self.dr_box[0, 1])
                        midpoints = np.int0([[p1[0], p1[1]], [p2[0], p2[1]]])

                    # determine front and back of car
                    if dist(midpoints[0, 0], midpoints[0, 1], self.dr_centroid[0], self.dr_centroid[1]) < dist(
                            midpoints[1, 0], midpoints[1, 1], self.dr_centroid[0], self.dr_centroid[1]):
                        new_dr_location_vector = midpoints
                    else:
                        new_dr_location_vector = np.roll(midpoints, 2)

                    # apply temporal filtering on the vector
                    for i in range(self.temporal_filtering):
                        if not self.dr_location_vector[
                                   self.temporal_filtering - 1, 0, 0] == 0:  # do not want to average before vector is filled
                            new_dr_location_vector = (new_dr_location_vector + self.dr_location_vector[
                                self.temporal_filtering - i]) / 2.0
                    self.dr_location_vector = np.roll(self.dr_location_vector, -1, axis=0)
                    self.dr_location_vector[0] = new_dr_location_vector

                    if self.display_windows:
                        print("area: " + str(area))
                        print("centroid: " + str(self.dr_centroid))
                        print("bounding rectangle: \n" + str(self.dr_box))
                        print("midpoint at end of rectangles: \n" + str(midpoints))
                        print("dr location vector: \n" + str(self.dr_location_vector))
                        output = cv2.bitwise_and(self.cv_image_filtered, self.cv_image_filtered, mask=mask)
                        # cv2.drawContours(output, [self.dr_box], 0, (0, 255, 255), 2)
                        # cv2.circle(output, (self.dr_centroid[0], self.dr_centroid[1]), 5, (0, 0, 255), -1)
                        # cv2.circle(output, (midpoints[0, 0], midpoints[0, 1]), 5, (255, 0, 0), -1)
                        # cv2.circle(output, (midpoints[1, 0], midpoints[1, 1]), 5, (0, 255, 0), -1)
                        cv2.arrowedLine(output,
                                        (int(self.dr_location_vector[0, 0, 0]), int(self.dr_location_vector[0, 0, 1])),
                                        (int(self.dr_location_vector[0, 1, 0]), int(self.dr_location_vector[0, 1, 1])),
                                        (255, 255, 0),
                                        3)
                        cv2.imshow("red mask", mask)
                        cv2.imshow("DR tracking", np.hstack([self.cv_image_filtered, output]))
                        print('\n')
        else:
            print("No image supplied to DR tracker")

    def publish_cv_image(self, image):
        """Extra member function to publish an opencv image if desired"""
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def compute_state(self):
        if self.dr_centroid is not None:
            last_position = self.dr_position
            self.dr_position = self.dr_centroid * ((2.0 * (self.cam_dist_from_ground - self.dr_height) * (
                np.tan(np.radians(self.camera_fov / 2.0)))) / self.camera_resolution)
            last_velocity = self.dr_velocity
            self.dr_velocity = (self.dr_position - last_position) / (time.time() - self.last_update_time)
            self.dr_acceleration = (self.dr_velocity - last_velocity) / (time.time() - self.last_update_time)
            self.last_update_time = time.time()
            if self.display_windows:
                print("dr position: " + str(self.dr_position))
                print("dr velocity: " + str(self.dr_velocity))
                print("dr acceleration: " + str(self.dr_acceleration))


if __name__ == "__main__":
    DRTracker = TrackDR()
    # private_param = rospy.get_param('~params/param')
    # print(private_param)
    rospy.init_node("dr_tracker")
    rospy.Subscriber("/usb_cam/image_raw", Image, DRTracker.process_image)
    rospy.spin()
