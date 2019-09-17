#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from geometry_msgs.msg import Vector3, PoseStamped, AccelStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
import time
import tf


def dist(x1, y1, x2, y2):
    return math.sqrt(float((y2 - y1) ** 2 + (x2 - x1) ** 2))


def midpoint(x1, y1, x2, y2):
    return (x2 + x1) / 2.0, (y2 + y1) / 2.0


class TrackDR:
    def __init__(self, display_windows=True, temporal_filtering=2, dr_contour_area_cutoff=100, cam_dist_from_ground=1,
                 camera_vertical_fov=80, camera_horizontal_fov=120, dr_height=0.17, arrow_lower=[170, 50, 90],
                 arrow_upper=[200, 255, 255]):
        self.acceleration = None
        self.image_pub = rospy.Publisher("dr_tracker/image_raw", Image, queue_size=1)
        self.pose_pub = rospy.Publisher("dr_tracker/pose", PoseStamped, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=1)
        self.bridge = CvBridge()
        self.display_windows = display_windows
        self.cv_image_raw = None
        self.cv_image_filtered = None  # Image with blur or median applied. This will be better for processing contours
        self.dr_centroid = None
        self.dr_box = None  # the bounding box the of DR triangle for determining orientation vector
        self.dr_contour_area_cutoff = dr_contour_area_cutoff
        self.arrow_lower = arrow_lower
        self.arrow_upper = arrow_upper
        # self.last_bounding_rect = None
        # self.last_centroid = None
        self.temporal_filtering = temporal_filtering  # the number of dr_location vectors to average (0 = no filtering)
        self.dr_location_vector = np.zeros((self.temporal_filtering + 1, 2,
                                            2))  # two points that represent the orientation and location fo the DeepRacer (first index is front)
        # final state variables for the DeepRacer
        self.dr_velocity = [0.0, 0.0]
        self.dr_acceleration = [0.0, 0.0]
        self.dr_position = [0.0, 0.0]
        self.dr_heading = 0.0
        self.dr_angular_accel = 0.0
        self.dr_angular_vel = 0.0
        self.last_update_time = time.time()  # used for finding dx/dt, dv/dt

        # variables for holding information about the cameras in order to compute the velocity and absolute position of the car
        # units = meters, degrees, seconds
        self.dr_height = dr_height
        self.cam_dist_from_ground = cam_dist_from_ground
        self.camera_fov = np.array((camera_horizontal_fov, camera_vertical_fov))
        self.camera_resolution = None

    def update_arrow_mask(self, arrow_lower_hue, arrow_lower_sat, arrow_lower_value, arrow_upper_hue, arrow_upper_sat, arrow_upper_val):
        self.arrow_lower = [arrow_lower_hue, arrow_lower_sat, arrow_lower_value]
        self.arrow_upper = [arrow_upper_hue, arrow_upper_sat, arrow_upper_val]

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
        self.publish_state()
        cv2.waitKey(3)

    def filter_image(self):
        # median = cv2.medianBlur(image, 9)
        # blur = cv2.bilateralFilter(image, 9, 75, 75)
        blur = cv2.GaussianBlur(self.cv_image_raw, (15, 15), 0)
        # if self.display_windows:
        #     cv2.imshow("Filter image window", blur)
        #     cv2.imshow("Blurred image window", blur)
        self.cv_image_filtered = blur

    def detect_dr_arrow(self):
        """Detects the red arrow that indicates the location and direction of the Deepracer"""
        if self.cv_image_filtered is not None:  # make sure that a frame exists
            # create NumPy arrays from the boundaries
            hsv = cv2.cvtColor(self.cv_image_filtered, cv2.COLOR_BGR2HSV)  # convert to hsv for easier color isolation
            low_red = np.array(self.arrow_lower)  # lower bound for red mask
            high_red = np.array(self.arrow_upper)  # upper bound for red mask

            # find the colors within the specified boundaries and apply
            # the mask
            mask1 = cv2.inRange(hsv, low_red, high_red)  # create a mask that only looks at the red arrow
            mask2 = cv2.inRange(hsv, np.array([0, 50, 90]), np.array([5, 255, 255]))
            mask = mask1 | mask2  # combine both HSV red fields
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
            # compute heading using angle between two vectors (radians)
            v = np.array((1, 0))
            u = (self.dr_location_vector[0, 0] - self.dr_location_vector[0, 1])
            self.dr_heading = math.pi - np.arccos(u.dot(v) / (np.sqrt(u.dot(u)) * np.sqrt(v.dot(v))))
            if self.dr_location_vector[0, 0, 1] < self.dr_location_vector[0, 1, 1]:
                self.dr_heading = 2 * math.pi - self.dr_heading
            last_heading = self.dr_heading
            self.dr_angular_vel = (self.dr_heading - last_heading) / (time.time() - self.last_update_time)
            if self.display_windows:
                print("dr position: " + str(self.dr_position))
                print("dr velocity: " + str(self.dr_velocity))
                print("dr acceleration: " + str(self.dr_acceleration))
                print("dr heading: " + str(self.dr_heading))
                print("\n")

    def publish_state(self):
        if self.dr_centroid is not None:
            header = Header()
            header.frame_id = "/base_link"
            header.stamp = rospy.Time.now()
            position_message = PoseStamped()
            position_message.header = header
            orientation = tf.transformations.quaternion_from_euler(0, 0, self.dr_heading)
            # rospy.logerr(self.dr_heading)
            position_message.pose.orientation.x = orientation[0]
            position_message.pose.orientation.y = orientation[1]
            position_message.pose.orientation.z = orientation[2]
            position_message.pose.orientation.w = orientation[3]
            position_message.pose.position.x = self.dr_position[0]
            position_message.pose.position.y = -self.dr_position[1]
            self.pose_pub.publish(position_message)

            odom_msg = Odometry()
            odom_msg.header = header
            odom_msg.pose.pose = position_message.pose
            odom_msg.twist.twist.linear.x = self.dr_velocity[0]
            odom_msg.twist.twist.linear.y = -self.dr_velocity[1]
            odom_msg.twist.twist.angular.z = self.dr_angular_vel
            self.odom_pub.publish(odom_msg)

            # acceleration_message = AccelStamped()
            # acceleration_message.header = header

            tr = TransformStamped()
            tr.header.stamp = rospy.Time.now()
            tr.header.frame_id = "map"
            tr.child_frame_id = "base_link"
            tr.transform.translation = position_message.pose.position
            tr.transform.rotation = position_message.pose.orientation
            tf_msg = TFMessage()
            tf_msg.transforms.append(tr)
            self.tf_pub.publish(tf_msg)


if __name__ == "__main__":
    rospy.init_node("dr_tracker")

    # ros parameters for setting up the dr tracking node
    display_windows_param = rospy.get_param('~debug', False)
    temporal_filtering_param = rospy.get_param('~temporal_filtering', 2)
    dr_contour_area_cutoff_param = rospy.get_param("dr_contour_area_cutoff", 100)
    cam_dist_from_ground_param = rospy.get_param("~camera_dist_from_ground", 1)
    camera_vertical_fov_param = rospy.get_param("~camera_vertical_fov", 80)
    camera_horizontal_fov_param = rospy.get_param("~camera_horizontal_fov", 120)
    dr_height_param = rospy.get_param("~dr_height", 0.17)
    input_camera_topic_param = rospy.get_param("~camera_topic", "/usb_cam/image_raw")

    DRTracker = TrackDR(display_windows_param, temporal_filtering_param, dr_contour_area_cutoff_param,
                        cam_dist_from_ground_param, camera_vertical_fov_param, camera_horizontal_fov_param,
                        dr_height_param, arrow_lower=[170, 50, 90], arrow_upper=[180, 255, 255])

    rospy.Subscriber(input_camera_topic_param, Image, DRTracker.process_image)
    rospy.spin()
