#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool
import time
import tf
import math


def constrain(x, min, max):
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x


class TrackUtil():

    def __init__(self, debug_mode=True, temporal_filtering=2, line_contour_cutoff=100, cam_dist_from_ground=1,
                 camera_vertical_fov=80, camera_horizontal_fov=120, dr_height=0.254, track_line_lower=None,
                 track_line_upper=None, blur_param=9, refresh_rate=0, center_point=(480, 100)):
        if track_line_lower is None:
            track_line_lower = [0, 0, 0]
        if track_line_upper is None:
            track_line_upper = [180, 50, 255]
        self.debug = debug_mode
        self.temporal_filtering = temporal_filtering
        self.line_contour_cutoff = line_contour_cutoff
        self.cam_dist_from_ground = cam_dist_from_ground
        self.camera_vertical_fov = camera_vertical_fov
        self.camera_horizontal_fov = camera_horizontal_fov
        self.track_line_lower = track_line_lower
        self.track_line_upper = track_line_upper
        self.blur_param = blur_param
        self.refresh_rate = refresh_rate
        self.center_point = center_point
        self.dr_height = dr_height
        self.camera_fov = np.array([camera_horizontal_fov, camera_vertical_fov])

        self.track_status_pub = rospy.Publisher("is_off_track", Bool, queue_size=1)
        self.waypoints_pub = rospy.Publisher("waypoints", PoseArray, queue_size=1, latch=True)
        self.nearest_waypoint_pub = rospy.Publisher("nearest_waypoint", PoseStamped, queue_size=1)

        self.waypoints = None  # this will be a numpy array in pixel x,y coordinates and r,p,y orientation based on heading
        self.last_image_time = 0
        self.cv_image_filtered = None
        self.cv_image_raw = None
        self.camera_resolution = None
        self.parsed_img = None
        self.track_mask = None
        self.bridge = CvBridge()

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
            cv2.imshow("Track mask", self.track_mask)
            cv2.imshow("parsed image window",
                       cv2.bitwise_and(self.cv_image_raw, self.cv_image_raw, mask=self.parsed_img))
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
            cnts = []
            for i, cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if self.line_contour_cutoff < area:
                    cnts.append(i)
                    if self.debug:
                        print("Area of contour " + str(i) + ": " + str(area))

            if len(cnts) > 1:
                # mask1 = outer, mask2 = inner track
                cv2.drawContours(mask1, contours, cnts[0], 255, -1)  # Draw filled contour in mask
                cv2.drawContours(mask2, contours, cnts[1], 255, -1)  # Draw filled contour in mask
                self.track_mask = cv2.bitwise_or(mask1, mask2)
                if cv2.contourArea(contours[cnts[0]]) > cv2.contourArea(contours[cnts[1]]):  # mask 1 is the outer track
                    self.generate_waypoints(mask2, mask1)
                else:
                    self.generate_waypoints(mask1, mask2)
                self.send_waypoints()
                for x in range(mask1.shape[0]):
                    for y in range(mask1.shape[1]):
                        if mask1[x, y] > 200:
                            cv2.line(mask1, (y, x), self.center_point, 150, 5)
                for x in range(mask2.shape[0]):
                    for y in range(mask2.shape[1]):
                        if mask2[x, y] > 200:
                            cv2.line(mask2, (y, x), self.center_point, 150, 5)
                if self.debug:
                    cv2.circle(mask2, self.center_point, 5, 200, thickness=10)
                    cv2.circle(mask1, self.center_point, 5, 200, thickness=10)
                self.parsed_img = cv2.bitwise_or(cv2.bitwise_xor(mask1, mask2),
                                                 cv2.dilate(self.track_mask, np.ones((7, 7), np.uint8)))
            else:
                self.cv_image_raw = None  # reload an image due to inadequacy of original

    def generate_waypoints(self, inner_track_mask, outer_track_mask, radius=20, direct=-1):

        """The premise of this algorithm is to start by getting a point on the mask and the using a rotation based off
         the line pointed toward the center where the initial inside track outer point was selected. After the first points
          then selects the second point by rotating with radius r, then the perpendicular line to the line between the two points is  
          use to form the r length rotating line that will select p3, p4, pn. The direction of rotation is defined as +1/-1 with the direct variable"""

        # select the starting point on the outside of the inner part of the track
        x = 0
        p1 = None
        while p1 is None:
            x += 1
            # opencv goes (y,x)
            if inner_track_mask[self.center_point[1], self.center_point[0] - x] > 200:
                p1 = np.array([self.center_point[0] - x, self.center_point[1]])

        # now computer new_p and begin moving around the track until distance to last_p < radius
        self.waypoints = []
        last_p = p1
        last_heading = 0
        for i in range(20):

            # compute the next point to use for generating a new waypoint
            new_p = None
            if direct > 0:
                theta = last_heading - math.pi / 2
            else:
                theta = last_heading + math.pi / 2
            while new_p is None:
                theta += 0.01 * direct / abs(direct)
                if inner_track_mask[
                    int(-radius * math.sin(theta) + last_p[1]), int(radius * math.cos(theta) + last_p[0])] > 200:
                    new_p = np.array(
                        [int(radius * math.cos(theta) + last_p[0]), int(-radius * math.sin(theta) + last_p[1])])

            # with new_p compute the line between them, get the midpoint and use the perpendicular line to the the distance to the outer track
            # once the distance to the outer track is found then find that midpoint for the coordinate of the waypoint, computer heading with
            # the line between last_p and new_p with the direction last_p->new_p, new radius will be the perpendicular line erected in the
            # opposite direction of the first perpendicular line

            # compute the line btw the two points
            slope = -(new_p[1] - last_p[1]).astype(float) / (
                    last_p[0] - new_p[0])  # swap x to deal with inverse y coordinate system
            perpendicular_slope = -1 / slope
            midpoint = ((last_p + new_p) / 2.0).astype(int)

            outer_track_point = midpoint.astype(int)
            x = 0
            while outer_track_mask[outer_track_point[1], outer_track_point[0]] < 200:
                # check both directions, this may cause issues if there are parts of the inner and outer track close together
                if perpendicular_slope == float("-inf") or perpendicular_slope == float("inf"):
                    outer_track_point = np.array([int(midpoint[0]), int(x * (-1) ** round(x) + midpoint[1])])
                    # outer_track_point = np.array([(int(x) * (-1) ** round(x)), int(perpendicular_slope * (midpoint[0] - int(x)) * (-1) ** round(x) + midpoint[1])])
                else:
                    # outer_track_point = np.array([int(x), int(perpendicular_slope * (midpoint[0] - x) + midpoint[1])])
                    outer_track_point = np.array([int(midpoint[0] + x * (-1) ** round(x / 0.01)), int(
                        perpendicular_slope * (x * (-1) ** round(x / 0.01)) + midpoint[1])])
                # cv2.line(self.track_mask, (int(midpoint[0]), int(midpoint[1])),
                #          (int(outer_track_point[0]), int(outer_track_point[1])), 150, 5)
                x -= 0.01  # make sure that no index is skipped using int() round() and += 0.5
            outer_inner_midpoint = (outer_track_point + midpoint) / 2.0
            if self.debug:
                cv2.line(self.track_mask, (int(midpoint[0]), int(midpoint[1])),
                         (int(outer_track_point[0]), int(outer_track_point[1])), 150, 5)

            # now compute heading for the point
            v = np.array((1, 0))
            u = (new_p - last_p)
            heading = np.arccos(u.dot(v) / (np.sqrt(u.dot(u)) * np.sqrt(v.dot(v))))
            if last_p[1] < new_p[1]:
                heading = -heading
            self.waypoints.append([outer_inner_midpoint[0], outer_inner_midpoint[1], heading])

            if self.debug:
                print("last_p: " + str(last_p))
                print("new_p: " + str(new_p))
                print("midpoint: " + str(midpoint))
                print("outer point: " + str(outer_track_point))
                print("perpendicular slope: " + str(perpendicular_slope))
                cv2.circle(self.track_mask, (last_p[0], last_p[1]), 5, 200, thickness=10)
                cv2.circle(self.track_mask, (new_p[0], new_p[1]), 5, 100, thickness=10)
                print("")

            last_p = new_p
            last_heading = heading

        for i in range(len(self.waypoints)):
            cv2.circle(self.track_mask, (int(self.waypoints[i][0]), int(self.waypoints[i][1])), 5, 255, thickness=10)
        print(self.waypoints)

    def send_waypoints(self):
        waypoints = PoseArray()
        waypoints.header.stamp = rospy.Time.now()
        waypoints.header.frame_id = "map"
        for i in range(len(self.waypoints)):
            pose = Pose()
            waypoint = np.array(self.waypoints[i])
            heading = waypoint[2]
            # convert waypoint position into meters
            waypoint = (waypoint[0:2] / (self.camera_resolution * 1.0)) * (
                        2.0 * (self.cam_dist_from_ground - self.dr_height) * (
                    np.tan(np.radians(self.camera_fov / 2.0))))
            pose.position.x = waypoint[0]
            pose.position.y = -waypoint[1]
            orientation = tf.transformations.quaternion_from_euler(0, 0, heading)

            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]
            waypoints.poses.append(pose)
        self.waypoints_pub.publish(waypoints)

    def get_nearest_waypoint(self, location_in_pixels):
        min_dist = 9999
        min_dist_waypoint_index = 0
        for i, waypoint in enumerate(self.waypoints):
            dist = math.sqrt((location_in_pixels[0] - waypoint[0]) ** 2 + (location_in_pixels[1] - waypoint[1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_dist_waypoint_index = i

        waypoint = np.array(self.waypoints[min_dist_waypoint_index])
        heading = waypoint[2]
        waypoint = (waypoint[0:2] / (self.camera_resolution * 1.0)) * (
                2.0 * (self.cam_dist_from_ground - self.dr_height) * (
            np.tan(np.radians(self.camera_fov / 2.0))))
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = -waypoint[1]
        orientation = tf.transformations.quaternion_from_euler(0, 0, heading)
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        self.nearest_waypoint_pub.publish(pose)

    def is_off_track(self, odom):
        if self.parsed_img is not None:
            # convert x,y position back to pixels (inverse operation)
            dr_position = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
            camera_fov = np.array([self.camera_horizontal_fov, self.camera_vertical_fov])
            centroid = np.abs((dr_position / (
                    2.0 * (self.cam_dist_from_ground - self.dr_height) * (
                np.tan(np.radians(camera_fov / 2.0))))) * self.camera_resolution)
            if self.debug:
                print("Position of car in pixels: " + str(centroid))
            centroid_pixels = [constrain(centroid.astype(int)[0], 0, self.camera_resolution[0]),
                               constrain(centroid.astype(int)[1], 0, self.camera_resolution[1])]
            self.get_nearest_waypoint(centroid_pixels)
            if self.parsed_img[centroid_pixels[1], centroid_pixels[0]] > 100:
                self.track_status_pub.publish(Bool(False))
            else:
                self.track_status_pub.publish(Bool(True))
        else:
            self.track_status_pub.publish(Bool(True))


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
    dr_height_param = rospy.get_param("~dr_height", 0.0254)
    input_camera_topic_param = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
    odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
    white_mask = map(int, rospy.get_param("~white_mask", "0, 0, 0, 180, 50, 255").split(','))
    center_point = map(int, rospy.get_param("~center_point", "400, 200").split(','))
    refresh_param = rospy.get_param('~refresh_rate', 10)

    track = TrackUtil(debug_param, temporal_filtering_param, line_contour_cutoff,
                      cam_dist_from_ground_param, camera_vertical_fov_param, dr_height=dr_height_param,
                      camera_horizontal_fov=camera_horizontal_fov_param,
                      blur_param=blur_param, refresh_rate=refresh_param,
                      track_line_lower=[white_mask[0], white_mask[1], white_mask[2]],
                      track_line_upper=[white_mask[3], white_mask[4], white_mask[5]],
                      center_point=(center_point[0], center_point[1]))

    rospy.Subscriber(input_camera_topic_param, Image, track.process_image)
    rospy.Subscriber(odom_topic, Odometry, track.is_off_track)
    rospy.spin()
