#!/usr/bin/python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse


def get_boundaries(event, x, y, flags, param):
    # grab references to the global variables
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        global image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        print("HSV at point: (%d, %d) is (%d, %d, %d)" % (x, y, hsv[y, x, 0], hsv[y, x, 1], hsv[y, x, 2]))


if __name__ == "__main__":

    rospy.init_node("color_tester")
    parser = argparse.ArgumentParser(description='Map generator arguments')
    parser.add_argument('-s', help="image subscriber topic name", default="/usb_cam/image_raw")
    args = parser.parse_args()

    bridge = CvBridge()
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", get_boundaries)

    print("Press c to end")
    done = False
    while not done and not rospy.is_shutdown():
        # display the image and wait for a keypress
        global image
        image = bridge.imgmsg_to_cv2(rospy.wait_for_message(args.s, Image), desired_encoding="bgr8")
        cv2.imshow("image", image)
        key = cv2.waitKey(1) & 0xFF
        # if the 'c' key is pressed, break from the loop
        if key == ord("c"):
            done = True
