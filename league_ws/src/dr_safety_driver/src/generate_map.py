#!/usr/bin/python

import os
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse
import numpy as np
import math

def get_boundaries(event, x, y, flags, param):
    # grab references to the global variables
    global points
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append([x, y])
        print("Point added: " + str(x) + " " + str(y))


# x = float(input("What is the horizontal dimensions of you track in meters: "))
# y = float(input("What is the horizontal dimensions of you track in meters: "))
# file_path = raw_input("Path to save generated map: ")
# name = raw_input("Name of map: ")
# line_thickness = 2  # pixels

def slope(x1, y1, x2, y2):
    m = (y2 - y1) / (x2 - x1)
    return m


# perform linear interpolations in order to figure out if pixel is in or out of bounds
def pixel_in_bounds(y, x):
    global points
    # first case (the y coordinate is above the top line)
    if y < (((points[1][1] - points[0][1]) / float((points[1][0] - points[0][0]))) * (x - points[0][0]) + points[0][1]):
        return False
    # second case (the y coordinate is below the bottom line)
    elif y > (((points[3][1] - points[2][1]) / float((points[3][0] - points[2][0]))) * (x - points[3][0]) + points[3][1]):
        return False
    # # third case (the x coordinate is to the left of the boundary line)
    elif x < (((points[0][0] - points[3][0]) / float((points[0][1] - points[3][1]))) * (y - points[0][1]) + points[0][0]):
        return False
    # fourth case (the x coordinate is to the right of the boundary line)
    elif x > (((points[1][0] - points[2][0]) / float((points[1][1] - points[2][1]))) * (y - points[1][1]) + points[1][0]):
        return False
    else:
        return True


if __name__ == "__main__":

    global points
    points = []

    rospy.init_node("map_generator")
    parser = argparse.ArgumentParser(description='Map generator arguments')
    parser.add_argument('-s', help="image subscriber topic name", default="/usb_cam/image_rect_color")
    parser.add_argument('-f', help="horizontal FOV", default=120, type=float)
    parser.add_argument('-v', help="vertical FOV", default=80, type=float)
    parser.add_argument('-d', help="distance from ground to camera (in meters)", default=2.0, type=float)
    parser.add_argument('-p', help="file path to save pgm and yaml map", default="../maps")
    parser.add_argument('-n', help="name of map", default="map")
    args = parser.parse_args()

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(rospy.wait_for_message(args.s, Image))
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", get_boundaries)

    # img = Imager.new('L', (image.shape[1], image.shape[0]))
    print("Please select your 4 points in order: upper left, upper right, bottom right, bottom left")
    print("Press c to end")
    while len(points) < 4:
        # display the image and wait for a keypress
        cv2.imshow("image", image)
        key = cv2.waitKey(1) & 0xFF
        # if the 'c' key is pressed, break from the loop
        if key == ord("c"):
            break
    map_img = np.zeros((image.shape[0], image.shape[1]), np.uint8)
    for u in range(map_img.shape[0]):
        for v in range(map_img.shape[1]):
            if pixel_in_bounds(u, v):
                map_img[u, v] = 255
            else:
                map_img[u, v] = 0

    # compute the factors to scale the map to the correct resolution
    res = 0.04  # 10cmx10cm per pixel
    # horizontal scale factor
    horiz_factor = (2.0*args.d*math.tan(math.radians(args.f)/2.0))/res
    vert_factor = (2.0*args.d*math.tan(math.radians(args.v)/2.0))/res
    scaled_map = cv2.resize(map_img, (int(horiz_factor), int(vert_factor)))
    cv2.imwrite("map.pgm", scaled_map)
    print("Temporary map image saved to directory where script was run from as map.pgm")
    confirmation = raw_input("Is this good? y/n")
    if confirmation == "y":
        f = open(os.path.join(args.p, args.n) + ".yaml", "w+")
        name = args.n + ".pgm"
        f.write(
            "image: %s\nresolution: %f\norigin: [%f, %f, 0]\noccupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0" %
            (name, res, points[0][0]*0, -vert_factor*res))
        cv2.imwrite(os.path.join(args.p, name), scaled_map)
        f.close()
