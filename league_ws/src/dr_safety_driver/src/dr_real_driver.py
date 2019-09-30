#!/usr/bin/env python

# For physical jetson car only

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from web_interface_core import DRInterface
import math


def range_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def constrain(x, min, max):
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x


def update_car(data):
    global drive, steer
    # compute steering ang -30deg to 30deg
    steer = constrain(data.drive.steering_angle, -math.pi / 6, math.pi / 6)
    steer = range_map(steer, -math.pi / 6, math.pi / 6, -1.0, 1.0)
    drive = data.drive.speed
    # handle deadzones
    if -0.1 < drive < 0.1:
        drive = 0
    elif drive > 0:
        drive = 0.65
    elif drive < 0:
        drive = -0.6
    drive = constrain(drive, -1.0, 1.0)
    rospy.loginfo("Steer Raw: %f Steer: %f Drive: %f" % (data.drive.steering_angle, steer, drive))


if __name__ == '__main__':
    rospy.init_node("dr_real_driver")
    global drive, steer
    drive = 0
    steer = 0
    car = DRInterface("uGRqirr3", '192.168.1.101')
    car.log_on()
    car.set_manual_mode()
    car.start_car()
    rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, update_car)
    while not rospy.is_shutdown():
        car.send_drive_command(steer * -1.0, drive * -1.0)
    car.stop_car()
