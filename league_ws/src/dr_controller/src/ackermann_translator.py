#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf


# converts cmd_vel to ackermann_msgs and update PID
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
    return math.atan(wheelbase / radius)


# receive update from PID
def new_speed(effort):
    global velocity, steering

    ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
    ackermann_msg = AckermannDriveStamped()

    header = Header()
    header.frame_id = "base_link"
    header.stamp = rospy.Time.now()
    ackermann_msg.header = header

    velocity = effort.data
    ackermann_msg.drive.speed = velocity
    ackermann_msg.drive.steering_angle = steering  # last steering angle
    ackermann_pub.publish(ackermann_msg)


def new_cmd(data):
    # # update PID with current velocity with reference to car frame
    # state_pub = rospy.Publisher('/drive_pid/setpoint', Float64, queue_size=1)
    # state_pub.publish(Float64(data.linear.x))
    #
    # # send Ackermann message with steering angle
    ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
    ackermann_msg = AckermannDriveStamped()

    header = Header()
    header.frame_id = "base_link"
    header.stamp = rospy.Time.now()
    ackermann_msg.header = header

    # positive is left and negative is right
    global wheelbase, velocity, steering
    # ackermann_msg.drive.speed = velocity  # last velocity
    ackermann_msg.drive.speed = data.linear.x  # last velocity
    steering = convert_trans_rot_vel_to_steering_angle(ackermann_msg.drive.speed, data.angular.z, wheelbase)
    ackermann_msg.drive.steering_angle = steering
    ackermann_pub.publish(ackermann_msg)


def new_state(odom):
    # speed = math.sqrt(odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2)
    # current_velocity = odom.twist.twist.linear.x/abs(odom.twist.twist.linear.x)*speed
    state_pub = rospy.Publisher('/drive_pid/state', Float64, queue_size=1)
    state_pub.publish(Float64(odom.twist.twist.linear.x))


if __name__ == '__main__':
    rospy.init_node("ackermann_translator")
    global wheelbase, velocity, steering
    velocity = 0
    steering = 0
    wheelbase = rospy.get_param("~wheelbase", 0.16)
    odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
    rospy.Subscriber(odom_topic, Odometry, new_state)  # for updating state
    rospy.Subscriber("/cmd_vel", Twist, new_cmd)
    rospy.Subscriber("/drive_pid/control_effort", Float64, new_speed)
    rospy.spin()
