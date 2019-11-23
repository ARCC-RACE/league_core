#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import math


# converts cmd_vel to steering angle ackermann_msg
# http://planning.cs.uiuc.edu/node658.html
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
    return math.atan(wheelbase / radius)


class AckermannTranslator:
    def __init__(self, wheelbase=0.16):
        # topics to publish updated PID values to
        self.setpoint_pub = rospy.Publisher('/drive_pid/setpoint', Float64, queue_size=1)
        self.state_pub = rospy.Publisher('/drive_pid/state', Float64, queue_size=1)

        # state variables
        self.wheelbase = wheelbase
        self.velocity = 0
        self.steering = 0

        # topics to publish for the deepracer to update control
        self.ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)

    # receive update from PID
    def new_speed(self, effort):
        ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
        ackermann_msg = AckermannDriveStamped()

        header = Header()
        header.frame_id = "base_link"
        header.stamp = rospy.Time.now()
        ackermann_msg.header = header

        self.velocity = effort.data
        ackermann_msg.drive.speed = self.velocity
        ackermann_msg.drive.steering_angle = self.steering  # last steering angle
        ackermann_pub.publish(ackermann_msg)

    def new_cmd(self, data):
        # update PID with current velocity with reference to car frame
        self.setpoint_pub.publish(Float64(data.linear.x))

        # send Ackermann message with steering angle
        ackermann_msg = AckermannDriveStamped()

        header = Header()
        header.frame_id = "base_link"
        header.stamp = rospy.Time.now()
        ackermann_msg.header = header

        # ackermann_msg.drive.speed = velocity  # last velocity
        ackermann_msg.drive.speed = data.linear.x  # last velocity
        self.steering = convert_trans_rot_vel_to_steering_angle(ackermann_msg.drive.speed, data.angular.z,
                                                                self.wheelbase)
        ackermann_msg.drive.steering_angle = self.steering
        self.ackermann_pub.publish(ackermann_msg)

    def new_state(self, odom):
        self.state_pub.publish(Float64(odom.twist.twist.linear.x))


if __name__ == '__main__':
    rospy.init_node("ackermann_translator")
    wheelbase_param = rospy.get_param("~wheelbase", 0.16)
    translator = AckermannTranslator(wheelbase_param)

    odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
    rospy.Subscriber(odom_topic, Odometry, translator.new_state)  # for updating state
    rospy.Subscriber("/cmd_vel", Twist, translator.new_cmd)
    rospy.Subscriber("/drive_pid/control_effort", Float64, translator.new_speed)
    rospy.spin()
