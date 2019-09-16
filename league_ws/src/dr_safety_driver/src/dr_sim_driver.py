#!/usr/bin/env python

# For physical jetson car only

import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


def set_throttle_steer(data):
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64,
                                              queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64,
                                               queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64,
                                               queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64,
                                                queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64,
                                                  queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64,
                                                   queue_size=1)

    throttle = data.drive.speed  # simulation needs values to be increased for descent speed
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)


if __name__ == '__main__':
    rospy.init_node("ackermann_translator")
    rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, set_throttle_steer)
    rospy.spin()