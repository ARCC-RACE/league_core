#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion


# converts cmd_vel to ackermann_msgs


def new_cmd(data):
    ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped,
                                    queue_size=1)
    ackermann_msg = AckermannDriveStamped()

    header = Header()
    header.frame_id = ""
    header.stamp = rospy.Time.now()
    ackermann_msg.header = header

    ackermann_msg.drive.speed = data.linear.x
    # positive is left and negative is right
    ackermann_msg.drive.steering_angle = data.angular.z

    ackermann_pub.publish(ackermann_msg)


if __name__ == '__main__':
    rospy.init_node("ackermann_controller")
    rospy.Subscriber("/cmd_vel", Twist, new_cmd)
    rospy.spin()
