#!/usr/bin/python

import rospy
import numpy as np
import math
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
import tf


def new_pose(data):
    tr = TransformStamped()
    tr.header.stamp = rospy.Time.now()
    tr.header.frame_id = "/map"
    tr.child_frame_id = "base_link"
    tr.transform.translation.x = data.pose.position.x
    tr.transform.translation.y = data.pose.position.y
    tr.transform.rotation = data.pose.orientation
    tf = TFMessage()
    tf.transforms.append(tr)
    tf_pub.publish(tf)


rospy.init_node("dr_tr_node")
tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=1)
dr_pose = rospy.Subscriber("/dr_tracker/pose", PoseStamped, new_pose)
rospy.spin()
