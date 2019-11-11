#!/usr/bin/env python

# For physical jetson car only

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from web_interface_core import DRInterface
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Bool
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


class Driver:

    def __init__(self, dr_ip="192.168.1.100", dr_password="JJo1qfmc", off_track_topic="/is_off_track",
                 nearest_waypoint_topic="/nearest_waypoint"):

        # start car
        self.car = DRInterface(dr_password, dr_ip)
        self.car.log_on()
        self.car.set_manual_mode()
        self.car.start_car()
        print("Car started")

        self.repositioning = False  # current attempting a goal
        self.is_off_track = False
        self.nearest_waypoint = None

        rospy.Subscriber(off_track_topic, Bool, self.dr_track_status)
        rospy.Subscriber(nearest_waypoint_topic, PoseStamped, self.update_nearest_waypoint)
        rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.update_car)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def __del__(self):
        self.car.stop_car()

    def dr_track_status(self, data):
        self.is_off_track = data.data
        if self.is_off_track == True and self.repositioning == False and self.nearest_waypoint is not None:
            self.repositioning = True
            self.set_nav_goal(self.nearest_waypoint)

    def update_nearest_waypoint(self, data):
        self.nearest_waypoint = data

    def set_nav_goal(self, setpoint):

        self.move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose = setpoint
        goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        print(self.move_base_client.get_result())
        self.repositioning = False

    def goal_status(self, data):
        # PENDING = 0  # The goal has yet to be processed by the action server
        # ACTIVE = 1  # The goal is currently being processed by the action server
        # PREEMPTED = 2  # The goal received a cancel request after it started executing
        # #   and has since completed its execution (Terminal State)
        # SUCCEEDED = 3  # The goal was achieved successfully by the action server (Terminal State)
        # ABORTED = 4  # The goal was aborted during execution by the action server due
        # #    to some failure (Terminal State)
        # REJECTED = 5  # The goal was rejected by the action server without being processed,
        # #    because the goal was unattainable or invalid (Terminal State)
        # PREEMPTING = 6  # The goal received a cancel request after it started executing
        # #    and has not yet completed execution
        # RECALLING = 7  # The goal received a cancel request before it started executing,
        # #    but the action server has not yet confirmed that the goal is canceled
        # RECALLED = 8  # The goal received a cancel request before it started executing
        # #    and was successfully cancelled (Terminal State)
        # LOST = 9  # An action client can determine that a goal is LOST. This should not be
        # #    sent over the wire by an action server
        print(data.status.status)
        if data.status.status == 3 or data.status.status == 5 or data.status.status == 4:
            self.repositioning = False
            if data.status.status == 3:
                print("########## Goal achieved!")
        if data.status.status == 9:
            print("######### Car lost!")
            self.car.stop_car()

    def update_car(self, data):
        # compute steering ang -30deg to 30deg
        if not self.repositioning:
            self.car.send_drive_command(0, 0)
        else:
            steer = constrain(data.drive.steering_angle, -math.pi / 6, math.pi / 6)
            steer = range_map(steer, -math.pi / 6, math.pi / 6, -1.0, 1.0)
            drive = data.drive.speed
            # handle deadzones
            drive_deadzone = [0.4, -0.9]
            if drive > 0:
                # drive = drive_deadzone[0] + drive ** 5  # exponential control
                drive = drive_deadzone[0]
            elif drive < 0:
                # drive = drive_deadzone[1] + drive ** 5
                drive = drive_deadzone[1]
            drive = constrain(drive, -1.0, 1.0)
            rospy.loginfo("Steer Raw: %f Steer: %f Drive: %f" % (data.drive.steering_angle, steer, drive))
            self.car.send_drive_command(steer * -1.0, drive * -1.0)


if __name__ == '__main__':
    rospy.init_node("dr_driver")
    driver = Driver()
    rospy.spin()
