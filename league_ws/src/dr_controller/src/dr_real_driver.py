#!/usr/bin/env python

# For physical jetson car only

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from web_interface_core import DRInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dr_controller.msg import EvaluateAction, EvaluateFeedback, EvaluateResult
import actionlib
from std_msgs.msg import Bool
import math
import os


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
                 nearest_waypoint_topic="/nearest_waypoint", starting_waypoint_topic="/starting_waypoint"):

        # start car
        self.car = DRInterface(dr_password, dr_ip)
        self.car.log_on()
        self.car.set_manual_mode()
        self.car.start_car()
        rospy.loginfo("Car started")

        # basic dr state variables
        self.is_off_track = False
        self.nearest_waypoint = None
        self.current_ai_model_name = None  # Currently loaded AI model
        self.num_waypoints = None
        self.waypoints = None

        # operational variables
        self.repositioning = False  # is the DR currently attempting a goal
        self.is_evaluating = False
        self.starting_waypoint = None  # reset the car here once it finishes its routine
        self.starting_waypoint_index = None
        self.nearest_waypoint_index = None
        self.lap_start_time = 0
        self.num_corrections = 0

        # variables for evaluation action action server
        # When action is called by web_api.launch then the car will begin 3 trials
        self.evaluation_server = actionlib.SimpleActionServer('evaluate_model', EvaluateAction, self.run_eval,
                                                              auto_start=False)

        # connect to rest of ARCC League systems
        rospy.Subscriber(off_track_topic, Bool, self.dr_track_status)  # track_util off track data source
        rospy.Subscriber(nearest_waypoint_topic, PoseStamped,
                         self.update_nearest_waypoint)  # track_util nearest waypoint data source
        rospy.Subscriber("/waypoints", PoseArray, self.update_waypoints)  # track_util nearest waypoint data source
        rospy.Subscriber(starting_waypoint_topic, PoseStamped,
                         self.update_starting_waypoint)  # track_util nearest waypoint data source
        rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.update_car)  # move_base ackermann steering cmd

        # move_base_client for p2p navigation and resetting the car
        self.move_base_client = actionlib.SimpleActionClient('move_base',
                                                             MoveBaseAction)  # will be called when car goes off track or finishes and needs to be reset

    def __del__(self):
        self.car.stop_car()  # make sure to stop the car when the node closes

    def run_eval(self, goal):
        rospy.loginfo("New Evaluation triggered")
        rospy.loginfo(goal.model_file_path)
        result = EvaluateResult
        feedback = EvaluateFeedback
        # reset car to starting location
        if self.set_nav_goal(self.starting_waypoint):
            rospy.loginfo("Deepracer reset to starting position")

            self.car.stop_car()
            self.car.set_autonomous_mode()
            self.is_evaluating = True

            # Upload AI model to car and load with designated model
            model_name_with_ext = os.path.basename(goal.model_file_path)
            model_name_wout_ext = os.path.splitext(model_name_with_ext)[0]
            self.car.upload_model(goal.model_file_path, model_name_with_ext)
            # wait for model to be ready before loading it
            model_ready = False
            while not model_ready:
                models = self.car.get_uploaded_models()
                for model in models:
                    if model["name"] == model_name_wout_ext and model["status"] == "Ready":
                        model_ready = True
                rospy.sleep(0.5)
            self.car.load_model(model_name_wout_ext)
            self.car.set_throttle_percent(goal.maximum_speed)

            # will trigger system to begin recording
            feedback.evaluating = True
            feedback.elapsed_time = 0
            feedback.percent_complete = 0
            self.evaluation_server.publish_feedback(feedback)

            # Start the autonomous car, monitor if it goes off track
            self.lap_start_time = rospy.Time.now()  # starting timer
            self.num_corrections = 0
            self.car.start_car()

            rospy.sleep(1)

            while self.nearest_waypoint.pose != self.starting_waypoint.pose and self.num_corrections <= goal.num_corrections_allowed:
                # will trigger system to begin recording
                feedback.evaluating = True
                feedback.elapsed_time = rospy.Time.now() - self.lap_start_time
                feedback.percent_complete = 100.0*abs(self.nearest_waypoint_index - self.starting_waypoint_index)/self.num_waypoints
                self.evaluation_server.publish_feedback(feedback)

            self.car.stop_car()
            rospy.loginfo("Lap complete")

            result.time = rospy.Time.now() - self.lap_start_time
            if self.num_corrections <= goal.num_corrections_allowed:  # if lap completed then give 100% even though progress evaluates to 0
                result.percent_complete = 100.0
            else:
                result.percent_complete = 100.0 * abs(
                    self.nearest_waypoint_index - self.starting_waypoint_index) / self.num_waypoints
            result.num_corrections = self.num_corrections
            self.evaluation_server.set_succeeded(result)
            self.is_evaluating = False

    def dr_track_status(self, data):
        current_lap_time = self.lap_start_time
        self.is_off_track = data.data
        if self.is_off_track == True and self.repositioning == False and self.nearest_waypoint is not None:
            self.set_nav_goal(self.nearest_waypoint)
            if self.is_evaluating:
                self.num_corrections += 1
                self.car.stop_car()
                self.car.set_autonomous_mode()
                self.car.start_car()
            else:
                self.car.stop_car()
            self.lap_start_time += rospy.Time.now() - current_lap_time  # time resetting car does not count for time in the lap

    def update_waypoints(self, data):
        self.waypoints = data
        self.num_waypoints = len(data.poses)
        self.evaluation_server.start()  # start server here since we know at this point that the tracker has started properly

    def update_nearest_waypoint(self, data):
        self.nearest_waypoint = data
        # find the waypoint index of the start_position
        while self.waypoints is None:  # wait for waypoints to load
            rospy.sleep(0.5)
        for i, waypoint in enumerate(self.waypoints.poses):
            if waypoint == self.nearest_waypoint.pose:
                self.nearest_waypoint_index = i

    def update_starting_waypoint(self, data):
        self.starting_waypoint = data
        # find the waypoint index of the start_position
        while self.waypoints is None:  # wait for waypoints to load
            rospy.sleep(0.5)
        for i, waypoint in enumerate(self.waypoints.poses):
            if waypoint == self.starting_waypoint.pose:
                self.starting_waypoint_index = i

    # ROS action client for getting the car back on track
    def set_nav_goal(self, setpoint):

        self.car.stop_car()
        self.car.set_manual_mode()
        self.car.start_car()

        self.repositioning = True

        self.move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose = setpoint
        goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result(rospy.Duration.from_sec(120.0))
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            return False
        self.repositioning = False
        return True

    def update_car(self, data):
        # If the car is in the process of repositioning to a new waypoint then turn off AI and take control
        if self.repositioning:
            # compute steering angle -30deg to 30deg
            steer = constrain(data.drive.steering_angle, -math.pi / 6, math.pi / 6)
            steer = range_map(steer, -math.pi / 6, math.pi / 6, -1.0, 1.0)
            drive = data.drive.speed

            # handle deadzones
            drive_deadzone = [0.35, -0.35]
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
    param_dr_password = rospy.get_param("~dr_password", "JJo1qfmc")
    param_dr_ip = rospy.get_param("~dr_ip", "192.168.1.100")
    driver = Driver(param_dr_ip, param_dr_password)
    rospy.spin()
