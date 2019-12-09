import rospy
from std_msgs.msg import Float64
import math


def range_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Class that interfaces with thea web page to control the DeepRacer, load models, and receive camera data
class SimDRInterface:
    def __init__(self, password, ip='192.168.1.100', name="deep_racer"):
        self.pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64,
                                                       queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command',
                                                        Float64,
                                                        queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command',
                                                        Float64,
                                                        queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command',
                                                         Float64,
                                                         queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command',
                                                           Float64,
                                                           queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command',
                                                            Float64,
                                                            queue_size=1)

        # state variables
        self.manual = True
        self.start = False

    def log_on(self):
        return

    '''General purpose functions for interacting with the DeepRacer'''

    def get_home(self):
        return

    def get_is_usb_connected(self):
        return

    def send_drive_command(self, steering_angle, throttle):

        throttle = throttle*10  # simulation needs values to be increased for descent speed
        steer = range_map(steering_angle, -1, 1, -math.pi/6, math.pi/6)  # map back to radians

        # Set angle and throttle commands from -1 to 1
        self.pub_vel_left_rear_wheel.publish(throttle)
        self.pub_vel_right_rear_wheel.publish(throttle)
        self.pub_vel_left_front_wheel.publish(throttle)
        self.pub_vel_right_front_wheel.publish(throttle)
        self.pub_pos_left_steering_hinge.publish(steer)
        self.pub_pos_right_steering_hinge.publish(steer)

    def set_manual_mode(self):
        return

    def stop_car(self):
        self.send_drive_command(0, 0)
        return

    def start_car(self):
        return

    def get_raw_video_stream(self):
        # Get the video stream
        return

    '''Functions for running autonomous mode'''

    def get_models(self):
        return

    def get_uploaded_models(self):
        return

    def set_autonomous_mode(self):
        # Set the car to use the autonomous mode and not care about input from this program
        self.stop_car()
        return

    def set_throttle_percent(self, throttle_percent):
        # Set the percent throttle from 0-100% (note for manual mode this has no effect)
        return

    def load_model(self, model_name):
        return

    def manual_drive(self):
        return

    def upload_model(self, model_zip_path, model_name):
        return
