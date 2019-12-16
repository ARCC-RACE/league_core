#!/usr/bin/env python

import rospy
import requests
import actionlib
from dr_controller.msg import EvaluateAction, EvaluateGoal
import json
import os

# URL - https://arcc-league.herokuapp.com/
# Model Object:
# id - Object ID
# ownerId - Owner ID
# trackName - Track model was tested on)
# modelName - Name of the model) Can be changed by User
# modelDescription - (User description) Can be changed by User
# dateUploaded - (Date the model was uploaded)
# isEvaluated - if evaluated (boolean)
# time - Speed car completed the track
# speedTested - (Speed the model was tested at (percentage))
# videoLink - (Link to video upload)
# modelLink - (Link to the file)
# modelId: dto.modelId - modelID
# invoiceNumber - Paypal Order ID
# isPaid: dto.isPaid - If users payed for it yet


class League_api:
    def __init__(self, root="https://arcc-league.herokuapp.com/", model_endpoint="models", update_period=60):
        # URL and request sessions
        self.root = root
        self.model_endpoint = self.root + model_endpoint
        self.session = requests.Session()

        # json objects from ARCC league web api
        self.models = None
        self.model_under_eval = None

        # ROS timer
        rospy.Timer(rospy.Duration(update_period), self.update_eval_queue)

        # action server
        self.evaluate_client = actionlib.SimpleActionClient('evaluate_model', EvaluateAction)
        self.evaluate_client.wait_for_server()
        self.update_eval_queue()

    def get_models(self):
        self.models = json.loads(self.session.get(self.model_endpoint).text)
        rospy.loginfo("Models updated")

    def update_eval_queue(self, event=None):
        self.get_models()
        for model in self.models:
            if model["isEvaluated"] is False and self.model_under_eval is None:
                rospy.loginfo("Evaluating model: " + str(model["modelName"]))
                self.model_under_eval = model

                goal = EvaluateGoal()
                # string model_file_path
                goal.model_file_path = model["modelLink"]
                # float64 maximum_speed
                goal.maximum_speed = 60.0
                # int64 num_corrections_allowed
                goal.num_corrections_allowed = 3
                # int64 num_laps
                goal.maximum_speed = 1

                self.evaluate_client.send_goal(goal)
                results = self.evaluate_client.wait_for_result()

                self.model_under_eval["isEvaluated"] = True
                self.model_under_eval["time"] = str(results.time)
                self.session.put(os.path.join(self.model_endpoint, model["id"]), json=self.model_under_eval)

                self.model_under_eval = None
                self.update_eval_queue()
        rospy.loginfo("No models to evaluate at this time")


if __name__ == "__main__":
    rospy.init_node("league_api_interface")
    rospy.loginfo("Starting API Interface")
    interface = League_api()
    rospy.spin()
