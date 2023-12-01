#!/usr/bin/env python3

import rospy
import actionlib
from vaccuum_robot.msg import ChargeAction, ChargeGoal
from std_msgs.msg import Header


class ChargingClient:
    def __init__(self, max_battery):
        self.client = actionlib.SimpleActionClient("Charge", ChargeAction)
        self.client.wait_for_server()
        rospy.loginfo("Charging client initialized")

        goal = ChargeGoal()
        goal.header = Header()
        goal.max_battery = max_battery

        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        rospy.loginfo(f"Charging station is sendig this goal: {max_battery}")
        self.client.wait_for_result()

    def feedback_callback(self, feedback):
        rospy.loginfo(
            f"Charging Station - Current battery level: {feedback.current_battery}"
        )


if __name__ == "__main__":
    rospy.init_node("charging_client_node")
    charging_client = ChargingClient(
        max_battery=80
    )  # Set the desired maximum battery level
