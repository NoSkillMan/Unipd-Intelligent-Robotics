#!/usr/bin/env python3

import rospy
import actionlib
from vaccuum_robot.msg import ChargeAction, ChargeFeedback, ChargeResult
from std_msgs.msg import Header
# from time import sleep


class RobotChargeServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "Charge", ChargeAction, self.execute, auto_start=False
        )
        self.server.start()
        rospy.loginfo("Robot Charge server initialized")

    def execute(self, goal):
        rospy.loginfo(
            f"Robot Charge server got this battery level as a goal: {goal.max_battery}"
        )
        rate = rospy.Rate(1)  # 1Hz feedback rate
        feedback = ChargeFeedback()
        feedback.header = Header()
        feedback.current_battery = 5  # Starting battery level (example)

        for _ in range(goal.max_battery - feedback.current_battery):
            feedback.header.stamp = rospy.Time.now()
            feedback.current_battery += 1
            self.server.publish_feedback(feedback)
            rate.sleep()

        result = ChargeResult()
        result.header = Header()
        result.goal_reached = True

        self.server.set_succeeded(result)
        rospy.loginfo("Robot Charge goal reached")


if __name__ == "__main__":
    rospy.init_node("robot_charge_server_node")
    robot_server = RobotChargeServer()
    rospy.spin()
