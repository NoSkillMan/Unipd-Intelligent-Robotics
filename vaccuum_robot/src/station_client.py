#!/usr/bin/env python3


import rospy
from vaccuum_robot.srv import Station, StationResponse, StationRequest
from vaccuum_robot.msg import Info
from std_msgs.msg import Header
import random


def charging_station_node(stationID, request_interval):
    rospy.init_node(f"station_node_{stationID}", anonymous=True)
    # Create the service proxy
    rospy.wait_for_service("station")
    station = rospy.ServiceProxy("station", Station)

    # Create a subscriber to receive robot responses
    response_subscriber = rospy.Subscriber(
        "robot_response", StationResponse, handle_response
    )

    rate = rospy.Rate(1.0 / request_interval)  # Request every X seconds

    while not rospy.is_shutdown():
        # Create the request
        request = StationRequest()
        request.header = Header()
        request.stationID = stationID

        # Call the service
        response = station(request)

        # Print the received information
        rospy.loginfo(
            f"Charging Station {stationID} - Request: {request} | Response: {response}"
        )

        rate.sleep()


def handle_response(response):
    # Handle responses from the robot
    rospy.loginfo(f"Received response from Robot: {response}")


if __name__ == "__main__":
    try:
        charging_station_node(
            stationID=random.randint(1, 100),
            request_interval=random.randint(1, 5),
        )
    except rospy.ROSInterruptException:
        pass
