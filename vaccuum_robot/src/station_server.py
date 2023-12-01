#!/usr/bin/env python3

import rospy
from vaccuum_robot.srv import Station, StationResponse
from vaccuum_robot.msg import Info
from std_msgs.msg import Header

import random

ROOMS = {
    1: "Robot Vision Lab",
    2: "SSL Lab",
    3: "Neurorobatics Lab",
    4: "IAS Lab",
    5: "Autonomus Robotics Lab",
}


def handle_station_info(req):
    rospy.loginfo(f"Received request from Charging Station {req.stationID}")

    # Simulate generating room information
    room_index = random.randint(1, 5)
    info = Info()
    info.room_name = ROOMS[room_index]
    info.room_id = room_index
    info.battery_level = random.randint(0, 100)

    # Create and return the response
    response_header = Header()
    response_header.stamp = rospy.Time.now()
    return StationResponse(header=response_header, info=info)


def robot_server_node():
    rospy.init_node("robot_server_node")

    # Create the service
    s = rospy.Service("station", Station, handle_station_info)
    rospy.loginfo("Robot is ready to send information.")

    # Create a publisher to broadcast responses to all charging stations
    pub = rospy.Publisher("robot_response", StationResponse, queue_size=10)

    rospy.Rate(5)  # Set the rate to 5Hz
    rospy.spin()


if __name__ == "__main__":
    try:
        robot_server_node()
    except rospy.ROSInterruptException:
        pass
