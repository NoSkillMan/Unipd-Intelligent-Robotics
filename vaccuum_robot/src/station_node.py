#!/usr/bin/env python3

import rospy
from vaccuum_robot.msg import Info


def robot_info_callback(data):
    room_name = data.room_name
    room_id = data.room_id
    battery_level = data.battery_level
    rospy.loginfo(
        f"Robot is in {room_name} (ID: {room_id}), Battery Level: {battery_level}%"
    )


def station_node():
    rospy.init_node("charging_station_node", anonymous=False)
    rospy.Subscriber("robot_info", Info, robot_info_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        station_node()
    except rospy.ROSInterruptException:
        pass
