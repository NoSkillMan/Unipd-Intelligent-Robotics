#!/usr/bin/env python3

import rospy
from vaccuum_robot.msg import Info
import random

ROOMS = {
    1: "Robot Vision Lab",
    2: "SSL Lab",
    3: "Neurorobatics Lab",
    4: "IAS Lab",
    5: "Autonomus Robotics Lab",
}


def robot_node():
    rospy.init_node("robot_node", anonymous=False)
    pub = rospy.Publisher("robot_info", Info, queue_size=10)
    rate = rospy.Rate(5)  # 5Hz

    while not rospy.is_shutdown():
        room_index = random.randint(1, 5)
        info = Info()
        info.room_name = ROOMS[room_index]
        info.room_id = room_index
        info.battery_level = random.randint(0, 100)
        pub.publish(info)
        rospy.loginfo(
            f"Robot is in {info.room_name} (ID: {info.room_id}), Battery Level: {info.battery_level}%"
        )

        rate.sleep()


if __name__ == "__main__":
    try:
        robot_node()
    except rospy.ROSInterruptException:
        pass
