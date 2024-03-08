#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def send_set_point():
    rospy.init_node('rover_control_node', anonymous=True)
    pub = rospy.Publisher('/rover/set_point', Point, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    target_point = Point()
    target_point.x = 2.0  # Set your desired x-coordinate
    target_point.y = 3.0  # Set your desired y-coordinate
    target_point.z = 0.0  # Assuming z-coordinate is 0 (ground level)

    while not rospy.is_shutdown():
        pub.publish(target_point)
        rospy.loginfo(f"Sending set point: {target_point.x}, {target_point.y}")
        rate.sleep()

if __name__ == '__main__':
    try:
        send_set_point()
    except rospy.ROSInterruptException:
        pass
