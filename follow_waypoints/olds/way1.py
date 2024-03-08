#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64  # Assuming you're sending a set point as a float

def send_set_point():
    rospy.init_node('set_point_publisher', anonymous=True)
    set_point_publisher = rospy.Publisher('/your/set_point/topic', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz, adjust as needed

    while not rospy.is_shutdown():
        # Publish your set point value here
        set_point_value = 1.0  # Example set point value
        set_point_publisher.publish(set_point_value)
        print("Set point published:", set_point_value)  # Add a print statement to verify if it enters the loop
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Initializing node...")
        send_set_point()
        print("Node initialized and running.")
    except rospy.ROSInterruptException:
        pass
    print("Exiting script.")
