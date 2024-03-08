#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import sqrt

class RoverController:
    def __init__(self):
        rospy.init_node('rover_controller', anonymous=True)
        self.rate = rospy.Rate(10)  # 4 Hz

        self.target_position = Point()
        self.current_position = Point()
        self.reached_threshold = 0.1

        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def set_target_position(self, x, y):
        self.target_position.x = x
        self.target_position.y = y

    def distance_to_target(self):
        return sqrt((self.target_position.x - self.current_position.x) ** 2 +
                    (self.target_position.y - self.current_position.y) ** 2)

    def control_loop(self):
        while not rospy.is_shutdown():
            distance = self.distance_to_target()

            if distance > self.reached_threshold:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.5  # linear velocity
                cmd_vel.angular.z = 0.0  # no angular velocity

                self.cmd_vel_pub.publish(cmd_vel)
            else:
                rospy.loginfo("Target reached!")
                cmd_vel = Twist()  # Stop the rover
                self.cmd_vel_pub.publish(cmd_vel)
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RoverController()
        controller.set_target_position(2.0, 0.0)  # Set your desired position here
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
