#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import atan2, sqrt

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x, y, theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()
r = rospy.Rate(4)

goal = Point()
goal.x = 3  # Setting the goal x coordinate to 3
goal.y = 0  # Keeping the goal y coordinate as 0

try:
    while not rospy.is_shutdown():
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2(inc_y, inc_x)
        angle_error = angle_to_goal - theta

        distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)

        if abs(distance_to_goal) > 0.1:
            speed.linear.x = 0.5
            speed.angular.z = 0.3 if angle_error > 0 else -0.3
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()

except KeyboardInterrupt:
    # If you press Ctrl+C, stop the robot immediately
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)
