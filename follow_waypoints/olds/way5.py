#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import atan2

x = 2.0  # Setting initial x position to 2
y = 5.0  # Setting initial y position to 5
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
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x)
    angle_error = angle_to_goal - theta

    if abs(angle_error) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3 if angle_error > 0 else -0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()
