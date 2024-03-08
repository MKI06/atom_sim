#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion
import numpy as np
import time

class Rover:

    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.yaw = None  # Yaw angle
        self.target_position_x = None
        self.target_position_y = None
        self.angle_to_target = None

    def main(self):
        rospy.init_node('model_position_listener', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.spin()

    def model_states_callback(self, msg):
        try:
            model_index = msg.name.index('atom')
        except ValueError:
            rospy.logerr("Model '{}' not found in ModelStates message".format('atom'))
            return

        model_pose = msg.pose[model_index]
        self.x = model_pose.position.x
        self.y = model_pose.position.y
        self.z = model_pose.position.z

        # Extracting quaternion orientation and converting it to Euler angles
        orientation_q = model_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

        rospy.loginfo("Position of 'your_model_name': x={:5f}, y={:5f}, z={:5f}, yaw={:5f}".format(self.x, self.y, self.z, self.yaw))
        time.sleep(1)
        self.calculate_angle()
        rospy.loginfo(self.angle_to_target)
        self.cmd_vel = Twist()
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)

    def set_target_position(self, x, y):
        self.target_position_x = x
        self.target_position_y = y

    def calculate_angle(self):
        self.set_target_position(5.0, 3.0)
        dx = self.target_position_x - self.x
        dy = self.target_position_y - self.y
        self.angle_to_target = np.deg2rad(atan2(dy, dx)) 

if __name__ == '__main__':
    controller = Rover()
    controller.main()
