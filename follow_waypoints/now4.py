#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from math import atan2
import time

class Rover:

    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.target_position_x = None
        self.target_position_y = None
        self.angle_to_target = None
        self.cmd_vel = None


    def main(self):
        rospy.init_node('model_position_listener', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.spin()

    def model_states_callback(self, msg):
        # Find the index of the model in the ModelStates message
        try:
            model_index = msg.name.index('atom')
        except ValueError:
            rospy.logerr("Model '{}' not found in ModelStates message".format('atom'))
            return

        # Extract pose information of the model
        model_pose = msg.pose[model_index]
        self.x = model_pose.position.x
        self.y = model_pose.position.y
        self.z = model_pose.position.z
        rospy.loginfo("Position of 'your_model_name': x={}, y={}, z={}".format(self.x, self.y, self.z))
        self.calculate_angle()
        rospy.loginfo(self.angle_to_target)
        self.cmd_vel = Twist()
        self.cmd_vel.angular.z = 0.3
        self.cmd_vel_pub.publish(self.cmd_vel)

    def set_target_position(self, x, y):
        self.target_position_x = x
        self.target_position_y = y

    def calculate_angle(self):
        self.set_target_position(5.0, 3.0)
        # Calculate differences in x and y components
        dx = self.target_position_x - self.x
        dy = self.target_position_y - self.y

        # Calculate angle towards the target using arctangent of ratio of y to x
        self.angle_to_target = atan2(abs(dy), abs(dx))
        #print("Position of 'your_model_name': x={}, y={}, z={}".format(self.x, self.y, self.z))
        #print(angle_to_target)
        
    

if __name__ == '__main__':
    controller = Rover()
    controller.main()
    

    