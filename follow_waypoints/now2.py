#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from math import atan2

class RoverController:
    def __init__(self):
        rospy.init_node('rover_controller', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.target_position = Point()
        self.current_position = Point()
        self.reached_threshold = 0.1

        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Publisher('/my_odom', Odometry)
        
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def model_states_callback(self, msg):
    # Find the index of the model in the ModelStates message
        try:
            model_index = msg.name.index('atom')
        except ValueError:
            rospy.logerr("Model '{}' not found in ModelStates message".format('atom'))
            return

        # Extract pose information of the model
        model_pose = msg.pose[model_index]
        x = model_pose.position.x
        y = model_pose.position.y
        z = model_pose.position.z
        print("Position of 'atom': x={}, y={}, z={}".format(x, y, z))


    def set_target_position(self, x, y):
        self.target_position.x = x
        self.target_position.y = y

    def calculate_angle(self):
        # Calculate differences in x and y components
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y

        # Calculate angle towards the target using arctangent of ratio of y to x
        angle_to_target = atan2(abs(dy), abs(dx))
        print(angle_to_target)
        return angle_to_target
    
    def main(self):
        rospy.init_node('model_position_listener', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.spin()

    def control_loop(self):
        while not rospy.is_shutdown():
            angle_to_target = self.calculate_angle()

            if abs(angle_to_target - self.current_position.z) > 0.05:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0  # no linear velocity
                cmd_vel.angular.z = 0.3 * (angle_to_target - self.current_position.z)  # adjust angular velocity

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
        controller.main()
        #controller.set_target_position(5.0, 3.0)  # Set your desired position here
        #controller.control_loop()
        
    except rospy.ROSInterruptException:
        pass
