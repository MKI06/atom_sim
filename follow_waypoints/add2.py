import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy as np
import time

class Rover:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.yaw = None
        self.target_position_x = None
        self.target_position_y = None
        self.angle_to_target = None

    def main(self):
        rospy.init_node('model_position_listener', anonymous=True)
        self.rate = rospy.Rate(10).sleep()  # 10 Hz
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.spin()

    def model_states_callback(self, msg):
        try:
            model_index = msg.name.index('atom')  # Assuming 'atom' is the name of your model
        except ValueError:
            rospy.logerr("Model 'atom' not found in ModelStates message")
            return

        model_pose = msg.pose[model_index]
        self.x = model_pose.position.x
        self.y = model_pose.position.y
        self.z = model_pose.position.z

        orientation_q = model_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

        rospy.loginfo("Position: x={:.5f}, y={:.5f}, z={:.5f}, yaw={:.5f}".format(self.x, self.y, self.z, self.yaw))
        
        # Calculate angle to target
        self.calculate_angle()
        rospy.loginfo("Angle to target: {:.5f}".format(self.angle_to_target))
        #time.sleep(1)
        # Publish command velocity
        self.publish_cmd_vel()

    def set_target_position(self, x, y):
        self.target_position_x = x
        self.target_position_y = y

    def calculate_angle(self):
        self.set_target_position(5.0, 3.0)
        dx = self.target_position_x - self.x
        dy = self.target_position_y - self.y
        self.angle_to_target = np.deg2rad(90) - np.arctan2(dy, dx)

    def publish_cmd_vel(self):
        cmd_vel = Twist()
        threshold = np.deg2rad(3)
        if np.abs(self.yaw - self.angle_to_target) < threshold:
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = 0.2

            if np.abs(self.target_position_y - self.y) < 0.2:
                cmd_vel.linear.x = 0.0

        else:
            cmd_vel.angular.z = 0.1            #cmd_vel.linear.x = 0.1
        self.cmd_vel_pub.publish(cmd_vel)
        

if __name__ == '__main__':
    controller = Rover()
    controller.main()
