#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def model_states_callback(msg):
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

    print("Position of 'your_model_name': x={}, y={}, z={}".format(x, y, z))
    #rospy.loginfo("Position of 'your_model_name': x={}, y={}, z={}".format(x, y, z))

def main():
    rospy.init_node('model_position_listener', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
