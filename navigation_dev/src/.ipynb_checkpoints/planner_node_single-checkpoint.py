#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np

ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)


def pose_callback(msg):
    global target_id
    move = 0.0
    stop = 1.0
    speed = 0.20
    
    # TODO: estimate control actions
    cmd_msg = Float32MultiArray()
    pose_mat = np.array(msg.pose.matrix)
    relative_x, relative_y, tag_id = pose_mat[0], pose_mat[1], pose_mat[2]
    
    print(relative_x, relative_y, tag_id)
    
    print("tag_id: {}".format(tag_id))
    
    if tag_id == 1: 
        if relative_y > 0.28:
            print("Moving closer...")
            cmd_msg.data = [move, speed, speed]
        else:
            print("Stopped")
            cmd_msg.data = [stop, 0, 0]
    else:
        cmd_msg.data = [stop, 0, 0]
        ctrl_pub.publish(cmd_msg)
    
    ctrl_pub.publish(cmd_msg)


if __name__ == "__main__":
    visited = False
    turn_flag = True
    target_id = -1
    tags_array = [0.0,1.0,2.0,3.0] 
    visited_tags = []
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
