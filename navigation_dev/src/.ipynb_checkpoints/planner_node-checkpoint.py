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
    target_id = -1
    tags_array = [0,1,2,3] 
    visited_tags = []
    move = 0.0
    stop = 1.0
    speed = 0.17
    
    # TODO: estimate control actions
    cmd_msg = Float32MultiArray()
    pose_mat = np.array(msg.pose.matrix)
    relative_x, relative_y, tag_id = pose_mat[0], pose_mat[1], pose_mat[2]
    
    print(relative_x, relative_y, tag_id)
    
    if tag_id in tags_array and target_id != -1:
        print("Tag ID: {} found.".format(tag_id))
        target_id = tag_id
        
    if target_id: 
        x_left_bound = -0.06
        x_right_bound = -0.01
        
        if relative_x >= x_left_bound and relative_x <= x_right_bound:
            print("Centered")
            if relative_y > 0.08:
                print("Moving closer...")
                cmd_msg.data = [move, speed, speed]
            else:
                print("Stopped")
                cmd_msg.data = [stop, 0, 0]
                visited_tags.append(target_id)
        
        elif relative_x > x_left_bound:
            print("Adjusting to the left...")
            cmd_msg.data = [move, -speed, speed]
            
        elif relative_x < x_right_bound:
            print("Adjusting to the right...")
            cmd_msg.data = [move, speed, -speed]
    else:
        cmd_msg.data = [move, speed, -speed]
    
    ctrl_pub.publish(cmd_msg)


if __name__ == "__main__":
    visited = False
    turn_flag = True
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
