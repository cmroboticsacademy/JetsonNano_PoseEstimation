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
    speed = 0.15
    
    # TODO: estimate control actions
    cmd_msg = Float32MultiArray()
    pose_mat = np.array(msg.pose.matrix)
    relative_x, relative_y, tag_id = pose_mat[0], pose_mat[1], pose_mat[2]
    
    print(relative_x, relative_y, tag_id)
    
    print("tag_id: {}, tags_array: {}, target_id: {}, visited_tags: {}".format(tag_id, tags_array, target_id, visited_tags))
    
    if tag_id in tags_array and target_id == -1 and tag_id not in visited_tags:
        print("Tag ID: {} found.".format(tag_id))
        target_id = tag_id
        print("target_id is: {}".format(target_id))
    

    if target_id == tag_id and target_id != -1: 
        x_left_bound = -0.06
        x_right_bound = -0.01
        
        if relative_x >= x_left_bound and relative_x <= x_right_bound:
            print("Centered")
            if relative_y > 0.07:
                print("Moving closer...")
                cmd_msg.data = [move, speed, speed]
            else:
                print("Stopped")
                cmd_msg.data = [stop, 0, 0]
                visited_tags.append(target_id) # Add this tag to the visited_tags list
                target_id = -1 # Reset the target_id variable
        
        elif relative_x > x_left_bound:
            print("Adjusting to the left...")
            cmd_msg.data = [move, -speed*1.25, speed*1.25]
            
        elif relative_x < x_right_bound:
            print("Adjusting to the right...")
            cmd_msg.data = [move, speed*1.25, -speed*1.25]
    elif sorted(tags_array) == sorted(visited_tags):
        cmd_msg.data = [stop, 0, 0]
        print("All done")
    else:
        cmd_msg.data = [move, speed, -speed]
        ctrl_pub.publish(cmd_msg)
    
    ctrl_pub.publish(cmd_msg)
    print("Visited tags: {}".format(visited_tags))


if __name__ == "__main__":
    visited = False
    turn_flag = True
    target_id = -1
    tags_array = [0.0,1.0,2.0,3.0] 
    visited_tags = []
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
