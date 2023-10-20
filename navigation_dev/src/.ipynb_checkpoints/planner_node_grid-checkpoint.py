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
    global target_id, turn_flag
    move = 0.0
    stop = 1.0
    speed = 0.20 
    
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
        x_left_bound = -0.065
        x_right_bound = -0.13
        
        if relative_x >= x_right_bound and relative_x <= x_left_bound:
            print("Centered")
            if relative_y > long_y_base-(target_row*long_y_inc):
                print("Moving closer...")
                cmd_msg.data = [move, speed, speed]
            else:
                print("Stopped")
                cmd_msg.data = [stop, 0, 0]
                visited_tags.append(target_id) # Add this tag to the visited_tags list
                target_id = -1 # Reset the target_id variable
        
        elif relative_x > x_right_bound:
            print("Adjusting to the left...")
            if turn_flag:
                cmd_msg.data = [move, -speed*1.25, speed*1.25]
            else:
                cmd_msg.data = [stop, 0, 0]
            turn_flag = not turn_flag
            
        elif relative_x < x_left_bound:
            print("Adjusting to the right...")  
            if turn_flag:
                cmd_msg.data = cmd_msg.data = [move, speed*1.25, -speed*1.25]
            else:
                cmd_msg.data = [stop, 0, 0]
            turn_flag = not turn_flag
            
    elif sorted(tags_array) == sorted(visited_tags):
        cmd_msg.data = [stop, 0, 0]
        print("All done")
    else:
        if turn_flag:
            cmd_msg.data = [move, speed*1.25, 0]
        else:
            cmd_msg.data = [stop, 0, 0]
        turn_flag = not turn_flag
    
    ctrl_pub.publish(cmd_msg)
    print("Visited tags: {}".format(visited_tags))


if __name__ == "__main__":
    visited = False
    turn_flag = True
    target_id = -1
    tags_array = [0.0,1.0] 
    visited_tags = []
    
    # Related to the target / goal
    target_row = 7
    target_column = 3
    long_y_base = 0.695 # This is the furthest rel_y that the robot sees from the april tag
    long_x_inc = 0.0412 
    long_y_inc = 0.0415 
    short_x_inc = 0.0289 
    short_x_inc = 0.0444
    
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
