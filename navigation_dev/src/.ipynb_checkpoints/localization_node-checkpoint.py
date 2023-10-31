#!/usr/bin/env python

import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np

def extract_axis_angle_from_matrix(matrix):
    # Ensure matrix is square
    assert matrix.shape[0] == matrix.shape[1] == 3, "Matrix must be 3x3."
    
    # Compute the angle of rotation
    angle = np.arccos((np.trace(matrix) - 1) / 2)
    
    # Compute the rotation axis
    rx = matrix[2, 1] - matrix[1, 2]
    ry = matrix[0, 2] - matrix[2, 0]
    rz = matrix[1, 0] - matrix[0, 1]
    
    norm = np.linalg.norm([rx, ry, rz])
    if norm < 1e-10:
        # No rotation, return a default axis (0, 0, 1)
        return [0, 0, 1], 0
    else:
        rx /= norm
        ry /= norm
        rz /= norm
        
    return [rx, ry, rz], angle
    
pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)


def tag_callback(msg):
    # TODO: implement localization logic
    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    if msg.ids:
        tag_id = msg.ids[0]
        tm = msg.detections[0].matrix
        tm = np.array(tm).reshape(4, 4)
        origin = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        relative_pos = np.matmul(tm, origin).reshape(4)
        relative_x, relative_y = relative_pos[0], relative_pos[2]
        
        # Extract rotation matrix from the transformation matrix
        rotation_matrix = tm[:3, :3]
        axis, angle = extract_axis_angle_from_matrix(rotation_matrix)
        angle = np.degrees(angle)  # Convert angle to degrees
        pose_msg.pose.axis_angle = axis + [angle]
    
        pose_msg.pose.matrix = [relative_x, relative_y, tag_id]
        # TODO
        # pose_msg.pose = [R11, R12, R13, t1,
        #                  R21, R22, R23, t2,
        #                  R31, R32, R33, t3]
    else:
        pose_msg.pose.matrix = [0, 0, -1]
    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()

