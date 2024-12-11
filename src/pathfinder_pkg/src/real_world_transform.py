#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix

def transform_pose(pose_in):
    """
    Transforms the input PoseStamped to a new coordinate system with scaling, translation, and rotation.
    """
    # Define transformation parameters
    # Scaling factors
    s_x, s_y = 350, 350  # Input system scale
    s_x2, s_y2 = 1.6, 1.575  # Target system scale

    # Translation (shifting the origin)
    T_x, T_y = -1, -0.14#(1.5/350)  # Translation of origin in the target system

    # Rotation angles (in degrees)
    theta = 0  # Rotation around x, y, z axes

    # Convert angles to radians
    theta_rad = np.radians(theta)

    # Build a 2D rotation matrix using Euler angles
    R = np.array([[np.cos(theta_rad), -np.sin(theta_rad)],
                  [np.sin(theta_rad), np.cos(theta_rad)]])
    
    # Extract position from PoseStamped
    x1, y1 = pose_in.pose.position.x, pose_in.pose.position.y

    # Scale the input position
    scaled_position = np.array([x1 * s_x2 / s_x, y1 * s_y2 / s_y])

    # Rotate the scaled position
    rotated_position = np.dot(R, scaled_position)

    # Translate the rotated position
    transformed_position = rotated_position + np.array([T_x, T_y])
    '''
    # Extract orientation (quaternion) and apply rotation
    qx, qy, qz, qw = pose_in.pose.orientation.x, pose_in.pose.orientation.y, pose_in.pose.orientation.z, pose_in.pose.orientation.w
    original_quaternion = np.array([qx, qy, qz, qw])
    original_rotation_matrix = quaternion_matrix(original_quaternion)[:3, :3]
    combined_rotation_matrix = np.dot(R, original_rotation_matrix)

    # Convert the combined rotation matrix back to quaternion
    transformed_quaternion = quaternion_from_matrix(np.vstack([np.hstack([combined_rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))

    print(transformed_quaternion)
    '''
    # Create a new PoseStamped message
    pose_out = PoseStamped()
    pose_out.header = pose_in.header
    pose_out.pose.position.x = transformed_position[0]
    pose_out.pose.position.y = transformed_position[1]
    '''
    pose_out.pose.position.z = transformed_position[2]
    pose_out.pose.orientation.x = transformed_quaternion[0]
    pose_out.pose.orientation.y = transformed_quaternion[1]
    pose_out.pose.orientation.z = transformed_quaternion[2]
    pose_out.pose.orientation.w = transformed_quaternion[3]
    '''

    return pose_out

def callback(path_in):
    """
    Callback function to transform all poses in the Path message.
    """
    rospy.loginfo("Received Path with %d poses", len(path_in.poses))

    # Create a new Path message
    path_out = Path()
    path_out.header = path_in.header

    # Transform each pose in the input Path
    for pose in path_in.poses:
        transformed_pose = transform_pose(pose)
        path_out.poses.append(transformed_pose)
    
    # Publish the transformed Path
    path_pub.publish(path_out)
    rospy.loginfo("Published transformed Path with %d poses", len(path_out.poses))


def listener():
    """
    Initializes the ROS node, subscribes to /grid_trajectory, and publishes transformed Path to /output_path.
    """
    rospy.init_node('path_transformer', anonymous=True)

    # Subscriber for input Path messages
    rospy.Subscriber('/grid_trajectory', Path, callback)

    # Publisher for transformed Path messages
    global path_pub
    path_pub = rospy.Publisher('/trajectory', Path, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
