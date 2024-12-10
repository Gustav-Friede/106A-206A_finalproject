#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix

def transform_pose(pose_in):
    """
    Transforms the input PoseStamped to a new coordinate system with scaling, translation, and rotation.
    """
    # Define transformation parameters
    # Scaling factors
    s_x, s_y, s_z = 2, 3, 1  # Input system scale
    s_x2, s_y2, s_z2 = 1, 2, 1  # Target system scale

    # Translation (shifting the origin)
    T_x, T_y, T_z = -2, 4, 1  # Translation of origin in the target system

    # Rotation angles (in degrees)
    theta_x, theta_y, theta_z = 0, 0, 45  # Rotation around x, y, z axes

    # Convert angles to radians
    theta_x_rad = np.radians(theta_x)
    theta_y_rad = np.radians(theta_y)
    theta_z_rad = np.radians(theta_z)

    # Build a 3D rotation matrix using Euler angles
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta_x_rad), -np.sin(theta_x_rad)],
                    [0, np.sin(theta_x_rad), np.cos(theta_x_rad)]])
    
    R_y = np.array([[np.cos(theta_y_rad), 0, np.sin(theta_y_rad)],
                    [0, 1, 0],
                    [-np.sin(theta_y_rad), 0, np.cos(theta_y_rad)]])
    
    R_z = np.array([[np.cos(theta_z_rad), -np.sin(theta_z_rad), 0],
                    [np.sin(theta_z_rad), np.cos(theta_z_rad), 0],
                    [0, 0, 1]])
    
    # Combined rotation matrix
    R = np.dot(np.dot(R_z, R_y), R_x)

    # Extract position from PoseStamped
    x1, y1, z1 = pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z

    # Scale the input position
    scaled_position = np.array([x1 * s_x2 / s_x, y1 * s_y2 / s_y, z1 * s_z2 / s_z])

    # Rotate the scaled position
    rotated_position = np.dot(R, scaled_position)

    # Translate the rotated position
    transformed_position = rotated_position + np.array([T_x, T_y, T_z])

    # Extract orientation (quaternion) and apply rotation
    qx, qy, qz, qw = pose_in.pose.orientation.x, pose_in.pose.orientation.y, pose_in.pose.orientation.z, pose_in.pose.orientation.w
    original_quaternion = np.array([qx, qy, qz, qw])
    original_rotation_matrix = quaternion_matrix(original_quaternion)[:3, :3]
    combined_rotation_matrix = np.dot(R, original_rotation_matrix)

    # Convert the combined rotation matrix back to quaternion
    transformed_quaternion = quaternion_from_matrix(np.vstack([np.hstack([combined_rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))

    print(transformed_quaternion)
    # Create a new PoseStamped message
    pose_out = PoseStamped()
    pose_out.header = pose_in.header
    pose_out.pose.position.x = transformed_position[0]
    pose_out.pose.position.y = transformed_position[1]
    pose_out.pose.position.z = transformed_position[2]
    pose_out.pose.orientation.x = transformed_quaternion[0]
    pose_out.pose.orientation.y = transformed_quaternion[1]
    pose_out.pose.orientation.z = transformed_quaternion[2]
    pose_out.pose.orientation.w = transformed_quaternion[3]

    return pose_out

def callback(odom_in):
    """
    Callback function to transform incoming Odometry messages.
    """

    def callback(odom_in):
        rospy.loginfo("Callback triggered! Received message on /odom")


    rospy.loginfo("Received Odometry: x: %f, y: %f, z: %f", 
                  odom_in.pose.pose.position.x, 
                  odom_in.pose.pose.position.y, 
                  odom_in.pose.pose.position.z)
    # Extract Pose from Odometry
    pose_in = PoseStamped()
    pose_in.header = odom_in.header
    pose_in.pose = odom_in.pose.pose

    # Transform the pose
    pose_out = transform_pose(pose_in)
    
    # Log the transformed pose
    rospy.loginfo("Transformed Odometry Pose: x: %f, y: %f, z: %f", 
                  pose_out.pose.position.x, 
                  pose_out.pose.position.y, 
                  pose_out.pose.position.z)
    
    # Publish the transformed pose
    pose_pub.publish(pose_out)
    print(pose_out)
    rospy.loginfo("Debugging: Your message here")
    rospy.logwarn("Warning: Your message here")
    rospy.logerr("Error: Your message here")


def listener():
    """
    Initializes the ROS node, subscribes to /odom, and publishes transformed odometry to /output_pose.
    """
    rospy.init_node('real_world_odometry_transformer', anonymous=True)

    # Subscriber for input Odometry messages
    rospy.Subscriber('/odom', Odometry, callback)

    # Publisher for transformed PoseStamped messages
    global pose_pub
    pose_pub = rospy.Publisher('/output_pose', PoseStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
