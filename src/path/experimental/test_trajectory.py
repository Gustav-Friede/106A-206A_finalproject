#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def generate_rectangle_path():
    """Generates a rectangular path with 100x100 in the xy-plane."""
    path = Path()
    path.header = Header()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "map"  # Adjust frame if necessary

    # Define the rectangle corners
    points = [
        (0, 0),    # Bottom-left corner
        (1, 0),  # Bottom-right corner
        (1, 1),  # Top-right corner
        (0, 1),  # Top-left corner
    ]

    # Create PoseStamped messages for each corner
    for x, y in points:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0  # z-plane is fixed to 0
        pose.pose.orientation.w = 1  # No rotation (quaternion default)

        path.poses.append(pose)

    return path


def rectangle_trajectory_publisher():
    rospy.init_node("rectangle_trajectory_node", anonymous=True)
    trajectory_pub = rospy.Publisher("trajectory", Path, queue_size=10)

    rate = rospy.Rate(1)  # Publish at 1 Hz
    rospy.loginfo("Publishing rectangular trajectory...")

    while not rospy.is_shutdown():
        path = generate_rectangle_path()
        trajectory_pub.publish(path)
        rate.sleep()


if __name__ == "__main__":
    try:
        rectangle_trajectory_publisher()
    except rospy.ROSInterruptException:
        pass
