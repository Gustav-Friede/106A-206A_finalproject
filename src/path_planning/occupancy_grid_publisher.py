#!/usr/bin/env python3
import os

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import cv2

def create_occupancy_grid():
    # Load the binary maze image
    #image_path = "/home/cc/ee106a/fa24/class/ee106a-acj/final_project/imgs/birds-view-maze.jpg"  # Replace with the path to your maze image
    script_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', 'camera_snapshots', 'snapshot_000.png')
    image_path = cv2.imread(img_path, cv2.IMREAD_REDUCED_GRAYSCALE_8)

    maze_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Process the image into a binary grid
    _, binary_grid = cv2.threshold(maze_image, 128, 255, cv2.THRESH_BINARY)

    # Convert binary grid to OccupancyGrid format (0 for free, 100 for occupied)
    occupancy_data = np.where(binary_grid == 0, 100, 0)  # Black pixels -> 100, White pixels -> 0
    occupancy_data = occupancy_data.flatten().tolist()  # Flatten 2D array to 1D list

    # Define the OccupancyGrid message
    occupancy_grid = OccupancyGrid()

    # Set the header
    occupancy_grid.header = Header()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "map"  # Frame of reference for the map

    # Set the map info
    occupancy_grid.info.resolution = 1.0  # Each grid cell represents 1x1 meters
    occupancy_grid.info.width = maze_image.shape[1]  # Number of columns in the grid
    occupancy_grid.info.height = maze_image.shape[0]  # Number of rows in the grid

    # Set the origin of the map (bottom-left corner of the grid in the world frame)
    occupancy_grid.info.origin = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    # Assign the processed grid data
    occupancy_grid.data = occupancy_data

    return occupancy_grid

def occupancy_grid_publisher():
    # Initialize the ROS node
    rospy.init_node("occupancy_grid_publisher", anonymous=True)

    # Create a publisher for the OccupancyGrid
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)

    # Publish the grid at a fixed rate
    rate = rospy.Rate(1)  # 1 Hz
    rospy.loginfo("Publishing the Occupancy Grid...")

    while not rospy.is_shutdown():
        occupancy_grid = create_occupancy_grid()  # Generate the OccupancyGrid
        pub.publish(occupancy_grid)  # Publish the grid
        rate.sleep()

if __name__ == "__main__":
    try:
        occupancy_grid_publisher()
    except rospy.ROSInterruptException:
        pass