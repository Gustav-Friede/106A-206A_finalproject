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
    script_dir = os.path.dirname(__file__)
    image_path = os.path.join(script_dir,
                              '..', '..',
                              'computer_vision',
                              'data',
                              'camera_calibration',
                               'sat_0.png')

    maze_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    MIN_THRESH_VALUE = 120
    MAX_THRESH_VALUE = 255

    # extract edges and remove background
    edges = cv2.Canny(maze_image, MIN_THRESH_VALUE, MAX_THRESH_VALUE)
    img_wc = edges.copy()  # for visual purpose
    krn = cv2.getStructuringElement(cv2.MORPH_RECT, (16, 16))
    dlt = cv2.dilate(edges, krn, iterations=5)
    res = 255 - cv2.bitwise_and(dlt, img_wc)

    desired_size = 700
    maze_image = cv2.resize(res, (desired_size, desired_size), interpolation=cv2.INTER_NEAREST)

    # Process the image into a binary grid
    _, binary_grid = cv2.threshold(maze_image, 128, 255, cv2.THRESH_BINARY)

    # Convert binary grid to OccupancyGrid format (0 for free, 100 for occupied)
    occupancy_data = np.where(binary_grid == 0, 100, 0).flatten().tolist()

    # Define the OccupancyGrid message
    occupancy_grid = OccupancyGrid()


    # resolution based on real-world dimensions:
    #     #  board is 1.524 m on each side, and we have 700 pixels
    #     # resolution = total_meters / total_pixels
    resolution = 1.524 / desired_size

    # Create the OccupancyGrid message
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "map"

    occupancy_grid.info.resolution = resolution
    occupancy_grid.info.width = desired_size
    occupancy_grid.info.height = desired_size
    occupancy_grid.info.origin = Pose(Point(0.0, 0.0, 0.0),
                                      Quaternion(0.0, 0.0, 0.0, 1.0))

    occupancy_grid.data = occupancy_data

    return occupancy_grid

def occupancy_grid_publisher():
    # Initialize the ROS node
    rospy.init_node("occupancy_grid_publisher", anonymous=True)

    # Create a publisher for the OccupancyGrid
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)

    # Publish the grid at a fixed rate
    rate = rospy.Rate(1)  # 1 Hz
    rospy.loginfo("Publishing the Occupancy Grid with scaling...")

    while not rospy.is_shutdown():
        occupancy_grid = create_occupancy_grid()  # Generate the OccupancyGrid
        pub.publish(occupancy_grid)  # Publish the grid
        rate.sleep()

if __name__ == "__main__":
    try:
        occupancy_grid_publisher()
    except rospy.ROSInterruptException:
        pass