#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class OccupancyGridPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.occupancy_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
        self.latest_hue = None

        # Subscribe to the hue image topic published by hsv_camera_feed node
        rospy.Subscriber("/hue_image", Image, self.hue_callback)

        # Publish at a fixed rate
        self.rate = rospy.Rate(1)  # 1 Hz

        rospy.loginfo("OccupancyGridPublisher node initialized.")
        rospy.loginfo("Subscribing to: /hue_image")
        rospy.loginfo("Publishing occupancy grid on: /map")

    def hue_callback(self, msg):
        # Convert ROS image to OpenCV image
        self.latest_hue = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def run(self):
        rospy.loginfo("Publishing the Occupancy Grid from hue image...")
        while not rospy.is_shutdown():
            if self.latest_hue is not None:
                occupancy_grid = self.create_occupancy_grid(self.latest_hue)
                self.occupancy_pub.publish(occupancy_grid)
            self.rate.sleep()

    def create_occupancy_grid(self, hue_image):
        MIN_THRESH_VALUE = 120
        MAX_THRESH_VALUE = 255

        # Process the hue image to generate edges and occupancy data
        edges = cv2.Canny(hue_image, MIN_THRESH_VALUE, MAX_THRESH_VALUE)
        img_wc = edges.copy()
        krn = cv2.getStructuringElement(cv2.MORPH_RECT, (16, 16))
        dlt = cv2.dilate(edges, krn, iterations=5)
        res = 255 - cv2.bitwise_and(dlt, img_wc)

        desired_size = 700
        maze_image = cv2.resize(res, (desired_size, desired_size), interpolation=cv2.INTER_NEAREST)

        _, binary_grid = cv2.threshold(maze_image, 128, 255, cv2.THRESH_BINARY)

        # Convert binary grid to OccupancyGrid data: 0 = free, 100 = occupied
        occupancy_data = np.where(binary_grid == 0, 100, 0).flatten().tolist()

        # resolution: board is 1.524 m each side, and we have 700 pixels
        # resolution = total_meters / total_pixels
        resolution = 1.524 / desired_size

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


if __name__ == "__main__":
    rospy.init_node("occupancy_grid_publisher", anonymous=True)
    node = OccupancyGridPublisher()
    node.run()
