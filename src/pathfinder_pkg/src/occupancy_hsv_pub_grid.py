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

        # seperate publishers for each map channel
        self.occupancy_pub_hue = rospy.Publisher("/map_hue", OccupancyGrid, queue_size=10)
        self.occupancy_pub_sat = rospy.Publisher("/map_sat", OccupancyGrid, queue_size=10)
        self.occupancy_pub_val = rospy.Publisher("/map_val", OccupancyGrid, queue_size=10)

        self.latest_hue = None
        self.latest_sat = None
        self.latest_val = None

        # subscribe to all three HSV channel topics
        rospy.Subscriber("/hue_image", Image, self.hue_callback)
        rospy.Subscriber("/saturation_image", Image, self.saturation_callback)
        rospy.Subscriber("/value_image", Image, self.value_callback)

        # Publish at 1 Hz
        self.rate = rospy.Rate(1)

        rospy.loginfo("OccupancyGridPublisher node initialized.")
        rospy.loginfo("Subscribing to: /hue_image, /saturation_image, /value_image")
        rospy.loginfo("Publishing occupancy grids on: /map_hue, /map_sat, /map_val")

    def hue_callback(self, msg):
        self.latest_hue = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def saturation_callback(self, msg):
        self.latest_sat = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def value_callback(self, msg):
        self.latest_val = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def run(self):
        rospy.loginfo("Publishing the Occupancy Grid from HSV channels...")
        while not rospy.is_shutdown():
            # create and publish hue-based occupancy grid
            if self.latest_hue is not None:
                occupancy_grid_hue = self.create_occupancy_grid(self.latest_hue)
                self.occupancy_pub_hue.publish(occupancy_grid_hue)

            # create and publish saturation-based occupancy grid
            if self.latest_sat is not None:
                occupancy_grid_sat = self.create_occupancy_grid(self.latest_sat)
                self.occupancy_pub_sat.publish(occupancy_grid_sat)

            # create and publish value-based occupancy grid
            if self.latest_val is not None:
                occupancy_grid_val = self.create_occupancy_grid(self.latest_val)
                self.occupancy_pub_val.publish(occupancy_grid_val)

            self.rate.sleep()

    def create_occupancy_grid(self, channel_image):
        MIN_THRESH_VALUE = 120
        MAX_THRESH_VALUE = 255

        # process the channel image to generate edges and occupancy data
        edges = cv2.Canny(channel_image, MIN_THRESH_VALUE, MAX_THRESH_VALUE)
        img_wc = edges.copy()
        krn = cv2.getStructuringElement(cv2.MORPH_RECT, (16, 16))
        dlt = cv2.dilate(edges, krn, iterations=5)
        res = 255 - cv2.bitwise_and(dlt, img_wc)

        desired_size = 700
        maze_image = cv2.resize(res, (desired_size, desired_size), interpolation=cv2.INTER_NEAREST)

        _, binary_grid = cv2.threshold(maze_image, 128, 255, cv2.THRESH_BINARY)

        # convert binary grid to OccupancyGrid data: 0 = free, 100 = occupied
        occupancy_data = np.where(binary_grid == 0, 100, 0).flatten().tolist()

        # resolution: board is 1.524 m each side
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
