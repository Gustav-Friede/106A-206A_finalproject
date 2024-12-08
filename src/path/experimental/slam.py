import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
from math import cos, sin, pi
from typing import Optional


class SLAM:
    def __init__(self):
        self.current_map: Optional[OccupancyGrid] = None
        self.maze = self._generate_static_maze(100, 100)

        # Publisher for OccupancyGrid
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # Timer for regular map publication
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_map)

        rospy.loginfo("SLAM node initialized and publishing to '/map'.")
    def publish_map(self, event):
        """Publish the current map periodically."""
        # Define map parameters
        map_size: tuple[int, int] = (100, 100)
        resolution: float = 0.1  # 0.1 meters per grid cell
        map_origin: tuple[float, float] = (-5.0, -5.0)

        # Create the OccupancyGrid message
        occupancy_msg: OccupancyGrid = OccupancyGrid()
        occupancy_msg.header.frame_id = "map"
        occupancy_msg.header.stamp = rospy.Time.now()
        occupancy_msg.info.resolution = resolution
        occupancy_msg.info.width = map_size[0]
        occupancy_msg.info.height = map_size[1]
        occupancy_msg.info.origin = Pose()
        occupancy_msg.info.origin.position.x = map_origin[0]
        occupancy_msg.info.origin.position.y = map_origin[1]
        occupancy_msg.data = self.maze.flatten().tolist()

        # Publish the map
        self.map_publisher.publish(occupancy_msg)
        rospy.loginfo("Published map to '/map'.")

    """
    def __init__(self):
        self.current_map: Optional[OccupancyGrid] = None
        self.maze = self._generate_static_maze(100, 100)

        # Subscribe to /scan topic
        #self.subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        # Publisher for OccupancyGrid
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        #self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_map)

        rospy.loginfo("SLAM node initialized, subscribed to '/scan', and publishing to '/map'.")
    """
    
    def lidar_callback(self, msg: LaserScan) -> None:
        """Callback to process LaserScan data and generate an OccupancyGrid map."""
        
        if not msg.ranges:
            rospy.logwarn("No data in LaserScan message.")
            return

        # Define map parameters
        map_size: tuple[int, int] = (100, 100)
        resolution: float = 0.1  # 0.1 meters per grid cell
        map_origin: tuple[float, float] = (-5.0, -5.0)

        # Create an empty grid (unknown = -1)
        occupancy_grid: np.ndarray = np.full(map_size, -1, dtype=int)

        # Process LaserScan data into occupancy grid
        occupancy_grid = self._generate_map(occupancy_grid, msg)

        # Create the OccupancyGrid message
        occupancy_msg: OccupancyGrid = OccupancyGrid()
        occupancy_msg.header.frame_id = "map"
        occupancy_msg.header.stamp = rospy.Time.now()
        occupancy_msg.info.resolution = resolution
        occupancy_msg.info.width = map_size[0]
        occupancy_msg.info.height = map_size[1]
        occupancy_msg.info.origin = Pose()
        occupancy_msg.info.origin.position.x = map_origin[0]
        occupancy_msg.info.origin.position.y = map_origin[1]
        occupancy_msg.data = occupancy_grid.flatten().tolist()

        # Store and publish the generated map
        self.current_map = occupancy_msg
        self.map_publisher.publish(self.current_map)
        rospy.loginfo("Published updated map to '/map'.")

    def _generate_map(self, occupancy_grid: np.ndarray, scan: LaserScan) -> np.ndarray:
        """Generate a random maze and update the occupancy grid."""
        rospy.loginfo("Generated a random maze for the occupancy grid.")
        return self.maze

    def _generate_static_maze(self, grid_height: int, grid_width: int) -> np.ndarray:
        """Generate a static maze using the recursive division method."""
        # Create an empty grid (walls = 100, free = 0)
        maze = np.full((grid_height, grid_width), 0, dtype=int)
        
        # Add outer walls
        maze[0, :] = 100
        maze[-1, :] = 100
        maze[:, 0] = 100
        maze[:, -1] = 100
        x_pos = [np.arange(0, 90, 1), np.arange(10, 100, 1)]
        y_pos = np.array([33, 66])

        for i in range(len(y_pos)):
            for j in range(len(x_pos[i])):
                x = x_pos[i][j]
                y = y_pos[i]
                maze[y, x] = 100    #occupied
        '''
        def divide(x, y, width, height, orientation):
            """Recursive function to divide the grid."""
            if width <= 2 or height <= 2:
                return

            # Choose where to place the wall
            if orientation == 'horizontal':
                wall_y = y + (np.random.randint(0, height // 2) * 2) + 1
                passage_x = x + (np.random.randint(0, width // 2) * 2)
                maze[wall_y, x:x + width] = 100
                maze[wall_y, passage_x] = 0
                divide(x, y, width, wall_y - y, 'vertical')  # Top section
                divide(x, wall_y + 1, width, y + height - wall_y - 1, 'vertical')  # Bottom section
            else:
                wall_x = x + (np.random.randint(0, width // 2) * 2) + 1
                passage_y = y + (np.random.randint(0, height // 2) * 2)
                maze[y:y + height, wall_x] = 100
                maze[passage_y, wall_x] = 0
                divide(x, y, wall_x - x, height, 'horizontal')  # Left section
                divide(wall_x + 1, y, x + width - wall_x - 1, height, 'horizontal')  # Right section

        # Start the recursive division
        divide(0, 0, grid_width, grid_height, 'horizontal' if grid_width < grid_height else 'vertical')

        # Ensure the start and end points are open
        maze[1, 1] = 0  # Start
        maze[-2, -2] = 0  # End

        rospy.loginfo("Static maze generated using recursive division method.")
        '''
        return maze


def main():
    rospy.init_node('slam_node')
    slam_node = SLAM()

    rospy.loginfo("SLAM node is running. Subscribed to '/scan'.")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down SLAM node.")


if __name__ == "__main__":
    main()