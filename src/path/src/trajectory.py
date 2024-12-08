import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import sys
import os
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import a_star_Gutsav_v4 as a_star


class TrajectoryPlanner:
    def __init__(self):
        # Initialize the node
        rospy.init_node('trajectory_planner', anonymous=True)

        # Subscriber to the /map topic
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Subscriber to the /goal_pose topic
        self.goal_subscriber = rospy.Subscriber('/goal_pose', PoseStamped, self.goal_callback)

        self.end_subscriber = rospy.Subscriber('/end_pose', PoseStamped, self.end_callback)


        # Publisher for the /trajectory topic
        self.trajectory_publisher = rospy.Publisher('/trajectory', Path, queue_size=10)

        # Internal storage for map and goal
        self.current_map = None
        self.goal_pose = None
        self.end_pose = None


        self.last_published_path = None

        rospy.loginfo("TrajectoryPlanner node initialized.")

    def map_callback(self, map_msg):
        """
        Callback function for the /map topic. Stores the received map
        and triggers trajectory planning if a goal is available.

        :param map_msg: The OccupancyGrid message from the /map topic.
        """
        rospy.loginfo("Received a map message.")
        self.current_map = map_msg

        if self.goal_pose and self.end_pose:
            self.plan_and_publish_trajectory()
    
    def goal_callback(self, goal_msg):
        """
        Callback function for the /goal_pose topic. Stores the received goal
        and triggers trajectory planning if a map is available.

        :param goal_msg: The PoseStamped message from the /goal_pose topic.
        """
        rospy.loginfo("Received a goal pose.")
        self.goal_pose = goal_msg
        '''
        if self.current_map:
            self.plan_and_publish_trajectory()
        '''
    def end_callback(self, end_msg):
        """
        Callback function for the /goal_pose topic. Stores the received goal
        and triggers trajectory planning if a map is available.

        :param goal_msg: The PoseStamped message from the /goal_pose topic.
        """
        rospy.loginfo("Received a end pose.")
        self.end_pose = end_msg
        '''
        if self.current_map:
            self.plan_and_publish_trajectory()
        '''
    def plan_and_publish_trajectory(self):
        """
        Plans a trajectory based on the current map and goal, and publishes it.
        """
        if not self.current_map or not self.goal_pose:
            rospy.logwarn("Map or goal pose is missing. Cannot plan trajectory.")
            return

        # Generate the trajectory
        trajectory = self.generate_trajectory(self.current_map, self.goal_pose, self.end_pose)

        # Publish the trajectory
        # Check if the new trajectory is different from the last published trajectory
        if not self.trajectory_is_same(trajectory, self.last_published_path):
            # Publish the trajectory
            self.trajectory_publisher.publish(trajectory)
            rospy.loginfo("Published trajectory.")

            # Update the last published trajectory
            self.last_published_path = trajectory

        #rospy.sleep(5)        
        #input("Press Enter to plan and publish the next trajectory...")

    def trajectory_is_same(self, new_trajectory, old_trajectory):
        """
        Compares two trajectories to check if they are the same.
        This comparison is based on the waypoint coordinates.

        :param new_trajectory: The new trajectory to compare.
        :param old_trajectory: The last published trajectory to compare against.
        :return: True if the trajectories are the same, False if they are different.
        """
        if old_trajectory is None:
            return False

        # Compare the number of waypoints
        if len(new_trajectory.poses) != len(old_trajectory.poses):
            return False

        # Compare the positions of each waypoint
        for new_point, old_point in zip(new_trajectory.poses, old_trajectory.poses):
            if (new_point.pose.position.x != old_point.pose.position.x or
                new_point.pose.position.y != old_point.pose.position.y):
                return False

        return True

    def generate_trajectory(self, map_msg, goal_pose, end_pose):
        """
        Generates a trajectory based on the input map and goal.

        :param map_msg: The OccupancyGrid message.
        :param goal_pose: The PoseStamped message representing the goal.
        :return: A nav_msgs/Path message.
        """
        path = Path()
        path.header = map_msg.header

        # TODO: Replace with actual algorithm to generate the path
        a_star.plot(map_msg,np.array([int(end_pose.pose.position.x),int(end_pose.pose.position.y)]),np.array([int(goal_pose.pose.position.x),int(goal_pose.pose.position.y)]))
        way_points = a_star.a_star(map_msg,np.array([int(end_pose.pose.position.x),int(end_pose.pose.position.y)]),np.array([int(goal_pose.pose.position.x),int(goal_pose.pose.position.y)]))
        #print(way_points)

        for i in range(len(way_points)):
            point = PoseStamped()
            point.header = map_msg.header
            point.header.stamp = rospy.Time.now()
            point.pose.position.x = way_points[i].x
            point.pose.position.y = way_points[i].y
            path.poses.append(point)

        # Add the goal as the last waypoint for demonstration
        '''
        goal_waypoint = PoseStamped()
        goal_waypoint.header = goal_pose.header
        goal_waypoint.pose = goal_pose.pose
        path.poses.append(goal_waypoint)
        '''
        path2 = Path()
        
        if path != path2:
            path2 = path
            return path


def main():
    try:
        trajectory_planner = TrajectoryPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down TrajectoryPlanner node.")


if __name__ == '__main__':
    main()
