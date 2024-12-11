#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np

from solve_maze import a_star

class TrajectoryPlanner:
    def __init__(self):
        # intitialize the node
        rospy.init_node('trajectory_planner', anonymous=True)

        # subscribers
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber('/camera_pixel_ar_tags', PointStamped, self.goal_callback)
        self.end_subscriber = rospy.Subscriber('/camera_pixel_ar_tags', PointStamped, self.end_callback)

        # publisher
        self.trajectory_publisher = rospy.Publisher('/grid_trajectory', Path, queue_size=10)

        # internal state
        self.current_map = None
        self.goal_point = None
        self.end_point = None
        self.last_published_path = None

        rospy.loginfo("TrajectoryPlanner node initialized.")

    def map_callback(self, map_msg):
        rospy.loginfo("Received a map message.")
        self.current_map = map_msg
        self.try_to_plan_trajectory()

    def goal_callback(self, goal_msg):
        if int(goal_msg.header.frame_id) == 0:
            rospy.loginfo("Received a goal point.")
            self.goal_point = goal_msg
            self.try_to_plan_trajectory()

    def end_callback(self, end_msg):
        if int(end_msg.header.frame_id) == 1:
            rospy.loginfo("Received an end point.")
            self.end_point = end_msg
            self.try_to_plan_trajectory()

    def try_to_plan_trajectory(self):
        if self.current_map and self.goal_point and self.end_point:
            self.plan_and_publish_trajectory()

    def plan_and_publish_trajectory(self):
        if not self.current_map or not self.goal_point or not self.end_point:
            rospy.logwarn("Cannot plan trajectory. Map, goal, or end point is missing.")
            return

        path_msg = self.generate_trajectory(self.current_map, self.goal_point, self.end_point)

        if not self.trajectory_is_same(path_msg, self.last_published_path):
            self.trajectory_publisher.publish(path_msg)
            rospy.loginfo("Published trajectory.")
            self.last_published_path = path_msg
        else:
            rospy.loginfo("New trajectory is identical to the last one. Not republishing.")

    def trajectory_is_same(self, new_trajectory, old_trajectory):
        if old_trajectory is None:
            return False

        if len(new_trajectory.poses) != len(old_trajectory.poses):
            return False

        for new_point, old_point in zip(new_trajectory.poses, old_trajectory.poses):
            if (new_point.pose.position.x != old_point.pose.position.x or
                new_point.pose.position.y != old_point.pose.position.y):
                return False

        return True

    def generate_trajectory(self, map_msg, goal_point, end_point):
        path_msg = Path()
        path_msg.header = map_msg.header


        goal_x = int(goal_point.point.x)
        goal_y = int(goal_point.point.y)
        end_x = int(end_point.point.x)
        end_y = int(end_point.point.y)

        # run A* from solve_maze.py
        waypoints = a_star(map_msg, (end_x, end_y), (goal_x, goal_y))

        if waypoints is None:
            rospy.logwarn("No path found by A*.")
            return path_msg

        # Convert waypoints to PoseStamped
        for (wx, wy) in waypoints:
            point = PoseStamped()
            point.header = map_msg.header
            point.header.stamp = rospy.Time.now()
            point.pose.position.x = wx
            point.pose.position.y = wy
            path_msg.poses.append(point)

        return path_msg

def main():
    try:
        planner = TrajectoryPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down TrajectoryPlanner node.")

if __name__ == '__main__':
    main()
