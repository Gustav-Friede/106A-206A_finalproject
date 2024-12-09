#!/usr/bin/env python

#imports
import rospy
import tf2_ros
import tf
import sys
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from trajectory import plan_curved_trajectory

#added for subscribing
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped

#input value is a list of tuples representing waypoints in a 2D grid of 5x5

#transform waypoints from 2D to positions for the turtlebot to drive to
class turtlebotController:
  def __init__(self):
    rospy.init_node('turtlebot_controller', anonymous=True)

    #rospy.Subscriber("goal_point", Point, planning_callback, queue_size=10) 
    # subscriber type is a path msg

    self.goal_path_sub = rospy.Subscriber("/trajectory", Path, self.planning_callback, queue_size= 10) #subscribe to Gustav's a* algo
    #self.goal_point_sub = rospy.Subscriber("goal_point", PointStamped, self.ar_tag_callback, queue_size= 10) 

    self.path = None
    self.point = None
    self.ar_tag_position = None
    self.ar_tag_id = None
    self.goal_position = None

    rospy.spin()

  #controller
  def controller(self, waypoint):
    """
    Controls a turtlebot whose position is denoted by turtlebot_frame,
    to go to a position denoted by target_frame
    Inputs:
    - turtlebot_frame: the tf frame of the AR tag on your turtlebot
    - goal_frame: the tf frame of the target AR tag
    """

    # Create a publisher and a tf buffer, which is primed with a tf listener
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
    tfBuffer = tf2_ros.Buffer() 
    tfListener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(10)   

    # All in the form [x, y]
  
    Kp = np.diag([2, 0.8]) #TUNE
    Kd = np.diag([-0.5, 0.5]) #TUNE
    Ki = np.diag([0, 0])

    prev_time = rospy.get_time() 
    integ = np.empty((2,), dtype = float) 
    derivative = np.empty((2,), dtype = float) 
    previous_error = np.empty((2,), dtype = float) 

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
      try:
        #                                              target_frame, source_frame, current_time_in_ros, how long to wait for transform lookup
        #trans_odom_to_base_link = tfBuffer.lookup_transform(rospy.get_param("~frames/fixed") , rospy.get_param("~frames/sensor"), rospy.Time(), rospy.Duration(5)) # TODO: create a transform between odom to base link
        trans_odom_to_base_link = tfBuffer.lookup_transform("base_footprint", "odom", rospy.Time(), rospy.Duration(5)) # TODO: create a transform between odom to base link

        (roll, pitch, baselink_yaw) = tf.transformations.euler_from_quaternion(
          [trans_odom_to_base_link.transform.rotation.x, trans_odom_to_base_link.transform.rotation.y,
              trans_odom_to_base_link.transform.rotation.z, trans_odom_to_base_link.transform.rotation.w])


        waypoint_trans = PoseStamped() 
        waypoint_trans.pose.position.x = waypoint[0] 
        waypoint_trans.pose.position.y = waypoint[1] 
        waypoint_trans.pose.position.z = 0

        quat = quaternion_from_euler(0, 0, waypoint[2]) 
        print(quat)
        waypoint_trans.pose.orientation.x = quat[0] 
        waypoint_trans.pose.orientation.y = quat[1] 
        waypoint_trans.pose.orientation.z = quat[2] 
        waypoint_trans.pose.orientation.w = quat[3] 

        # Use the transform to compute the waypoint's pose in the base_link frame
        waypoint_in_base_link = do_transform_pose(waypoint_trans, trans_odom_to_base_link) # TODO: what would be the inputs to this function (there are 2)
        #waypoint_in_base_link = do_transform_pose(waypoint_trans, rospy.get_param("~frames/fixed"))
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
          [waypoint_in_base_link.pose.orientation.x, waypoint_in_base_link.pose.orientation.y,
              waypoint_in_base_link.pose.orientation.z, waypoint_in_base_link.pose.orientation.w])


        curr_time = rospy.get_time()

        # some debug output below
        print(f"Current: {trans_odom_to_base_link.transform.translation.x}, {trans_odom_to_base_link.transform.translation.y}, {baselink_yaw  }")
        print(f"Target: {waypoint}")

        # Process trans to get your state error
        # Generate a control command to send to the robot
        x_error = waypoint_in_base_link.pose.position.x
        #y_error = waypoint_in_base_link.pose.position.y # THIS WAS ADDED
        error = np.array([x_error, yaw])
        
        # proportional term
        proportional = np.dot(Kp, error).squeeze()
        
        # integral term
        dt = curr_time - prev_time 
        integ += previous_error * dt
        integral = np.dot(Ki, integ).squeeze()

        # dervative term
        error_deriv = (error - previous_error)/dt 
        derivative = np.dot(Kd, error_deriv).squeeze()

        msg = Twist()
        msg.linear.x = proportional[0] + derivative[0] + integral[0] 
        msg.angular.z = proportional[1] + derivative[1] + integral[1] 

        control_command = msg

        previous_error = error
        prev_time = curr_time
        pub.publish(control_command)

        if np.abs(x_error) < 0.05 and np.abs(yaw) < 0.05: 
          print("Moving to next waypoint in trajectory")
          return

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print("TF Error in Turtlebot Controller: " + e)
        pass

      r.sleep()

  #should we use a bezier curve or will that cause us to run into walls?

  def ar_tag_callback(self, msg):
    if msg.tag_id == 1:
      self.ar_tag_position = (msg.point.x, msg.point.y, msg.point.z)
      print(f"AR Tag #1 position: {self.ar_tag_position}")
    elif msg.tag_id == 0:
      self.goal_position = (msg.point.x, msg.point.y, msg.point.z)
      print(f"AR Tag #0 position: {self.goal_position}")
    else:
      print(f"Found AR Tag {msg.tag_id}")

  def planning_callback(self, msg):
    try:
      #msg is ar tag at ending point
      #trajectory = plan_curved_trajectory(msg.x, msg.y, msg.z) # TODO: What is the tuple input to this function? the target position, in this case the green cup

      #tuple_final_point = (msg.x, msg.y, msg.z)
      if self.goal_position is None:
        print("AR tag position not yet found")
        return

      if isinstance(msg, Path):
        self.path = [(pose.pose.position.x, pose.pose.position.y, 0) for pose in msg.poses]
      # want to choose the correct point, not the turtlebot
      elif isinstance(msg, Point):
        self.point = (msg.x, msg.y, msg.z)
      else:
        return
      
      if self.path is not None and self.goal_position is not None:
        #trajectory = plan_curved_trajectory(self.goal_position, self.path) #grab whe waypoints on goal path
        trajectory = plan_curved_trajectory(self.path) #grab whe waypoints on goal path
        #instead of calculating the trajectory to the final point, input the waypoints given by a* and final ar_tag as the trajectory
        for waypoint in trajectory:
          self.controller(self, waypoint)

    except rospy.ROSInterruptException as e:
      print("Exception thrown in planning callback: " + e)
      pass

  #do we want to implement object avoidance for colors? (blue and white as they represent the walls?)

#main method
if __name__ == '__main__':
    turtlebotController()