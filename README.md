# launch calibration
    roslaunch computer_vision snap_photos.launch
# Shift - S to save snapshot, Shift - Q to Quit, Ctrl - C to kill

# launch birds eye view, ar tag detector, oc-grid, pathfinder and transformation
    roslaunch pathfinder_pkg launch_pathfinder.launch

# ssh into robot and run bringup
    roslaunch turtlebot3_bringup turtlebot3_robot.launch --screen

# run controller
    rosrun path_planning turtlebot_controller.py