## How to use:
###### Most of the files are not needed, are test/help files or are temporary files!
###### Files in use: 
####### - snap_photos.py
####### - snap_photos.yaml
####### - snap_photos.launch
####### - hsv_camera_feed.py
####### - ar_tag_detector.py
####### - occupancy_grid_publisher.py
####### - trajectory.py (from pathfinder_pkg)
####### - a_star_Gutsav_v4.py
####### - real_world_transform.py
####### - turtlebot_controller.py
####### - trajectory.py (from path_planning)
####### -snap_photos.py
####### -snap_photos.py
#
### Launches and runs:
##### launch calibration
    roslaunch computer_vision snap_photos.launch
##### Shift - S to save snapshot, Shift - Q to Quit, Ctrl - C to kill
#   
##### launch birds eye view, ar tag detector, oc-grid, pathfinder and transformation
    roslaunch pathfinder_pkg launch_pathfinder.launch
#
##### ssh into robot and run bringup
    roslaunch turtlebot3_bringup turtlebot3_robot.launch --screen
#
##### run controller
    rosrun path_planning turtlebot_controller.py
#
