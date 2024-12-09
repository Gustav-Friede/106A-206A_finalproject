Setup commands

# Step 1: capture images using snap_photos (for camera calibration using chAruco board)
roslaunch computer_vision snap_photos.launch

# Step 2: run the marker calibration
rosrun computer_vision marker_calibrate.py

# Step 3: start bird's eye view node after calibration finishes
rosun computer_vision birds_eye_view.py

