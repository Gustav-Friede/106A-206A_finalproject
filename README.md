Setup commands

# Initial Setup:

  # Capture images using snap_photos (for camera calibration using chAruco board)
    roslaunch computer_vision snap_photos.launch

  # Run the marker calibration to ensure accurary
    rosrun computer_vision marker_calibrate.py


  # Display bird's eye view camera feed after calibration finishes
    rosun computer_vision birds_eye_view.py

