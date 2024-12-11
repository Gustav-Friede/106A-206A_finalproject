# launch calibration
    roslaunch computer_vision snap_photos.launch

# launch normal birds eye camera feed
    rosrun computer_vision birds_eye_view.py
    rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _camera_name:=usb_cam

# launch hsv camera feed
    rosrun computer_vision hsv_camera_feed.py
    rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _camera_name:=usb_cam

