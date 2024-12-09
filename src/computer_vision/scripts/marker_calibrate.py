#!/usr/bin/env python3
import rospy
import os
import cv2 as cv
import numpy as np

def calibrate_parameters(aruco_dict, squares_horizontally, squares_vertically, square_length, marker_length, img_dir):
    dictionary = cv.aruco.getPredefinedDictionary(aruco_dict)
    board = cv.aruco.CharucoBoard_create(squares_horizontally, squares_vertically, square_length, marker_length, dictionary)
    detector_params = cv.aruco.DetectorParameters_create()

    img_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith('.png')]
    img_files.sort()

    all_corners = []
    all_ids = []
    height, width = None, None

    for img_file in img_files:
        img = cv.imread(img_file)
        if img is None:
            continuec


        if height is None or width is None:
            height, width = img.shape[:2]

        marker_corners, marker_ids, _ = cv.aruco.detectMarkers(img, dictionary, parameters=detector_params)
        if marker_ids is not None and len(marker_ids) > 0:
            retval, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(marker_corners, marker_ids, img, board)
            if retval > 0:
                # print how many charuco corners were detected in this image (for debugging)
                num_corners = len(charuco_corners)
                print(f"{img_file} has {num_corners} charuco corners detected.")

                # throw away images with fewer than 4 corners
                if num_corners < 4:
                    print(f"Insufficient corners in {img_file}, skipping this image.")
                    continue

                # if enough corners were found, add them to lists
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
            else:
                print(f"Charuco interpolation failed for {img_file}, skipping this image.")
        else:
            print(f"No ArUco markers detected in {img_file}, skipping this image.")

    if not all_corners:
        print("No Charuco corners detected for calibration. Ensure images contain the ChArUco board.")
        return None, None

    retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
        all_corners,
        all_ids,
        board,
        (width, height),
        None,
        None
    )

    return camera_matrix, dist_coeffs


if __name__ == '__main__':
    rospy.init_node('charuco_calibrator', anonymous=True)

    # load parameters (same as snap_photos.yaml) of the marker board
    aruco_dict_name = rospy.get_param('~aruco_dictionary', 'DICT_6X6_250')
    squares_horizontally = rospy.get_param('~squares_horizontally', 5)
    squares_vertically = rospy.get_param('~squares_vertically', 5)
    square_length = rospy.get_param('~square_length', 0.3048)
    marker_length = rospy.get_param('~marker_length', 0.15)

    # convert dict to a constant and save it to the imgs directory as camera_snapshots
    aruco_dict = getattr(cv.aruco, aruco_dict_name)
    script_dir = os.path.dirname(os.path.realpath(__file__))
    img_dir = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', 'camera_snapshots')

    camera_matrix, dist_coeffs = calibrate_parameters(
        aruco_dict,
        squares_horizontally,
        squares_vertically,
        square_length,
        marker_length,
        img_dir
    )

    if camera_matrix is not None and dist_coeffs is not None:
        # save to imgs/ (one directory up from camera_snapshots)
        imgs_dir = os.path.join(script_dir, '..', '..', '..', 'camera_calibration')
        np.save(os.path.join(imgs_dir, 'cameraMatrix.npy'), camera_matrix)
        np.save(os.path.join(imgs_dir, 'distCoeffs.npy'), dist_coeffs)
        print("Camera calibration successful!")
        print("Parameters saved to cameraMatrix.npy and distCoeffs.npy in camera_calibration/")
    else:
        print("Calibration failed. Not enough sufficient images with a visible ChArUco board were found, try again.")
