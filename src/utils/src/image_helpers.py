#!/usr/bin/env python3
import os
import cv2 as cv
import numpy as np

# file path that leads to directory of -- final_project/imgs/ --
script_dir = os.path.dirname(os.path.abspath(__file__))
img_path = os.path.join(script_dir, '..', '..', '..', 'imgs/')


def resizeImage(image, width=None, height=None, inter=cv.INTER_AREA):
    (h, w) = image.shape[:2]
    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))
    return cv.resize(image, dim, interpolation=inter)

ARUCO_DICT = cv.aruco.DICT_6X6_250
SQUARES_VERTICALLY = 7
SQUARES_HORIZONTALLY = 5
SQUARE_LENGTH = 0.03
MARKER_LENGTH = 0.015
TOTAL_PAGE_LTH_PX = 640
MARGIN_PX = 20
SAVE_NAME = img_path + 'calibration_marker.png'

# creates and saves a Charuco board marker that we will use for image calibration
def create_and_save_new_board():

    dictionary = cv.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv.aruco.CharucoBoard_create(SQUARES_HORIZONTALLY, SQUARES_VERTICALLY, SQUARE_LENGTH, MARKER_LENGTH, dictionary)

    size_ratio = SQUARES_HORIZONTALLY / SQUARES_VERTICALLY
    img = board.draw((TOTAL_PAGE_LTH_PX, int(TOTAL_PAGE_LTH_PX * size_ratio)), marginSize=MARGIN_PX)

    cv.imshow("Charuco Board", img)
    # uncomment to see what the calibration marker looks like!!
    #cv.waitKey(2000)
    cv.imwrite(SAVE_NAME, img)


def calibrate_parameters():
    dictionary = cv.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv.aruco.CharucoBoard_create(SQUARES_HORIZONTALLY, SQUARES_VERTICALLY, SQUARE_LENGTH, MARKER_LENGTH, dictionary)

    detector_params = cv.aruco.DetectorParameters_create()

    # load images
    img_files = [os.path.join(img_path, f) for f in os.listdir(img_path) if f.endswith('.png')]
    img_files.sort()

    all_corners = []
    all_ids = []

    for img_file in img_files:
        img = cv.imread(img_file)
        img_copy = img.copy()
        marker_corners, marker_ids, _ = cv.aruco.detectMarkers(img, dictionary, parameters=detector_params)

        if marker_ids is not None and len(marker_ids) > 0:
            cv.aruco.drawDetectedMarkers(img_copy, marker_corners, marker_ids)
            retval, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(marker_corners, marker_ids, img, board)

            if retval > 0:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)

    # calibrate camera
    height, width = img.shape[:2]
    retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
        all_corners,
        all_ids,
        board,
        (width, height),
        None,
        None
    )

    np.save('cameraMatrix.npy', camera_matrix)
    np.save('distCoeffs.npy', dist_coeffs)

    # show results
    for img_file in img_files:
        img = cv.imread(img_file)
        undistorted_image = cv.undistort(img, camera_matrix, dist_coeffs)
        cv.imshow('Undistorted Image', undistorted_image)
        cv.waitKey(0)

    cv.destroyAllWindows()

#calibrate_parameters()







