# import os
# import cv2 as cv
# import numpy as np
#
# def resizeImage(image, width=None, height=None, inter=cv.INTER_AREA):
#     (h, w) = image.shape[:2]
#     if width is None and height is None:
#         return image
#     if width is None:
#         r = height / float(h)
#         dim = (int(w * r), height)
#     else:
#         r = width / float(w)
#         dim = (width, int(h * r))
#     return cv.resize(image, dim, interpolation=inter)
#
# def create_and_save_new_board(aruco_dict, squares_horizontally, squares_vertically, square_length, marker_length, total_page_length_px, margin_px, save_dir):
#     dictionary = cv.aruco.getPredefinedDictionary(aruco_dict)
#     board = cv.aruco.CharucoBoard_create(squares_horizontally, squares_vertically, square_length, marker_length, dictionary)
#
#     size_ratio = squares_horizontally / squares_vertically
#     img = board.draw((total_page_length_px, int(total_page_length_px * size_ratio)), marginSize=margin_px)
#
#     save_path = os.path.join(save_dir, 'calibration_marker.png')
#     cv.imwrite(save_path, img)
#     return save_path

# def calibrate_parameters(aruco_dict, squares_horizontally, squares_vertically, square_length, marker_length, img_dir):
#     dictionary = cv.aruco.getPredefinedDictionary(aruco_dict)
#     board = cv.aruco.CharucoBoard_create(squares_horizontally, squares_vertically, square_length, marker_length, dictionary)
#     detector_params = cv.aruco.DetectorParameters_create()
#
#     img_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith('.png')]
#     img_files.sort()
#
#     all_corners = []
#     all_ids = []
#     height, width = None, None
#
#     for img_file in img_files:
#         img = cv.imread(img_file)
#         if img is None:
#             continue
#         if height is None or width is None:
#             height, width = img.shape[:2]
#
#         marker_corners, marker_ids, _ = cv.aruco.detectMarkers(img, dictionary, parameters=detector_params)
#         if marker_ids is not None and len(marker_ids) > 0:
#             retval, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(marker_corners, marker_ids, img, board)
#             if retval > 0:
#                 all_corners.append(charuco_corners)
#                 all_ids.append(charuco_ids)
#
#     if not all_corners:
#         print("No Charuco corners detected for calibration. Ensure images contain the board.")
#         return None, None
#
#     retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
#         all_corners,
#         all_ids,
#         board,
#         (width, height),
#         None,
#         None
#     )
#
#     return camera_matrix, dist_coeffs
#

import os
import cv2 as cv
import numpy as np

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

def create_and_save_new_board(aruco_dict, squares_horizontally, squares_vertically, square_length, marker_length, total_page_length_px, margin_px, save_dir):
    dictionary = cv.aruco.getPredefinedDictionary(aruco_dict)
    board = cv.aruco.CharucoBoard_create(squares_horizontally, squares_vertically, square_length, marker_length, dictionary)
    size_ratio = squares_horizontally / squares_vertically
    img = board.draw((total_page_length_px, int(total_page_length_px * size_ratio)), marginSize=margin_px)

    save_path = os.path.join(save_dir, 'calibration_marker.png')
    cv.imwrite(save_path, img)
    return save_path
