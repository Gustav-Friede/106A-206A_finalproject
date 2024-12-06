#!/usr/bin/env python3
import os
import cv2 as cv

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

# uncomment if you want to create a new board
#create_and_save_new_board()


