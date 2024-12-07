import os
import cv2 as cv

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
