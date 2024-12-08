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

squares_horizontally = 5
squares_vertically = 5

# Each square is 12 inches which is 0.3048 meters
square_length = 0.3048  # meters per square side
marker_length = 0.15    # meters, choose smaller than square_length for good visibility

total_page_length_px = 2000  # resolution of the output image in pixels
margin_px = 20


aruco_dict = cv.aruco.DICT_6X6_250  # or whichever you prefer
save_dir = "../../../imgs/"

save_path = create_and_save_new_board(
    aruco_dict=aruco_dict,
    squares_horizontally=squares_horizontally,
    squares_vertically=squares_vertically,
    square_length=square_length,
    marker_length=marker_length,
    total_page_length_px=total_page_length_px,
    margin_px=margin_px,
    save_dir=save_dir
)
