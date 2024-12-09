import os
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


# MORPH_KERNEL_SIZE = (4, 4)
#
# # initialize directory paths
# script_dir = os.path.dirname(os.path.abspath(__file__))
# img_path = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', 'camera_snapshots', 'snapshot_000.png')
# #img_path = os.path.join(script_dir, '..', '..', '..', 'imgs', 'birds-view-maze.jpg')
# img = cv.imread(img_path)
# if img is None:
#     raise FileNotFoundError("Image not found at the specified path.")
#
# # threshold the image to create a binary mask
# gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# clahe = cv.createCLAHE(clipLimit=1.0, tileGridSize=(8, 8))
# gray = clahe.apply(gray)
# _, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
#
# # fill small holes and consolidate the maze shape using morphological closing
# kernel = cv.getStructuringElement(cv.MORPH_RECT, MORPH_KERNEL_SIZE)
# closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)
#
# # find contours in the processed image
# contours, hierarchy = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# if not contours:
#     print("No contours found. Consider adjusting THRESH_VALUE or MORPH_KERNEL_SIZE.")
#     exit(1)
#
# # sort contours by area and pick the largest contour assuming it's the maze outline
# contours = sorted(contours, key=cv.contourArea, reverse=True)
# maze_contour = None
#
# # approximate the contour to a polygon
# for cnt in contours:
#     perimeter = cv.arcLength(cnt, True)
#     epsilon = 0.03 * perimeter  # Adjust epsilon if needed
#     approx = cv.approxPolyDP(cnt, epsilon, True)
#
#     # ensure approximated polygon has 4 vertices
#     if len(approx) == 4:
#         maze_contour = approx
#         break
#
# if maze_contour is None:
#     print("No quadrilateral found. Try adjusting preprocessing steps.")
#     exit(1)
#
# # sort corners by y-coordinate and allow x-coordinate be the tie-breaker
# corners = maze_contour.reshape(4, 2)
#
# maze_corners = np.zeros((4, 2), dtype=np.int32)
# s = corners.sum(axis=1)
#
# # top-left point has smallest difference (x-y)
# # bottom-right point will have the largest sum (x+y)
# maze_corners[0] = corners[np.argmin(s)]
# maze_corners[2] = corners[np.argmax(s)]
#
# diff = np.diff(corners, axis=1)
# maze_corners[1] = corners[np.argmin(diff)]
# maze_corners[3] = corners[np.argmax(diff)]

#
# def order_points(pts):
#     # pts: numpy array of shape (4, 2)
#     rect = np.zeros((4, 2), dtype="float32")
#
#     # the top-left point will have the smallest sum (x+y)
#     # the bottom-right point will have the largest sum (x+y)
#     s = pts.sum(axis=1)
#     rect[0] = pts[np.argmin(s)]
#     rect[2] = pts[np.argmax(s)]
#
#     # the top-right point will have the smallest difference (x-y)
#     # the bottom-left will have the largest difference (x-y)
#     diff = np.diff(pts, axis=1)
#     rect[1] = pts[np.argmin(diff)]
#     rect[3] = pts[np.argmax(diff)]
#
#     return rect
#
# ordered_corners = order_points(corners)
#
# # draw the detected corners and outline on the original image for visualization
# result_img = img.copy()
# cv.drawContours(result_img, [maze_contour], -1, (0,255,0), 5)
# for (x, y) in ordered_corners:
#     cv.circle(result_img, (int(x), int(y)), 15, (0,0,255), -1)
#
# # display results
# plt.figure(figsize=(10,5))
#
# plt.subplot(1,2,1)
# plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
# plt.title('Original Image')
#
# plt.subplot(1,2,2)
# plt.imshow(cv.cvtColor(result_img, cv.COLOR_BGR2RGB))
# plt.title('Detected Maze Corners')
#
# plt.tight_layout()
# plt.show()
#
# print("Found corners (ordered):")
# print(ordered_corners)
