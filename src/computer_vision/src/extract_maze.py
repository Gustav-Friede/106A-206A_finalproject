import os

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

MIN_THRESH_VALUE = 120
MAX_THRESH_VALUE = 255

# read image
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, '..', 'data', 'camera_calibration', 'sat_0.png')
img = cv.imread(image_path, cv.IMREAD_REDUCED_GRAYSCALE_2)

# extract edges and remove background
edges = cv.Canny(img, MIN_THRESH_VALUE, MAX_THRESH_VALUE)
img_wc = edges.copy() # for visual purpose
krn = cv.getStructuringElement(cv.MORPH_RECT, (16, 16))
dlt = cv.dilate(edges, krn, iterations=5)
res = 255 - cv.bitwise_and(dlt, img_wc)

# pinpoint corners of the maze
# corners = cv.goodFeaturesToTrack(edges, 16,0.50, 50)
# corners = np.int0(corners)
# for corner in corners:
#     x, y = corner.ravel()
#     cv.circle(res, (int(x), int(y)), 5, 0, -1)

image = cv.findContours(img_wc, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# display original and edge-detected images
plt.figure(figsize=(10, 5))
#
# original Image
plt.subplot(1, 2, 1)  # 1 row, 2 columns, 1st subplot
plt.imshow(img, cmap='gray')
plt.title('Original Image')
#plt.xticks([]), plt.yticks([])

# edge Image
plt.subplot(1, 2, 2)  # 1 row, 2 columns, 2nd subplot
plt.imshow(res, cmap='gray')
plt.title('Edge and Corners')
#plt.xticks([]), plt.yticks([])

plt.tight_layout()
plt.show()


# find_corners.py code, just in case i need to use it later

