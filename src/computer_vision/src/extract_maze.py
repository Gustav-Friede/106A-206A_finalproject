import os

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

MIN_THRESH_VALUE = 120
MAX_THRESH_VALUE = 255

# read image
script_dir = os.path.dirname(os.path.abspath(__file__))
img_path = os.path.join(script_dir, '..', '..', '..','camera_calibration', 'calibrated_maze', 'snapshot_000.png')
img = cv.imread(img_path, cv.IMREAD_REDUCED_GRAYSCALE_8)

# extract edges and remove background
edges = cv.Canny(img, MIN_THRESH_VALUE, MAX_THRESH_VALUE)
img_wc = edges.copy() # for visual purpose
krn = cv.getStructuringElement(cv.MORPH_RECT, (16, 16))
dlt = cv.dilate(edges, krn, iterations=5)
res = 255 - cv.bitwise_and(dlt, img_wc)

# pinpoint corners of the maze
corners = cv.goodFeaturesToTrack(edges, 32,0.08, 50)
corners = np.int0(corners)
for corner in corners:
    x, y = corner.ravel()
    cv.circle(res, (int(x), int(y)), 5, 0, -1)

image = cv.findContours(img_wc, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
print(image)
#plt.imshow(image)

# cv.imshow('edge and corners', img_wc)
# cv.waitKey(0)
# cv.destroyAllWindows()

# Display original and edge-detected images
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
