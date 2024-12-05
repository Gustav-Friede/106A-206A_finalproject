
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

filepath = '../imgs/'

# initialize image
img = cv.imread(filepath + 'calibrated.jpg', cv.IMREAD_REDUCED_GRAYSCALE_8)


edges = cv.Canny(img, 64, 264)
corners = cv.goodFeaturesToTrack(edges, 64, 0.02, 36)
corners = np.int0(corners)

img_wc = edges.copy()
for corner in corners:
    x, y = corner.ravel()
    cv.circle(img_wc, (int(x), int(y)), 5, 255, -1)

plt.figure(figsize=(12, 12))

# original image
plt.subplot(2, 2, 1)
plt.imshow(cv.imread(filepath + 'calibrated.jpg'), cmap='gray')
plt.title('Original Image')
plt.xticks([]), plt.yticks([])

# grayscale image
plt.subplot(2, 2, 2)
plt.imshow(cv.imread(filepath + 'calibrated.jpg', cv.IMREAD_REDUCED_GRAYSCALE_8), cmap='gray')
plt.title('Grayscale')
plt.xticks([]), plt.yticks([])

# canny edge detection
plt.subplot(2, 2, 3)
plt.imshow(edges, cmap='gray')
plt.title('Edges')
plt.xticks([]), plt.yticks([])

# harris corner detection
plt.subplot(2, 2, 4)
plt.imshow(img_wc, cmap='gray')
plt.title('Corners')
plt.xticks([]), plt.yticks([])

plt.tight_layout()
plt.show()

