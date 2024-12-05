
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image

filepath = '../imgs/'

# initialize image
img = cv.imread(filepath + 'calibrated.jpg', cv.IMREAD_REDUCED_GRAYSCALE_8)


edges = cv.Canny(img, 64, 264)
corners = cv.goodFeaturesToTrack(edges, 32,0.08, 50)
print(corners)
corners = np.int0(corners)

img_wc = edges.copy()
krn = cv.getStructuringElement(cv.MORPH_RECT, (16, 16))
dlt = cv.dilate(img_wc, krn, iterations=5)
res = 255 - cv.bitwise_and(dlt, img_wc)


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
plt.figure(figsize=(10, 5))  # Adjust figure size for better visualization
#
#Subplot 1: Original Image
plt.subplot(1, 2, 1)  # 1 row, 2 columns, 1st subplot
plt.imshow(img, cmap='gray')
plt.title('Original Image')
#plt.xticks([]), plt.yticks([])

# Subplot 2: Edge Image
plt.subplot(1, 2, 2)  # 1 row, 2 columns, 2nd subplot
plt.imshow(res, cmap='gray')
plt.title('Edge and Corners')
#plt.xticks([]), plt.yticks([])

plt.tight_layout()
plt.show()





# plt.subplot(1, 2, 2)  # 1 row, 2 columns, 2nd subplot
# plt.imshow(edges, cmap='gray')
# plt.title('Canny Edge Detection')
# plt.xticks([]), plt.yticks([])
#
#



# plt.figure(figsize=(12, 12))
# #original image
# plt.subplot(2, 2, 1)
# plt.imshow(cv.imread(filepath + 'calibrated.jpg'), cmap='gray')
# plt.title('Original Image')
# plt.xticks([]), plt.yticks([])
#
# #grayscale image
# plt.subplot(2, 2, 2)
# plt.imshow(cv.imread(filepath + 'calibrated.jpg', cv.IMREAD_REDUCED_GRAYSCALE_8), cmap='gray')
# plt.title('Grayscale')
# plt.xticks([]), plt.yticks([])
#
# #canny edge detection
# plt.subplot(2, 2, 3)
# plt.imshow(edges, cmap='gray')
# plt.title('Edges')
# plt.xticks([]), plt.yticks([])
#
# # harris corner detection
# plt.subplot(2, 2, 4)
# plt.imshow(img_wc, cmap='gray')
# plt.title('Corners')
# plt.xticks([]), plt.yticks([])
# #
# plt.tight_layout()
# plt.show()

