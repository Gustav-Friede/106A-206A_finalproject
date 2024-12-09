import os
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

# initialize directory paths
script_dir = os.path.dirname(os.path.abspath(__file__))
img_path = os.path.join(script_dir, '..', 'data', 'camera_calibration', 'snapshot_00.png')
img = cv.imread(img_path)
if img is None:
    raise FileNotFoundError("Image not found at the specified path.")

hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# convert from HSV back to BGR, then to RGB for correct Matplotlib display
rgb_from_hsv = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
rgb_from_hsv = cv.cvtColor(rgb_from_hsv, cv.COLOR_BGR2RGB)

# split HSV channels for separate grayscale display
h, s, v = cv.split(hsv)

plt.figure(figsize=(12, 8))

# original Image (in RGB)
plt.subplot(2, 3, 1)
plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
plt.title('Original Image')

# result Image with corners
plt.subplot(2, 3, 2)
plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
plt.title('Place holder')

# HSV Image interpreted back as RGB
plt.subplot(2, 3, 3)
plt.imshow(rgb_from_hsv)
plt.title('HSV -> BGR -> RGB')

# H Channel
plt.subplot(2, 3, 4)
plt.imshow(h, cmap='gray')
plt.title('H Channel')

# S Channel
plt.subplot(2, 3, 5)
plt.imshow(s, cmap='gray')
plt.title('S Channel')

# V Channel
plt.subplot(2, 3, 6)
plt.imshow(v, cmap='gray')
plt.title('V Channel')

plt.tight_layout()
plt.show()
