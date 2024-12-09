import os
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

# initialize directory paths
script_dir = os.path.dirname(os.path.abspath(__file__))
#img_path = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', 'camera_snapshots', 'snapshot_002.png')
#img_path = os.path.join(script_dir, '..', '..', '..', 'imgs', 'maze_map_labroom_with_marker.png')
#img_path = os.path.join(script_dir, '..', '..', '..', 'imgs', 'birds-view-maze.jpg')
img_path = os.path.join(script_dir, '..', '..', '..','camera_calibration', 'calibrated_maze', 'snapshot_000.png')
img = cv.imread(img_path)
if img is None:
    raise FileNotFoundError("Image not found at the specified path.")

hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# define color ranges for the white tape (adjust as needed)
sensitivity = 40
lower_white = np.array([0, 0, 115])
upper_white = np.array([90, 40, 255])
mask = cv.inRange(hsv, lower_white, upper_white)

# Morphological operations to clean up the mask
kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

# Find contours in the mask
contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
if not contours:
    print("No contours found in the mask. Adjust the HSV ranges or lighting.")
    exit(1)

# Sort and approximate contours
contours = sorted(contours, key=cv.contourArea, reverse=True)
maze_contour = None
for cnt in contours:
    perimeter = cv.arcLength(cnt, True)
    epsilon = 0.02 * perimeter
    approx = cv.approxPolyDP(cnt, epsilon, True)
    if len(approx) == 4:
        maze_contour = approx
        break

if maze_contour is None:
    print("No quadrilateral found. Try adjusting color ranges or epsilon.")
    exit(1)



# Convert from HSV back to BGR, then to RGB for correct Matplotlib display
rgb_from_hsv = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
rgb_from_hsv = cv.cvtColor(rgb_from_hsv, cv.COLOR_BGR2RGB)

# Split HSV channels for separate grayscale display
h, s, v = cv.split(hsv)

plt.figure(figsize=(12,8))

# Original Image (in RGB)
plt.subplot(2,3,1)
plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
plt.title('Original Image')

# Result Image with corners
plt.subplot(2,3,2)
plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
plt.title('Place holder')

# HSV Image interpreted back as RGB
plt.subplot(2,3,3)
plt.imshow(rgb_from_hsv)
plt.title('HSV -> BGR -> RGB')

# H Channel
plt.subplot(2,3,4)
plt.imshow(h, cmap='gray')
plt.title('H Channel')

# S Channel
plt.subplot(2,3,5)
plt.imshow(s, cmap='gray')
plt.title('S Channel')

# V Channel
plt.subplot(2,3,6)
plt.imshow(v, cmap='gray')
plt.title('V Channel')

plt.tight_layout()
plt.show()

print("Found corners (ordered):")
print(ordered_corners)
