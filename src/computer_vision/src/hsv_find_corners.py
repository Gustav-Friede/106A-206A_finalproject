import os
import cv2 as cv
import numpy as np
from cv2 import waitKey
from matplotlib import pyplot as plt

# initialize directory paths
script_dir = os.path.dirname(os.path.abspath(__file__))
img_path = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', 'camera_snapshots', 'snapshot_000.png')
#img_path = os.path.join(script_dir, '..', '..', '..', 'imgs', 'birds-view-maze.jpg')
img = cv.imread(img_path)
if img is None:
    raise FileNotFoundError("Image not found at the specified path.")

# Convert to HSV
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# cv.imshow('HSV', hsv)
# waitKey(5000)

# Define color ranges for the white tape
# You may need to adjust these bounds depending on your tape and lighting.
# The given example tries to capture bright white regions.
lower_white = np.array([0, 0, 115])   # Lower boundary for (H,S,V)
upper_white = np.array([179, 36, 255]) # Upper boundary for (H,S,V)
mask = cv.inRange(hsv, lower_white, upper_white)

# Optional: apply morphological operations to clean up the mask
kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

# Find contours of the white regions
contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
if not contours:
    print("No contours found in the mask. Adjust the HSV ranges or lighting.")
    exit(1)

# Sort contours by area, largest first
contours = sorted(contours, key=cv.contourArea, reverse=True)

maze_contour = None
for cnt in contours:
    perimeter = cv.arcLength(cnt, True)
    epsilon = 0.02 * perimeter  # Adjust as needed for approximation
    approx = cv.approxPolyDP(cnt, epsilon, True)
    if len(approx) == 4:
        maze_contour = approx
        break

if maze_contour is None:
    print("No quadrilateral found. Try adjusting color ranges or epsilon.")
    exit(1)

# order the corners for consistency
corners = maze_contour.reshape(4, 2)

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

ordered_corners = order_points(corners)

# Draw the detected corners and contour on the original image
result_img = img.copy()
cv.drawContours(result_img, [maze_contour], -1, (0,255,0), 5)
for (x, y) in ordered_corners:
    cv.circle(result_img, (int(x), int(y)), 15, (0,0,255), -1)

# Display the results
plt.figure(figsize=(10,5))
plt.subplot(1,2,1)
plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
plt.title('Original Image')

# plt.subplot(1,2,2)
# plt.imshow(cv.cvtColor(result_img, cv.COLOR_BGR2RGB))
# plt.title('Detected Maze Corners (Color-Based)')

plt.subplot(1,2,2)
plt.imshow(hsv)
plt.title('HSV Image')

plt.tight_layout()
plt.show()

print("Found corners (ordered):")
print(ordered_corners)
