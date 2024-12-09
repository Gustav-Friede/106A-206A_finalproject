import os
import cv2
import numpy as np

def nothing(x):
    pass

# initialize directory paths
script_dir = os.path.dirname(os.path.abspath(__file__))
img_path = os.path.join(script_dir, '..', 'data', 'camera_calibration', 'snapshot_00.png')
image = cv2.imread(img_path)
if image is None:
    raise FileNotFoundError("Image not found at the specified path.")

# screen size
screen_width = 1920
screen_height = 1080

# image dimensions
img_height, img_width = image.shape[:2]

available_width = screen_width
available_height = screen_height - 500

scale_factor = min(available_width / img_width, available_height / img_height)

if scale_factor < 1:
    new_width = int(img_width * scale_factor)
    new_height = int(img_height * scale_factor)
    image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

# resizable window
cv2.namedWindow('image', cv2.WINDOW_NORMAL)

# create trackbars
cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('SMin','image',0,255,nothing)
cv2.createTrackbar('VMin','image',0,255,nothing)
cv2.createTrackbar('HMax','image',0,179,nothing)
cv2.createTrackbar('SMax','image',0,255,nothing)
cv2.createTrackbar('VMax','image',0,255,nothing)

# git default values for MAX HSV trackbars.
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

wait_time = 33

while True:
    hMin = cv2.getTrackbarPos('HMin','image')
    sMin = cv2.getTrackbarPos('SMin','image')
    vMin = cv2.getTrackbarPos('VMin','image')
    hMax = cv2.getTrackbarPos('HMax','image')
    sMax = cv2.getTrackbarPos('SMax','image')
    vMax = cv2.getTrackbarPos('VMax','image')

    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(image,image, mask=mask)

    if ((phMin != hMin) or (psMin != sMin) or (pvMin != vMin) or
        (phMax != hMax) or (psMax != sMax) or (pvMax != vMax)):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)"
              % (hMin, sMin, vMin, hMax, sMax, vMax))
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax

    # show the result
    cv2.imshow('image', output)

    if cv2.waitKey(wait_time) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
