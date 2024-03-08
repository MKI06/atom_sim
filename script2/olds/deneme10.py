import cv2
import numpy as np

# Load the image
image_path = "/home/mustafa/catkin_ws/src/atom/script2/img/9_image_hough_4.jpg"
image = cv2.imread(image_path)

# Convert the image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define range of green color in HSV
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

# Threshold the HSV image to get only green colors
mask = cv2.inRange(hsv, lower_green, upper_green)

# Strip the masked image vertically each 10 pixels
stripped_mask = np.copy(mask)
stripped_mask[:,::10] = 0

# Perform morphological operations to retain only the lines on green areas
kernel = np.ones((5,5), np.uint8)
dilated_mask = cv2.dilate(stripped_mask, kernel, iterations=1)
eroded_mask = cv2.erode(dilated_mask, kernel, iterations=1)

# Bitwise-AND mask and original image
green_only = cv2.bitwise_and(image, image, mask=eroded_mask)

# Display the lines on green areas
cv2.imshow('Lines on Green Areas', green_only)
cv2.waitKey(0)
cv2.destroyAllWindows()
