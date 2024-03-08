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

# Find contours
contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Create a mask of the same size as the input image
mask_filled = np.zeros_like(image)

# Draw filled contours on the mask
cv2.drawContours(mask_filled, contours, -1, (0, 255, 0), thickness=cv2.FILLED)

# Apply the mask to the original image
green_inside_contour = cv2.bitwise_and(image, mask_filled)

# Display the image with green painted inside the contours
cv2.imshow('Green Inside Contour', green_inside_contour)
cv2.waitKey(0)
cv2.destroyAllWindows()
