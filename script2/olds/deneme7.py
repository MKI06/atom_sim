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

# Create a green map
green_map = np.zeros_like(image)
green_map[mask != 0] = (0, 255, 0)  # Set green color in the areas where mask is not zero

# Show the green map
cv2.imshow("Green Map", green_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
