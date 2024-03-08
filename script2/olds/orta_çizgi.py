import cv2
import numpy as np

image_path = "/home/mustafa/catkin_ws/src/atom/script2/img/green_2.jpg"
image = cv2.imread(image_path)

# Convert the image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define range of green color in HSV
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

# Threshold the HSV image to get only green colors
mask = cv2.inRange(hsv, lower_green, upper_green)
cv2.imshow("a", mask)

# Calculate center of mass of the mask
M = cv2.moments(mask)

# Calculate center of mass
center_of_mass = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

print("Center of color density:", center_of_mass)

# Draw a dot on the original image at the center of mass
image_with_line = image.copy()
line_color = (0, 0, 255)  # Red color
line_thickness = 2
cv2.line(image_with_line, center_of_mass, (center_of_mass[0], image.shape[0]), line_color, line_thickness)

# Display the image with the line
cv2.imshow('Image with Line', image_with_line)
cv2.waitKey(0)
cv2.destroyAllWindows()
