import cv2
import numpy as np

image_path = "/home/mustafa/catkin_ws/src/atom/script2/img/9_image_hough_4.jpg"
image = cv2.imread(image_path)


# Convert the image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define range of green color in HSV
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

# Threshold the HSV image to get only green colors
mask = cv2.inRange(hsv, lower_green, upper_green)

# Bitwise-AND mask and original image
green_only = cv2.bitwise_and(image, image, mask=mask)


# Get image dimensions
height, width, _ = green_only.shape

# Define the offset from the middle
offset = 20  # Adjust this value as needed

# Define the coordinates for the line
start_point = (width // 2 - offset, 0)
end_point = (width // 2 - offset, height)

# Draw the line on the image
line_color = (0, 255, 0)  # Green color
line_thickness = 2
image_with_line = cv2.line(green_only, start_point, end_point, line_color, line_thickness)

# Display the result
cv2.imshow('Image with Line', image_with_line)
cv2.waitKey(0)
cv2.destroyAllWindows()
