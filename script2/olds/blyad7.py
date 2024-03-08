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
middle_line_x = width // 2 - offset

# Find contours in the binary image
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Initialize empty lists to store points for left and right sides
left_points = []
right_points = []

# Iterate through contours
for contour in contours:
    # Get all points of the contour
    for point in contour[:, 0]:
        # Check if the point is on the left or right side of the middle line
        if point[0] < middle_line_x:
            left_points.append(point)
        else:
            right_points.append(point)

# Convert points to numpy arrays
left_points = np.array(left_points)
right_points = np.array(right_points)

# Find convex hull for left and right points
left_hull = cv2.convexHull(left_points)
right_hull = cv2.convexHull(right_points)

# Draw convex hulls as trapeziums
cv2.drawContours(image, [left_hull], -1, (0, 0, 255), 2)
cv2.drawContours(image, [right_hull], -1, (0, 0, 255), 2)

# Display the result
cv2.imshow('Image with Left and Right Trapeziums', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
