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

# Calculate the center of the left hull
left_center = np.mean(left_hull, axis=0)[0]
left_center = tuple(map(int, left_center))

# Calculate the center of the right hull
right_center = np.mean(right_hull, axis=0)[0]
right_center = tuple(map(int, right_center))

# Calculate the top and bottom edge center points for the left hull
left_top = tuple(left_hull[left_hull[:, :, 1].argmin()][0])
left_bottom = tuple(left_hull[left_hull[:, :, 1].argmax()][0])
left_edge_center = ((left_top[0] + left_bottom[0]) // 2, (left_top[1] + left_bottom[1]) // 2)

# Calculate the top and bottom edge center points for the right hull
right_top = tuple(right_hull[right_hull[:, :, 1].argmin()][0])
right_bottom = tuple(right_hull[right_hull[:, :, 1].argmax()][0])
right_edge_center = ((right_top[0] + right_bottom[0]) // 2, (right_top[1] + right_bottom[1]) // 2)

# Compute the direction vector from edge center to center
left_direction = (left_center[0] - left_edge_center[0], left_center[1] - left_edge_center[1])
right_direction = (right_center[0] - right_edge_center[0], right_center[1] - right_edge_center[1])

# Extend the lines from the edge centers in both directions
# For the left hull
left_extended_start = (left_edge_center[0] - 1000 * left_direction[0], left_edge_center[1] - 1000 * left_direction[1])
left_extended_end = (left_edge_center[0] + 1000 * left_direction[0], left_edge_center[1] + 1000 * left_direction[1])
# For the right hull
right_extended_start = (right_edge_center[0] - 1000 * right_direction[0], right_edge_center[1] - 1000 * right_direction[1])
right_extended_end = (right_edge_center[0] + 1000 * right_direction[0], right_edge_center[1] + 1000 * right_direction[1])

# Draw the extended lines
cv2.line(image, (int(left_extended_start[0]), int(left_extended_start[1])), (int(left_extended_end[0]), int(left_extended_end[1])), (255, 0, 0), 2)
cv2.line(image, (int(right_extended_start[0]), int(right_extended_start[1])), (int(right_extended_end[0]), int(right_extended_end[1])), (255, 0, 0), 2)

# Draw dots for the edge centers
cv2.circle(image, left_edge_center, 5, (255, 0, 255), -1)
cv2.circle(image, right_edge_center, 5, (255, 0, 255), -1)

# Draw convex hulls as trapeziums
cv2.drawContours(image, [left_hull], -1, (0, 0, 255), 2)
cv2.drawContours(image, [right_hull], -1, (0, 0, 255), 2)

# Display the result
cv2.imshow('Image with Left and Right Trapeziums', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
