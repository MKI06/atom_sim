import cv2
import numpy as np

def separate_left_right(image):
    # Define a vertical line at the center
    center_x = image.shape[1] // 2
    
    # Split the image into left and right parts
    left_region = image[:, :center_x]
    right_region = image[:, center_x:]
    
    return left_region, right_region

def ransac_line_fit(points, iterations=100, threshold_distance=5):
    best_line = None
    best_inliers = []

    for _ in range(iterations):
        # Randomly sample 2 points
        sample_points = points[np.random.choice(len(points), 2, replace=False)]
        x1, y1 = sample_points[0]
        x2, y2 = sample_points[1]

        # Fit line using the sampled points
        if x2 - x1 == 0:
            continue  # Avoid division by zero
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1

        # Calculate distances from all points to the line
        distances = np.abs(points[:, 1] - slope * points[:, 0] - intercept)
        inliers = points[distances < threshold_distance]

        # Update best line if this model has more inliers
        if len(inliers) > len(best_inliers):
            best_line = (slope, intercept)
            best_inliers = inliers

    return best_line

image_path = "/home/mustafa/catkin_ws/src/atom/script2/img/9_image_hough_4.jpg"
image = cv2.imread(image_path)

# Define lower and upper bounds for the green color in HSV space
lower_green = np.array([35, 50, 50])
upper_green = np.array([90, 255, 255])

# Convert the image to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a mask for the green color
green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

# Apply the mask to the original image
green_masked_image = cv2.bitwise_and(image, image, mask=green_mask)

# Separate the image into left and right regions
left_region, right_region = separate_left_right(green_masked_image)

# Edge detection for left and right regions
left_edges = cv2.Canny(left_region, 50, 150)
right_edges = cv2.Canny(right_region, 50, 150)

# Find non-zero points from edge images
left_points = np.column_stack(np.where(left_edges > 0))
right_points = np.column_stack(np.where(right_edges > 0))

# Fit line using RANSAC for left and right regions
left_slope, left_intercept = ransac_line_fit(left_points)
right_slope, right_intercept = ransac_line_fit(right_points)

# Draw lines on the masked image for left and right regions
if left_slope is not None:
    left_line_y1 = int(left_slope * 0 + left_intercept)
    left_line_y2 = int(left_slope * left_region.shape[1] + left_intercept)
    cv2.line(green_masked_image, (0, left_line_y1), (left_region.shape[1], left_line_y2), (255, 0, 0), 2)

if right_slope is not None:
    right_line_y1 = int(right_slope * 0 + right_intercept)
    right_line_y2 = int(right_slope * right_region.shape[1] + right_intercept)
    cv2.line(green_masked_image, (right_region.shape[1], right_line_y1), (image.shape[1], right_line_y2), (255, 0, 0), 2)

# Display the result
cv2.imshow('RANSAC Lines', green_masked_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
