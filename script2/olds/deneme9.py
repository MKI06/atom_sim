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

# Iterate through contours
for contour in contours:
    # Calculate bounding box for each contour
    x, y, w, h = cv2.boundingRect(contour)
    
    # Check if height is greater than 5 pixels vertically
    if h > 1:
        # Calculate center of the green area vertically
        center_y = y + h // 2
        
        # Draw a small dot at the center of the green area vertically
        for i in range(x, x + w, 20):
            cv2.circle(image, (i, center_y), 2, (0, 0, 255), -1)

# Display the image with dots at the center of green areas vertically
cv2.imshow('Green Areas with Dots', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
