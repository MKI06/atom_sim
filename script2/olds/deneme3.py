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

# Bitwise-AND mask and original image
green_only = cv2.bitwise_and(image, image, mask=mask)

# Convert green-only image to grayscale
gray = cv2.cvtColor(green_only, cv2.COLOR_BGR2GRAY)

# Apply Canny edge detection
edges = cv2.Canny(gray, 50, 150, apertureSize=3)

# Find contours
contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Sort contours based on their x-coordinate (left to right)
contours = sorted(contours, key=lambda x: cv2.boundingRect(x)[0])

# Draw contours on original image
cv2.drawContours(image, contours, -1, (0, 0, 255), 2)

# Draw crop row lines
for i in range(len(contours) - 1):
    # Get bounding box coordinates of adjacent contours
    x1, y1, w1, h1 = cv2.boundingRect(contours[i])
    x2, y2, w2, h2 = cv2.boundingRect(contours[i+1])
    
    # Calculate center points of bounding boxes
    center1 = (int(x1 + w1/2), int(y1 + h1/2))
    center2 = (int(x2 + w2/2), int(y2 + h2/2))
    
    # Calculate the distance between the center points
    distance = np.sqrt((center2[0] - center1[0])**2 + (center2[1] - center1[1])**2)
    
    # Define a threshold to skip drawing a line if the distance between centers is above it
    threshold = 55
    
    # Draw line between center points if distance is below the threshold
    if distance < threshold:
        cv2.line(image, center1, center2, (0, 0, 255), 2)

# Display the original image with contours and crop row lines
cv2.imshow('Crop Rows Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
