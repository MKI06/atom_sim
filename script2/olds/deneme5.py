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
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# If contours are found
if contours:
    # Get the largest contour
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Get the leftmost and rightmost points of the contour
    leftmost = tuple(largest_contour[largest_contour[:,:,0].argmin()][0])
    rightmost = tuple(largest_contour[largest_contour[:,:,0].argmax()][0])

    # Draw dots at leftmost and rightmost points on the original image
    cv2.circle(mask, leftmost, 5, (255, 0, 0), -1)
    cv2.circle(mask, rightmost, 5, (255, 0, 0), -1)

    # Display the image with dots
    cv2.imshow("Image with dots", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No green object found in the image.")
