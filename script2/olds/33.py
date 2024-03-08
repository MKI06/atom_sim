import cv2
import numpy as np

# Load an image
image_path = "/home/mustafa/catkin_ws/src/atom/script2/img/9_image_hough_4.jpg"
image = cv2.imread(image_path)

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#cv2.imshow("gray", gray_image)

# Apply Gaussian blur to reduce noise and improve contour detection
blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
#cv2.imshow("blurred", blurred)

# Apply adaptive thresholding to better separate objects from the background
thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
#cv2.imshow("thresh", thresh)

# Apply morphological operations to further clean up the image
kernel = np.ones((5,5), np.uint8)
#cv2.imshow("kernel", kernel)

morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

# Find contours in the processed image
contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter out contours based on area and aspect ratio
min_contour_area = 100
max_aspect_ratio = 5
valid_contours = []
for cnt in contours:
    area = cv2.contourArea(cnt)
    bbox = cv2.boundingRect(cnt)
    aspect_ratio = bbox[2] / bbox[3] if bbox[3] != 0 else 0
    if area > min_contour_area and aspect_ratio < max_aspect_ratio:
        valid_contours.append(cnt)

# Draw the detected crop rows on the original image
result_image = np.copy(image)
cv2.drawContours(result_image, valid_contours, -1, (0, 255, 0), 2)

# Display the result
cv2.imshow("Plants Contours", result_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
