import cv2
import numpy as np

# Load an image (you can replace this with your own crop field image)
image_path = "/home/mustafa/catkin_ws/src/atom/script/img/2_image_bin_3.jpg"
image = cv2.imread(image_path)

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to reduce noise and improve contour detection
blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)

# Apply adaptive thresholding to better separate objects from the background
thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

# Apply morphological operations to further clean up the image
kernel = np.ones((5,5), np.uint8)
morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

# Find contours in the processed image
contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter out small contours (adjust the threshold as needed)
min_contour_area = 100
valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

# Draw the detected crop rows on the original image
cv2.drawContours(image, valid_contours, -1, (0, 255, 0), 2)

# Display the result
cv2.imshow("Crop Rows", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
