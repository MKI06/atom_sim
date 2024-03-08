import cv2
import numpy as np

# Load an image (you can replace this with your own crop field image)
image_path = "/home/mustafa/catkin_ws/src/atom/script2/img/nihai_3.jpg"
image = cv2.imread(image_path)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Threshold the image
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Find contours
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the minimum bounding rectangle that encloses all the contours
rect = cv2.minAreaRect(np.concatenate(contours))

# Get the coordinates of the rectangle's vertices
box = cv2.boxPoints(rect)
box = np.int0(box)

# Draw the rectangle on the original image
cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

# Draw a line connecting opposite corners of the rectangle
cv2.line(image, tuple(box[0]), tuple(box[2]), (0, 0, 255), 2)

# Display the image
cv2.imshow('Bounding Rectangle with Line', image)
cv2.waitKey(0)
cv2.destroyAllWindows()