import numpy as np
import cv2 as cv

# Load the image
image_path = "/home/mustafa/catkin_ws/src/atom/script2/img/9_image_hough_4.jpg"
image = cv.imread(image_path)

assert image is not None, "file could not be read, check with os.path.exists()"
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)

# noise removal
kernel = np.ones((3, 3), np.uint8)
opening = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel, iterations=2)

# sure background area
sure_bg = cv.dilate(opening, kernel, iterations=3)

# Finding sure foreground area
dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 5)
ret, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)

# Finding unknown region
sure_fg = np.uint8(sure_fg)
unknown = cv.subtract(sure_bg, sure_fg)

# Marker labelling
ret, markers = cv.connectedComponents(sure_fg)
# Add one to all labels so that sure background is not 0, but 1
markers = markers + 1
# Now, mark the region of unknown with zero
markers[unknown == 255] = 0

markers = cv.watershed(image, markers)
image[markers == -1] = [255, 0, 0]

# Convert the image to HSV color space
hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

# Define range of green color in HSV
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

# Threshold the HSV image to get only green colors
mask = cv.inRange(hsv, lower_green, upper_green)

# Find contours
contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# Sort contours based on their x-coordinate (left to right)
contours = sorted(contours, key=lambda x: cv.boundingRect(x)[0])

# Draw contours on original image
drawn = cv.drawContours(image, contours, -1, (0, 0, 255), 1)

# Store the centers of filled areas
centers = []

# Paint the area between blue and red lines and store centers
for cnt in contours:
    filled = cv.fillPoly(drawn, [cnt], (0, 255, 0))
    
    # Calculate center of filled area
    M = cv.moments(cnt)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        # Put a small dot at the center
        cv.circle(image, (cx, cy), 3, (255, 255, 255), -1)
        centers.append((cx, cy))

# Function to calculate Euclidean distance between two points
def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# Connect the closest dots
for i in range(len(centers)):
    min_dist = float('inf')
    closest_point = None
    for j in range(i+1, len(centers)):
        dist = euclidean_distance(centers[i], centers[j])
        if dist < min_dist:
            min_dist = dist
            closest_point = j
    if closest_point is not None:
        cv.line(image, centers[i], centers[closest_point], (0, 255, 255), 1)

cv.imshow("area51", image)
cv.waitKey(0)
cv.destroyAllWindows()
