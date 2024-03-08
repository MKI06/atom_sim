#import dependencies
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load image using cv's imread(nameoffile)
img = cv2.imread('/home/mustafa/catkin_ws/src/atom/script2/img/9_image_hough_4.jpg')

# Split the image into blue, green, and red channels
b,g,r = cv2.split(img) 
# Amplify the color green to stand out, without red/blue
gscale = 2*g-r-b  # We are going to refer to this as our grayscale img

# Canny edge detection 
gscale = cv2.Canny(gscale, 400, 400, apertureSize=3)

# Checking the results (good practice)
plt.figure()
plt.plot(), plt.imshow(gscale)
plt.title('Canny Edge-Detection Results')
plt.xticks([]), plt.yticks([])
plt.show()

# Thresholding and skeletonization
size = np.size(gscale)
skel = np.zeros(gscale.shape, np.uint8)
ret, gscale = cv2.threshold(gscale, 128, 255, 0)
element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
done = False
while not done:
    eroded = cv2.erode(gscale, element)
    temp = cv2.dilate(eroded, element)
    temp = cv2.subtract(gscale, temp)
    skel = cv2.bitwise_or(skel, temp)
    gscale = eroded.copy()
    zeros = size - cv2.countNonZero(gscale)
    if zeros == size:
        done = True

# Hough lines detection
lines = cv2.HoughLines(skel, 1, np.pi/180, 130)
if lines is not None:
    for rho, theta in lines[:, 0]:
        # Convert polar coordinates to Cartesian coordinates
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        # Adjust line length to cover the entire width of the image
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        # Draw the lines on the skeleton image
        cv2.line(skel, (x1, y1), (x2, y2), (255, 0, 0), 2, cv2.LINE_AA)

# Showing the results
plt.subplot(121)
# OpenCV reads images as BGR, this corrects so it is displayed as RGB
plt.plot(), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)) 
plt.title('Original Image with Detected Rows'), plt.xticks([]), plt.yticks([])
plt.subplot(122)
plt.plot(), plt.imshow(skel, cmap='gray')
plt.title('Skeletal Image with Detected Rows'), plt.xticks([]), plt.yticks([])
plt.show()
