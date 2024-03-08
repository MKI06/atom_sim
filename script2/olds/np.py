import cv2
import numpy as np



def generate_image():
    # Generate a blank white image
    image = np.ones((400, 400, 3), dtype=np.uint8) * 255
    
    # Draw a blue rectangle on the image
    cv2.rectangle(image, (50, 50), (350, 350), (255, 0, 0), 3)  # BGR color format
    
    return image

def draw_red_line_on_image(image):
    # Draw a red line on the image
    cv2.line(image, (100, 100), (300, 300), (0, 0, 255), 5)  # BGR color format
    
    # Display the image
    cv2.imshow('Image with Red Line', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Generate the image
generated_image = generate_image()


# Draw a red line on the generated image
draw_red_line_on_image(generated_image)
