import cv2
import numpy as np

def find_shape_center(image_path):
    # Load the image
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Preprocess the image (you might need to adjust these parameters)
    _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print("No contours found.")
        return None, None

    # Draw all contours for debugging
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

    # Assume the largest contour is the desired shape
    largest_contour = max(contours, key=cv2.contourArea)

    # Compute the center of the contour
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        # Default to center of image if contour has no area (to avoid division by zero)
        cX, cY = image.shape[1] // 2, image.shape[0] // 2

    
    # Print the center
    print(f"The center of the shape is at ({cX}, {cY})")

    # Mark the center on the image
    cv2.circle(image, (cX, cY), 25, (255, 0, 0), -1)

    # Display the image with contours and center marked
    cv2.imshow("Center of Shape", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return cX, cY

# Example usage
#center_x, center_y = find_shape_center('path_to_your_image.png')
center_x, center_y = find_shape_center('/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/1_contours.png')

