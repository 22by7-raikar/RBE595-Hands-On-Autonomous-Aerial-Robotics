import cv2
import numpy as np

#flags for the drone to move
movexy = False
movex = False
movey = False
# Load the image
image_path = '/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/1_contours.png'
image = cv2.imread(image_path)

center = image.shape[1]/2,image.shape[0]/2
# Check if the image has been loaded correctly
if image is None:
    raise ValueError("Could not read the image.")

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Threshold the image to get a binary mask
# Assuming the contours are in black
_, binary_mask = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY_INV)

# Find contours
contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the largest contour by area
largest_contour = max(contours, key=cv2.contourArea)

# Create a new black image of the same size
filled_largest_contour = np.zeros_like(gray)

# Fill the largest contour with white
cv2.drawContours(filled_largest_contour, [largest_contour], -1, 255, thickness=cv2.FILLED)

# Calculate the moments of the largest contour
M = cv2.moments(largest_contour)
if M["m00"] != 0:
    # Calculate centroid
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
else:
    # Set centroid to the center of the image
    cX, cY = center
 
# Mark the centroid on the image
cv2.circle(image, (cX, cY), 15, (0, 0, 255), -1)  # Red dot
print(cX,cY)
print(center)

move = cX - center[0] , cY - center[1]
if move[0]> 20 and move[1] > 20:
    movexy = True
    print("Move in X-Y positive direction")
elif move[0] > 20:
    movex == True
    print("Move in positive X")
elif move[1] > 20:
    movey == True
    print("move in positive y")
elif move[0]< -20 and move[1] < -20:
    movexy = True
    print("Move in X-Y negative direction")
elif move[0] < -20:
    movex == True
    print("Move in negative X")
elif move[1] < -20:
    movey == True
    print("move in negative y")
else:
    Donot_move = True
    print("Do not move")
# Save or display the image with the largest contour filled and centroid marked
filled_contours_path = image_path.replace('.png', '_filled_largest_contour_centroid.png')
cv2.imwrite(filled_contours_path, image)

# Optionally, display the image
cv2.imshow('Filled Largest Contour with Centroid', image)
cv2.waitKey(100000)
cv2.destroyAllWindows()
