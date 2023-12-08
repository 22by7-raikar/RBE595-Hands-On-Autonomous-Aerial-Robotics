import cv2
import numpy as np

# Read the image
window = cv2.imread('/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/colorthresholding/dynamicwindow.png')

# Convert the image from BGR to RGB
window_rgb = cv2.cvtColor(window, cv2.COLOR_BGR2RGB)

# Convert the RGB image to HSV
window_hsv = cv2.cvtColor(window_rgb, cv2.COLOR_RGB2HSV)

# Define the pink color range in HSV
light_pink = (161, 50, 50)
dark_pink = (171, 255, 255)

# Create a mask using inRange function for the pink hand
mask_pink_hand = cv2.inRange(window_hsv, light_pink, dark_pink)

# Define the blue color range in HSV for the window
lower_blue = np.array([90, 50, 50], dtype=np.uint8)
upper_blue = np.array([120, 255, 255], dtype=np.uint8)

# Create a mask using inRange function for the blue window
mask_blue_window = cv2.inRange(window_hsv, lower_blue, upper_blue)

# Combine pink and blue masks using bitwise OR
combined_mask = cv2.bitwise_or(mask_pink_hand, mask_blue_window)

# Save the combined mask image
cv2.imwrite('/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/colorthresholding/dynamicwindow_combined_mask.png', combined_mask)

# Apply the combined mask to the original image
result_image = cv2.bitwise_and(window_rgb, window_rgb, mask=combined_mask)

# Save the result image
cv2.imwrite('/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/colorthresholding/dynamicwindow_result.png', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
