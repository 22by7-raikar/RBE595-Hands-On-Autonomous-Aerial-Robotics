iimport cv2
import numpy as np

# Load the main image with the gray and white background and a window at the center
main_image = cv2.imread("/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/new_data/imgs/render0_0_2.0_6.jpg")

# Load the background image that you want to add
background_image = cv2.imread("/home/anuj/Pictures/glass.jpg", cv2.IMREAD_COLOR).astype(np.uint8)


# Create a mask for the window region in the main image
gray_main_image = cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)
_, window_mask = cv2.threshold(gray_main_image, 200, 255, cv2.THRESH_BINARY)

# Invert the mask to select the background region
inverse_mask = cv2.bitwise_not(window_mask)

# Extract the window region from the main image
window_region = cv2.bitwise_and(main_image, main_image, mask=window_mask)

# Extract the background region from the background image
background_region = cv2.bitwise_and(background_image, background_image, mask=inverse_mask)

# Combine the window region and the background region
result = cv2.add(window_region, background_region)

# Save the result
cv2.imwrite("/home/anuj/Desktop/output_image.jpg", result)

# Display the result
cv2.imshow("Result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
