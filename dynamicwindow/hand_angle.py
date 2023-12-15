import cv2
import numpy as np

# Convert the RGB image to HSV
def get_mask_fit_line(image, color_one, color_two, color_three, color_four):
    window_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)                    # Convert the image from BGR to RGB
    window_hsv = cv2.cvtColor(window_rgb, cv2.COLOR_RGB2HSV)                # Convert the image from RGB to HSV
    mask_pink_hand = cv2.inRange(window_hsv, color_one, color_two)          # Create a mask using inRange function for the pink hand
    mask_blue_window = cv2.inRange(window_hsv, color_three, color_four)     # Create a mask using inRange function for the blue window

    combined_mask = cv2.bitwise_or(mask_pink_hand, mask_blue_window)        # Combine pink and blue masks using bitwise OR
    cv2.imwrite('/home/ankush/Desktop/dynamicwindow_combined_mask.png', mask_pink_hand) # Save the combined mask image
    
    result_image = cv2.bitwise_and(window_rgb, window_rgb, mask=combined_mask)          # Apply the combined mask to the original image
    cv2.imwrite('/home/ankush/Desktop/dynamicwindow_result.png', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)) # Save the result image

    return result_image, mask_pink_hand, mask_blue_window, combined_mask


def get_edges(image, t1=100, t2=200):
    edges = cv2.Canny(image, t1, t2)  # Apply Canny edge detector to the result image
    edges_colored = np.zeros_like(image)  # Create a blank image with the same dimensions as the input

    # Find the indices where edges are present
    y_indices, x_indices = np.where(edges == 255)

    # Set the color of the edges (e.g., red) at these indices
    for y, x in zip(y_indices, x_indices):
        edges_colored[y, x, :] = [0, 0, 255]  # Assign color to all three channels

    result_with_edges = cv2.addWeighted(image, 1, edges_colored, 1, 0)  # Overlay the edges on the original image
    cv2.imwrite('/home/ankush/Desktop/dynamicwindow_with_edges.png', result_with_edges)  # Save the result

    return result_with_edges


def draw_line_on_mask(mask, original_image):
    # Find edges in the mask
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)

    # Find lines in the edge image using Hough Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    # Calculate the angle of each line with respect to the horizontal axis
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle_radians = np.arctan2(y2 - y1, x2 - x1)
            angle_degrees = np.degrees(angle_radians)
            print(f"Angle with respect to horizontal: {angle_degrees} degrees")

    # Display the image with lines
    image_with_lines = cv2.cvtColor(original_image.copy(), cv2.COLOR_BGR2RGB)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image_with_lines, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # cv2.imshow('Image with Lines', image_with_lines)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return image_with_line, angle_degrees


# Read the image
window = cv2.imread('/home/ankush/Downloads/2.png')

# Define the pink color range in HSV
light_pink = (150, 50, 50)  #c1
dark_pink = (171, 255, 255) #c2

# Define the blue color range in HSV for the window
lower_blue = np.array([110, 50, 50], dtype=np.uint8)     #c3
upper_blue = np.array([120, 255, 255], dtype=np.uint8)   #c4
_, mph, mbw, cm = get_mask_fit_line(window, light_pink, dark_pink, lower_blue, upper_blue)
# result_with_edges = get_edges(mph)
image_with_line, angle_in_degrees = draw_line_on_mask(mph, window)
# cv2.imshow('Image with Lines', image_with_line)
