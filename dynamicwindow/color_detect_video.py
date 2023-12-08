import cv2
import numpy as np

def process_frame(frame):
    window_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    window_hsv = cv2.cvtColor(window_rgb, cv2.COLOR_RGB2HSV)

    light_pink = (161, 50, 50)
    dark_pink = (171, 255, 255)

    light_blue = (90, 50, 50)
    dark_blue = (120, 255, 255)

    mask_pink_hand = cv2.inRange(window_hsv, light_pink, dark_pink)
    mask_blue_window = cv2.inRange(window_hsv, light_blue, dark_blue)

    combined_mask = cv2.bitwise_or(mask_pink_hand, mask_blue_window)
    result = cv2.bitwise_and(window_rgb, window_rgb, mask=combined_mask)

    return result

    # Find contours for the pink hand
    contours_pink_hand, _ = cv2.findContours(mask_pink_hand, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find contours for the blue window
    contours_blue_window, _ = cv2.findContours(mask_blue_window, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create an empty frame for drawing contours
    frame_with_contours = window_rgb.copy()

    # Draw contours for the blue window on the frame with contours
    cv2.drawContours(frame_with_contours, contours_blue_window, -1, (0, 0, 0), thickness=1)

    # Draw contours for the pink hand on the frame with contours
    cv2.drawContours(frame_with_contours, contours_pink_hand, -1, (0, 255, 0), thickness=1)

    return frame_with_contours



# Input and output file paths
input_video_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dynamicwindowinput.mp4'
output_video_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dynamicwindowoutput.mp4'

# Open the input video file
cap = cv2.VideoCapture(input_video_path)

# Get video properties
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can change the codec if needed
out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

# Process each frame in the input video
while True:
    ret, frame = cap.read()
    if not ret:
        break  # Break the loop if no more frames

    processed_frame = process_frame(frame)

    # Display the processed frame (optional)
    cv2.imshow("Processed Frame", cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR))

    # Write the processed frame to the output video
    out.write(processed_frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()
