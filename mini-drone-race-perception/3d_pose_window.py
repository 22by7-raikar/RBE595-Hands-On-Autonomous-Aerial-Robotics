import cv2
import numpy as np

# Load your camera calibration matrix K
# You can replace the values below with your actual camera parameters
K = np.array([[1378.6998, 0, 624.4608],
              [0, 1378.2931, 362.5225],
              [0, 0, 1]]
)

# Known real-world dimensions of the window
window_width = 838  # Width of the window in millimeters
window_height = 838  # Height of the window in millimeters

def mark_polygon_corners(input_image_path, output_image_path):
    # Read the image
    img = cv2.imread(input_image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find edges using Canny edge detection
    edges = cv2.Canny(gray, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Assuming the largest contour is the outer polygon and the second largest is the inner one
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    inner_contour = contours[1]

    # Approximate the contour to get the corners
    epsilon = 0.02 * cv2.arcLength(inner_contour, True)
    corners = cv2.approxPolyDP(inner_contour, epsilon, True)

    # Draw corners on the image
    for point in corners:
        cv2.circle(img, (point[0][0], point[0][1]), 5, (0, 0, 255), -1)  # Draw a red circle for each corner

    # Save the image with marked corners
    cv2.imwrite(output_image_path, img)

    # Prepare your 2D image points
    image_points = np.array([corner[0].tolist() for corner in corners], dtype=np.float32)

    # List to store the 3D world coordinates of the corners
    world_coordinates = []

    # Triangulation for each corner
    for image_point in image_points:
        # Perform triangulation for each corner
        A = np.linalg.inv(K).dot(np.array([image_point[0], image_point[1], 1]))
        x_world = A[0] * window_width
        y_world = A[1] * window_height
        z_world = A[2]

        # Append the 3D world coordinates of the corner to the list
        world_coordinates.append([x_world, y_world, z_world])

    # Use solvePnP to estimate the pose
    success, rotation_vector, translation_vector = cv2.solvePnP(np.array(world_coordinates), image_points, K, None)

    if success:
        # `rotation_vector` contains the rotation parameters (R)
        print("Rotation Vector (R):")
        print(rotation_vector)

        # `translation_vector` contains the translation parameters (T)
        print("Translation Vector (T):")
        print(translation_vector)
    else:
        print("solvePnP failed to estimate the pose.")

    return image_points, world_coordinates  # Returning both image points and world coordinates

if __name__ == "__main__":
    output_path = "/home/hasithab/Desktop/rbe595_aerial_vehicles/p3a/marked_images-20231030T033410Z-001/marked_images/inner_corners/render0.5_0.5.png"
    input_path = "/home/hasithab/Desktop/rbe595_aerial_vehicles/p3a/marked_images-20231030T033410Z-001/marked_images/render0.5_0.5.png"
    corners, world_coords = mark_polygon_corners(input_path, output_path)

    for idx, corner in enumerate(corners, start=1):
        print(f"Corner {idx}: {corner}")

    for idx, world_coord in enumerate(world_coords, start=1):
        print(f"3D World Coordinates for Corner {idx}: {world_coord}")

    # For debugging, display the marked corners in an OpenCV window
    cv2.imshow("Marked Corners", cv2.imread(output_path))
    cv2.waitKey(0)

    # Close the OpenCV window properly
    cv2.destroyAllWindows()