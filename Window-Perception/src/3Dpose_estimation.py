import cv2
import numpy as np
import matplotlib.pyplot as plt

# Known real-world dimensions of the window
width = 83.3 # Replace with actual width
height = 83.3 # Replace with actual height
K = np.array([[1378.6998, 0, 624.4608],
                [0, 1378.2931, 362.5225],
                [0, 0, 1]])

def get_pose(path):
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray, 50, 150)     # Find edges using Canny edge detection

    # No lens distortion
    distCoeffs = np.zeros((4, 1))

    # Find contours
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Assuming the largest contour is the outer polygon and the second largest is the inner one
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    inner_contour = contours[2]

    # Approximate the contour to get the corners
    epsilon = 0.02 * cv2.arcLength(inner_contour, True)
    corners = cv2.approxPolyDP(inner_contour, epsilon, True)

    # Prepare your 2D image points
    image_points = np.array([corner[0].tolist() for corner in corners], dtype=np.float32)
    centroid = np.mean(image_points, axis=0)

    # Sort the points based on their distance to the centroid
    sorted_image_points = sorted(image_points, key=lambda p: np.arctan2(p[1] - centroid[1], p[0] - centroid[0]))

    # Define the 3D points of the window in world coordinates
    objectPoints = np.array([
        [0, 0, 0],        # Bottom-left
        [width, 0, 0],    # Bottom-right
        [width, height, 0],# Top-right
        [0, height, 0]    # Top-left
    ], dtype="double")

    # Solve the PnP problem
    success, rotation_vector, translation_vector = cv2.solvePnP(objectPoints, sorted_image_points, K, distCoeffs)

    # Convert the rotation vector into a rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

    # Transform the object points to the camera coordinate system
    object_points_camera = np.dot(rotation_matrix, objectPoints.T).T + translation_vector.T

    # Extracting the x, y, and z coordinates
    x_vals = object_points_camera[:, 0]
    y_vals = object_points_camera[:, 1]
    z_vals = object_points_camera[:, 2]

    return object_points_camera
