import numpy as np
import matplotlib.pyplot as plt
import struct
import cv2
import matplotlib.colors as mcolors

def read_flo_file(file_path):
    with open(file_path, 'rb') as file:
        # Read the header
        header = file.read(4)
        if header != b'PIEH':
            raise ValueError("Invalid .flo file")

        width = struct.unpack('i', file.read(4))[0]
        height = struct.unpack('i', file.read(4))[0]

        # Read the flow data
        data = np.zeros((height, width, 2), dtype=np.float32)
        for y in range(height):
            for x in range(width):
                data[y, x, 0] = struct.unpack('f', file.read(4))[0]
                data[y, x, 1] = struct.unpack('f', file.read(4))[0]

    return data

import matplotlib.colors as mcolors


def visualize_flow(flow_data,):
    # Ensure the flow_data has the correct shape (height, width, 2)
    if flow_data.ndim != 3 or flow_data.shape[2] != 2:
        raise ValueError("Invalid flow_data shape. It should have shape (height, width, 2).")

    height, width, _ = flow_data.shape

    # Calculate the magnitude of flow vectors
    magnitude = np.sqrt(flow_data[:, :, 0] ** 2 + flow_data[:, :, 1] ** 2)

    # Create a colormap for flow magnitude
    cmap = plt.get_cmap('jet')
    norm = plt.Normalize(vmin=magnitude.min(), vmax=magnitude.max())
    
    # Apply the colormap to the magnitudes
    flow_color = cmap(norm(magnitude))

    # Display the visualized flow
    plt.imshow(flow_color)
    plt.axis('off')

    # Save the image
    save_path = '/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/1.png'  # Update this path as needed
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Optionally, display the image
    saved_image = plt.imread(save_path)
    plt.imshow(saved_image)
    plt.axis('off')
    plt.show()

    # Load the saved image
    saved_image = cv2.imread(save_path)

    # Convert to grayscale for contour detection
    gray_image = cv2.cvtColor(saved_image, cv2.COLOR_BGR2GRAY)
    
    # Optional: Apply Gaussian blur to smooth out the image
    blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

    # Use Canny edge detection, consider adjusting the thresholds
    edges = cv2.Canny(blurred_image, threshold1=30, threshold2=100)

    # Optional: Dilate the edges to close gaps
    kernel = np.ones((5, 5), np.uint8)
    dilated_edges = cv2.dilate(edges, kernel, iterations=1)

    # Find contours from the dilated Canny edges
    contours, _ = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours
    contour_image = cv2.drawContours(saved_image.copy(), contours, -1, (0, 0, 0), 4)

    
    # Save or display the image with contours
    contour_path = save_path.replace('.png', '_contours.png')
    cv2.imwrite(contour_path, contour_image)

    # Optionally, display the image
    cv2.imshow('Flow with Contours', contour_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



# Usage:
flow_data = read_flo_file('/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/out.flo')
visualize_flow(flow_data)
