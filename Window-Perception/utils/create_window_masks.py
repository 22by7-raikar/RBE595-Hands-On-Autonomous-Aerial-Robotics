import os
import json
from PIL import Image, ImageDraw

# Load the JSON file with image and vertex coordinates
json_file_path = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/datasets/mydataset/annotations/image_vertex_mapping_7.json"
output_folder =  "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/datasets/mydataset/window_masks/train/7/"

with open(json_file_path, 'r') as json_file:
    image_vertex_mapping = json.load(json_file)

# Function to fill the polygons
def fill_polygons(image_path, vertex_coordinates):
    # Open the original image to get its dimensions
    original_img = Image.open(image_path)
    width, height = original_img.size
    
    # Create a blank black image of the same dimensions
    img = Image.new('RGB', (width, height), color='black')
    draw = ImageDraw.Draw(img)

    # Fill the outer polygon with white color
    outer_coords = [tuple(vertex_coordinates[idx]['image_coordinates']) for idx in [0, 1, 3, 2]]
    draw.polygon(outer_coords, fill="white", outline="white")
    
    # Fill the inner polygon with black color
    inner_coords = [tuple(vertex_coordinates[idx]['image_coordinates']) for idx in [4, 5, 6, 7]]
    draw.polygon(inner_coords, fill="black", outline="black")

    return img

# Process each image and fill the polygons
for image_path, vertex_coordinates in image_vertex_mapping.items():
    output_image = fill_polygons(image_path, vertex_coordinates)

    # Save the modified image to the output folder
    image_filename = os.path.basename(image_path)
    output_image_path = os.path.join(output_folder, image_filename)
    output_image.save(output_image_path)

    print(f"Processed image: {output_image_path}")

print("All images processed and saved to the output folder.")