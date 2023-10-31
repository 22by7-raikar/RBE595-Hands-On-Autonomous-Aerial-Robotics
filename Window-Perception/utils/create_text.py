# import os

# # Define the path to the "mydataset" folder
# dataset_dir = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/datasets/mydataset/"

# # Path to the "images" folder
# images_dir = os.path.join(dataset_dir, "images/train")

# # Create or open the "train.txt" file in write mode
# with open(os.path.join(dataset_dir, "train.txt"), "w") as file:
#     # List all files in the "images" folder
#     image_files = [os.path.join(images_dir, filename) for filename in os.listdir(images_dir) if filename.endswith(".png")]
    
#     # Write the paths of the image files to the "train.txt" file
#     file.write("\n".join(image_files))

# print("train.txt file has been created.")

import os

# Define the path to the "mydataset" folder
dataset_dir = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/datasets/mydataset/"

# Path to the "images" folder
images_dir = os.path.join(dataset_dir, "images/train")

# Create or open the "train.txt" file in write mode
with open(os.path.join(dataset_dir, "train.txt"), "w") as file:
    # List all files in the "images" folder
    image_files = [f"./{os.path.relpath(os.path.join(images_dir, filename), start=dataset_dir)}" for filename in os.listdir(images_dir) if filename.endswith(".png")]
    
    # Write the paths of the image files to the "train.txt" file
    file.write("\n".join(image_files))

print("train.txt file has been created.")
