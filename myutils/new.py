import os
from PIL import Image
input_folder = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/data/3masks/"
output_folder = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/data/masks/"

# Ensure the output folder exists
os.makedirs(output_folder, exist_ok=True)

for filename in os.listdir(input_folder):
    if filename.endswith(".png"):  # You can change the file extension to match your images
        img = Image.open(os.path.join(input_folder, filename))
        img_gray = img.convert("L")
        output_path = os.path.join(output_folder, filename)
        img_gray.save(output_path)

