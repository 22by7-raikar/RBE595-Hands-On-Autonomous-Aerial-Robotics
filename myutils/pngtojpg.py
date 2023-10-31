from PIL import Image
import os

# Specify the directory containing the PNG images
png_directory = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/data/imgspng/"

# Specify the directory where you want to save the JPG images
jpg_directory = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/data/imgsjpg/"

# Create the output directory if it doesn't exist
if not os.path.exists(jpg_directory):
    os.makedirs(jpg_directory)

# Loop through the PNG images in the input directory
for filename in os.listdir(png_directory):
    if filename.endswith(".png"):
        # Open the PNG image
        png_image = Image.open(os.path.join(png_directory, filename))
        
        # Convert and save it as JPG in the output directory
        jpg_filename = os.path.splitext(filename)[0] + ".jpg"
        jpg_image = png_image.convert("RGB")
        jpg_image.save(os.path.join(jpg_directory, jpg_filename), "JPEG")

# Print a message when the conversion is complete
print("Conversion from PNG to JPG completed.")