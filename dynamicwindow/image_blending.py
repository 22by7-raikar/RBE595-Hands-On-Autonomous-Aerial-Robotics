from PIL import Image
import os

def blend_images(folder1, folder2, output_folder, alpha=0.5):
    # Get a list of files in folder1 and folder2
    files1 = os.listdir(folder1)
    files2 = []
    for subfolder in os.listdir(folder2):
        subfolder_path = os.path.join(folder2, subfolder)
        if os.path.isdir(subfolder_path):
            files2.extend([os.path.join(subfolder_path, filename) for filename in os.listdir(subfolder_path)])

    # Create the output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)

    # Iterate over all pairs of images from folder1 and folder2
    for file1 in files1:
        for file2_path in files2:
            # Open images from both folders
            image1 = Image.open(os.path.join(folder1, file1))
            image2 = Image.open(file2_path)

            # Resize images to a common size
            common_size = (min(image1.width, image2.width), min(image1.height, image2.height))
            image1 = image1.resize(common_size)
            image2 = image2.resize(common_size)

            # Ensure both images have the same format
            if image1.mode != image2.mode:
                image2 = image2.convert(image1.mode)

            # Blend images
            blended_image = Image.blend(image1, image2, alpha)

            # Create a unique output file name based on the input image names and subfolder name
            file1_name = os.path.splitext(os.path.basename(file1))[0]
            file2_name = os.path.splitext(os.path.basename(file2_path))[0]
            subfolder_name = os.path.basename(os.path.dirname(file2_path))
            output_file = os.path.join(output_folder, f"blended_{file1_name}_{subfolder_name}_{file2_name}.png")

            # Save the resulting image
            blended_image.save(output_file)

    print(f"Blending completed. Blended images saved in {output_folder}.")

# Example usage
folder1_path = "/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/Textures"
folder2_path = "/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images"
output_folder_path = "/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/blended_images"
blend_images(folder1_path, folder2_path, output_folder_path, alpha=0.5)
