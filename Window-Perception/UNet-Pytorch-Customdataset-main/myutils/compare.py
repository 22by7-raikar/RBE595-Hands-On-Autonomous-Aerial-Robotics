import os

# Paths to the two folders you want to compare
folder2_path = '/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/rename_data/imgspng/'
folder1_path = '/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/new_data/masks/'
# /home/anuj/Desktop/AerialRobotics/apairaikar_p3a/UNet-Pytorch-Customdataset-main/data
# Get a list of file names in each folder
folder1_files = set(os.listdir(folder1_path))
folder2_files = set(os.listdir(folder2_path))

# Find files that are unique to each folder
unique_to_folder1 = set(folder1_files) - set(folder2_files)
unique_to_folder2 = set(folder2_files) - set(folder1_files)

# Delete files that are unique to folder1
for file_name in unique_to_folder1:
    file_path = os.path.join(folder1_path, file_name)
    os.remove(file_path)
    print(f"Deleted: {file_path}")

# Delete files that are unique to folder2
for file_name in unique_to_folder2:
    file_path = os.path.join(folder2_path, file_name)
    os.remove(file_path)
    print(f"Deleted: {file_path}")

print("Cleanup complete.")