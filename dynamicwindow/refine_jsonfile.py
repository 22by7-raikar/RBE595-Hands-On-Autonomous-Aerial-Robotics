import os
import json

def get_filename_from_path(path):
    return os.path.basename(path)

def remove_nonexistent_entries(json_file_path, folder_path):
    # Load existing JSON data
    with open(json_file_path, 'r') as json_file:
        image_vertex_mapping = json.load(json_file)

    # Get a list of all files in the folder
    all_files = [get_filename_from_path(f) for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

    # Filter entries in JSON for existing files
    filtered_mapping = {file_name: data for file_name, data in image_vertex_mapping.items() if get_filename_from_path(file_name) in all_files}

    # Save the filtered data back to the JSON file
    with open(json_file_path, 'w') as json_file:
        json.dump(filtered_mapping, json_file, indent=4)

    print(f"Removed nonexistent entries from {json_file_path}.")

# Specify the path to your JSON file and folder
json_file_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/annotations/image_vertex_mapping_54.json'
folder_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/54'

remove_nonexistent_entries(json_file_path, folder_path)
