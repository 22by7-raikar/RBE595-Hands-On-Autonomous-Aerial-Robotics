import os
import json
import csv

def normalize_path(path):
    return os.path.normpath(path)

def remove_nonexistent_entries(csv_file_path, folder_path):
    # Load existing CSV data
    with open(csv_file_path, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        rows = list(csv_reader)

    # Get a list of all files in the folder
    all_files = [normalize_path(f) for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

    # Filter entries in CSV for existing files
    filtered_rows = [row for row in rows if normalize_path(row['filename']) in all_files]

    # Save the filtered data back to the CSV file
    fieldnames = csv_reader.fieldnames
    with open(csv_file_path, 'w', newline='') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
        csv_writer.writerows(filtered_rows)

    print(f"Removed nonexistent entries from {csv_file_path}.")

# Specify the path to your CSV file and folder
csv_file_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/annotations/image_vertex_mapping_45.csv'
folder_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/45'

remove_nonexistent_entries(csv_file_path, folder_path)
