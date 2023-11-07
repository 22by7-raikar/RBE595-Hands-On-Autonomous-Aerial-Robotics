import json
import csv
import os
import glob

# Define the folder containing the JSON files
json_folder = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p3a/Window-Perception/datasets/mydataset/annotations/'

# Get a list of all JSON files in the folder
json_files = glob.glob(os.path.join(json_folder, '*.json'))

# Iterate through the JSON files
for json_file_path in json_files:  # Change variable name to json_file_path
    # Load the JSON data from the file
    with open(json_file_path, 'r') as json_file:  # Change variable name to json_file
        data = json.load(json_file)

    # Create the CSV file name by replacing the extension with .csv
    csv_file = os.path.splitext(json_file_path)[0] + '.csv'

    # Open a CSV file for writing
    with open(csv_file, 'w', newline='') as csv_file_obj:  # Change variable name to csv_file_obj
        writer = csv.writer(csv_file_obj)

        # Write the header row with the column names you specified
        writer.writerow(['filename', 'x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'x4', 'y4', 'x5', 'y5', 'x6', 'y6', 'x7', 'y7', 'x8', 'y8'])

        # Iterate through the JSON data and write to the CSV file
        for image_file, vertices in data.items():
            # Extract just the last part of the file name
            file_name = os.path.basename(image_file)

            # Create a list to store the values for each row
            row_values = [file_name]

            for vertex in vertices:
                image_coordinates = vertex['image_coordinates']
                x_coordinate, y_coordinate = image_coordinates
                row_values.extend([x_coordinate, y_coordinate])

            # Ensure the row has 16 values by adding empty strings if necessary
            row_values.extend([''] * (16 - len(row_values)))

            # Write the row to the CSV file
            writer.writerow(row_values)
