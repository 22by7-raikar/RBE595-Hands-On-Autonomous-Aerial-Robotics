import json

# Load your JSON data
json_path = '/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/datasets/mydataset/annotations/image_vertex_mapping_20.json'
with open(json_path, 'r') as json_file:
    data = json.load(json_file)

# Define the image dimensions (width and height)
image_width = 300  # Replace with your image width
image_height = 200  # Replace with your image height

# Define a mapping of keypoints to class indices based on your data.yaml
keypoint_to_class_mapping = {
    0: 0,
    1: 1,
    2: 2,
    3: 3,
    4: 4,
    5: 5,
    6: 6,
    7: 7
}

for image_path, keypoints in data.items():
    label_path = image_path.replace('/raw_images/train/20/', '/labels/train/20/').replace('.png', '.txt')

    with open(label_path, 'w') as label_file:
        for keypoint in keypoints:
            ax, ay = keypoint['image_coordinates']
            x_normalized = ax / image_width
            y_normalized = ay / image_height

            # Map your keypoints to class indices using the mapping
            class_index = keypoint_to_class_mapping.get(keypoint['vertex_index'], 0)

            label_file.write(f'{class_index} {x_normalized} {y_normalized} 0\n')
