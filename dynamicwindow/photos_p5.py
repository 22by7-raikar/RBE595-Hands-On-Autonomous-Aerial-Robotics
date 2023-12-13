import bpy
import random

# Set the world background to white
bpy.context.scene.world.use_nodes = False
bpy.context.scene.world.color = (1, 1, 1)

# Set rendering settings
bpy.context.scene.render.image_settings.file_format = 'PNG'
output_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/58/'

# Store the initial camera location
initial_location = bpy.context.scene.camera.location.copy()

# Define parameters for camera movement
step_size = 1.0
num_steps = 5

# Iterate over X, Y, and Z directions
for direction in ['X', 'Y', 'Z']:
    # Reset camera location to the initial location
    bpy.context.scene.camera.location = initial_location

    # Move the camera along the positive direction
    for step in range(num_steps):
        # Adjust camera location based on the step size
        exec(f"bpy.context.scene.camera.location.{direction.lower()} += step_size")

        # Set the output file path for each image
        file_path = f"{output_path}rendered_image_{direction}_positive_{step + 1}.png"
        bpy.context.scene.render.filepath = file_path

        # Render the image
        bpy.ops.render.render(write_still=True)

    # Reset camera location to the initial location
    bpy.context.scene.camera.location = initial_location

    # Move the camera along the negative direction
    for step in range(num_steps):
        # Adjust camera location based on the step size
        exec(f"bpy.context.scene.camera.location.{direction.lower()} -= step_size")

        # Set the output file path for each image
        file_path = f"{output_path}rendered_image_{direction}_negative_{step + 1}.png"
        bpy.context.scene.render.filepath = file_path

        # Render the image
        bpy.ops.render.render(write_still=True)

# Reset the camera location to the initial location
bpy.context.scene.camera.location = initial_location

# Reset the file path to the original value
bpy.context.scene.render.filepath = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/rendered_image.png'
