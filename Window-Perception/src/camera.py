import bpy

# Create a new camera object
bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', location=(0, 0, 0), rotation=(0, 0, 0))

# Set the camera's name (optional)
new_camera = bpy.context.object
new_camera.name = "NN_cam"  # Replace "My_Camera" with your desired camera name
