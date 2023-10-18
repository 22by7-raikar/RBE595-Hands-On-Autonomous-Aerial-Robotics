import os
import yaml
from scipy import io
import bpy

curr_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(curr_dir, 'cfg.yaml')) as file:
    try:
        cfg = yaml.safe_load(file)
    except yaml.YAMLError as exception:
        print(exception)

# Clear existing mesh objects
bpy.ops.wm.read_factory_settings(use_empty=True)

def create_boundary(x0, y0, z0, x1, y1, z1):
    # Create a new mesh for the boundary
    bpy.ops.mesh.primitive_cube_add(size=1)
    boundary = bpy.context.object
    boundary.scale = [(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2]
    boundary.location = [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]

def create_window(x, y, z, xdelta, ydelta, zdelta, qw, qx, qy, qz, xangdelta, yangdelta, zangdelta, texture_path):
    # Create a new mesh for the window
    bpy.ops.mesh.primitive_cube_add(size=1)
    window = bpy.context.object
    window.scale = [xdelta, ydelta, zdelta]
    window.location = [x, y, z]

    # Set the window orientation using the quaternion
    window.rotation_quaternion = [qw, qx, qy, qz]

    # Apply Euler angle variation
    bpy.context.active_object.rotation_euler = [xangdelta, yangdelta, zangdelta]

    # Add a texture to the window object
    if texture_path:
        # Create a new material
        window_material = bpy.data.materials.new(name="Window_Material")

        # Assign the material to the window object
        window.data.materials.append(window_material)

        # Create an image texture node
        image_texture = window_material.node_tree.nodes.new('ShaderNodeTexImage')
        image_texture.image = bpy.data.images.load(texture_path)

        # Connect the image texture to the material's surface
        shader_node = window_material.node_tree.nodes["Principled BSDF"]
        window_material.node_tree.links.new(shader_node.inputs["Base Color"], image_texture.outputs["Color"])

# Define the boundary and window data file
env_path = cfg['env']['main_path']
map_file = cfg['env']['map_file']
env_file = os.path.expanduser(os.path.join(env_path, map_file))
env = io.loadmat(env_path)

with open(env, 'r') as file:
    lines = file.readlines()

for line in lines:
    tokens = line.strip().split()
    
    if tokens[0] == 'boundary':
        x0, y0, z0, x1, y1, z1 = map(float, tokens[1:])
        create_boundary(x0, y0, z0, x1, y1, z1)
    
    elif tokens[0] == 'window':
        x, y, z, xdelta, ydelta, zdelta, qw, qx, qy, qz, xangdelta, yangdelta, zangdelta = map(float, tokens[1:])
        texture_location = cfg["textures"]["main_path"]
        texture_file = cfg["textures"]["pic"]  # Set your texture path here
        texture_path = os.path.expanduser(os.path.join(texture_location, texture_file))
        create_window(x, y, z, xdelta, ydelta, zdelta, qw, qx, qy, qz, xangdelta, yangdelta, zangdelta, texture_path)


