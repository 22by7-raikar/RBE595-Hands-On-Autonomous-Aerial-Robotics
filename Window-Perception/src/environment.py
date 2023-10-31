import numpy as np
import random
import math
import bpy
import os
import yaml

curr_dir = os.path.dirname('/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/')

with open(os.path.join(curr_dir, 'src/configs/cfg.yaml')) as file:
    try:
        print(curr_dir)
        cfg = yaml.safe_load(file)

    except yaml.YAMLError as exception:
        print(exception)

env_path = cfg['env']['main_path']
map_file = cfg['env']['map_file']
env_file = os.path.expanduser(os.path.join(env_path, map_file))


def create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b):
    bpy.ops.mesh.primitive_cube_add(scale=((xmax-xmin)/2, (ymax-ymin)/2, (zmax-zmin)/2))
    block = bpy.context.active_object
    block.location.x = (xmax+xmin) / 2
    block.location.y = (ymax+ymin) / 2
    block.location.z = (zmax+zmin) / 2
    material = bpy.data.materials.new(name="BlockMaterial")
    block.data.materials.append(material)
    material.diffuse_color = (r / 255, g / 255, b / 255, 1)


def create_sphere(location, r, g, b, radius):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=location)
    sphere = bpy.context.active_object
    material = bpy.data.materials.new(name="SphereMaterial")
    sphere.data.materials.append(material)
    material.diffuse_color = (r / 255, g / 255, b / 255, 1)
   

def create_boundary(xmin, ymin, zmin, xmax, ymax, zmax, transparency):
    bpy.ops.mesh.primitive_cube_add(scale=((xmax-xmin)/2, (ymax-ymin)/2, (zmax-zmin)/2))
    boundary = bpy.context.active_object
    boundary.display_type = 'WIRE'
    boundary.location.x = (xmax+xmin) / 2
    boundary.location.y = (ymax+ymin) / 2
    boundary.location.z = (zmax+zmin) / 2
    material = bpy.data.materials.new(name="BoundaryMaterial")
    boundary.data.materials.append(material)
    material.diffuse_color = (0, 0, 0, 1)
    material.use_nodes = True
    principled_bsdf = material.node_tree.nodes.get('Principled BSDF')
    if principled_bsdf:
        principled_bsdf.inputs['Alpha'].default_value = 1 - transparency


def create_cylinder(p1, p2, radius=0.01):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    dist = ((dx)**2 + (dy)**2 + (dz)**2)**0.5
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=dist, location=(dx/2 + p1[0], dy/2 + p1[1], dz/2 + p1[2]))
    phi = math.atan2(dy, dx)
    theta = math.acos(dz/dist)
    bpy.context.active_object.rotation_euler[1] = theta
    bpy.context.active_object.rotation_euler[2] = phi


def create_window(x, y, z, xdelta, ydelta, zdelta, qw, qx, qy, qz, xangdelta, yangdelta, zangdelta, text_path):
    # Create a new mesh for the window
    x_offset = random.uniform(-xdelta, xdelta)
    y_offset = random.uniform(-ydelta, ydelta)
    z_offset = random.uniform(-zdelta, zdelta)
    x_angle_offset = math.radians(random.uniform(-xangdelta, xangdelta))
    y_angle_offset = math.radians(random.uniform(-yangdelta, yangdelta))
    z_angle_offset = math.radians(random.uniform(-zangdelta, zangdelta))
    
    # Apply the offsets to the center and orientation
    x += x_offset
    y += y_offset
    z += z_offset
    qw, qx, qy, qz = qw + x_angle_offset, qx + y_angle_offset, qy + z_angle_offset, qz

    # Create a new mesh for the window
    bpy.ops.mesh.primitive_plane_add(size=1, enter_editmode=False, align='WORLD', location=(x, y, z))
    window = bpy.context.object

    # Rotate the plane to match the orientation
    window.rotation_quaternion = [qw, qx, qy, qz]
    window.scale.z = zdelta
    bpy.context.active_object.rotation_euler = [xangdelta, yangdelta, zangdelta]
    bpy.context.view_layer.update()
    
    if text_path:
        window_material = bpy.data.materials.new(name="Window_Material")
        window.data.materials.append(window_material)
        window.active_material.use_nodes = True
        shader_node = window_material.node_tree.nodes["Principled BSDF"]
        image_texture = window_material.node_tree.nodes.new('ShaderNodeTexImage')
        image_texture.image = bpy.data.images.load(text_path)

        window_material.node_tree.links.new(shader_node.inputs["Base Color"], image_texture.outputs["Color"])


def clear_scene():
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

clear_scene()
