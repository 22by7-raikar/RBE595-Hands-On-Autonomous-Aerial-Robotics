import bpy
from mathutils import Vector

def visualize_path(nodes, sphere_radius=0.1, cylinder_radius=0.05):
    # Deselect all objects in the scene
    bpy.ops.object.select_all(action='DESELECT')

    # Clear existing mesh objects
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

    # creating spheres for nodes
    spheres = []
    for i, node in enumerate(nodes):
        bpy.ops.mesh.primitive_uv_sphere_add(radius=sphere_radius, location=node)
        sphere = bpy.context.object
        sphere.name = f"Node_{i}"
        spheres.append(sphere)

    # creating cylinders to connect adjacent nodes
    for i in range(len(spheres) - 1):
        node1 = spheres[i]
        node2 = spheres[i + 1]
        midpoint = ((node1.location.x + node2.location.x) / 2, 
                    (node1.location.y + node2.location.y) / 2, 
                    (node1.location.z + node2.location.z) / 2)
        direction = node2.location - node1.location
        distance = direction.length # .length calculates magnitude of vector

        bpy.ops.mesh.primitive_cylinder_add(radius=cylinder_radius, depth=distance, location=midpoint)
        cylinder = bpy.context.object

        rot_quat = direction.to_track_quat('Z', 'Y')
        cylinder.rotation_euler = rot_quat.to_euler()
        cylinder.name = f"PathEdge_{i}"

#nodes = [(5, 16, 3), (10, 26, 13)]

#visualize_path(nodes)
