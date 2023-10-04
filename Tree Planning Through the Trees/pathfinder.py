import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import random
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.interpolate import CubicSpline
# import bpy

map_file = "/home/anuj/Desktop/AerialRobotics/apairaikar_p2a/src/sample_maps/map4.txt"

EPS = 1
numNodes =5000 #5000
r = 50  # Radius to look for neighbors
map_file = "/home/ankush/Desktop/YourDirectoryID_p2a/src/sample_maps/map4.txt"

def create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b):

    bpy.ops.mesh.primitive_cube_add(scale=(xmax-xmin, ymax-ymin, zmax-zmin))
    block = bpy.context.active_object
    block.location.x = (xmax+xmin) / 2
    block.location.y = (ymax+ymin) / 2
    block.location.z = (zmax+zmin) / 2
    material = bpy.data.materials.new(name="BlockMaterial")
    block.data.materials.append(material)
    material.diffuse_color = (r / 255, g / 255, b / 255, 1)

def create_sphere(location, r, g, b):

    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0, location=location)
    sphere = bpy.context.active_object
    material = bpy.data.materials.new(name="SphereMaterial")
    sphere.data.materials.append(material)
    material.diffuse_color = (r / 255, g / 255, b / 255, 1)
    
def create_boundary(xmin, ymin, zmin, xmax, ymax, zmax, transparency):

    bpy.ops.mesh.primitive_cube_add(scale=(xmax-xmin, ymax-ymin, zmax-zmin))
    boundary = bpy.context.active_object
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

def clear_scene():

    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

clear_scene()

with open(map_file) as file:

    obstacles = []
    lines = file.readlines()

    for line in lines:

        line_parts = line.strip().split()
        if len(line_parts) > 0:

            if line_parts[0] == "block" and len(line_parts) == 10:

                xmin, ymin, zmin, xmax, ymax, zmax, r, g, b = map(float, line_parts[1:])
                create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b)
                obstacles.append([xmin, ymin, zmin, xmax, ymax, zmax])

            elif line_parts[0] == "boundary" and len(line_parts) == 7:

                bxmin, bymin, bzmin, bxmax, bymax, bzmax = map(float, line_parts[1:])
                xmin, ymin, zmin, xmax, ymax, zmax = map(float, line_parts[1:])
                transparency = 0.344  # Set the desired transparency value
                create_boundary(xmin, ymin, zmin, xmax, ymax, zmax, transparency)

# Initialize
q_start = {'coord': np.array([5, 16, 3]), 'cost': 0, 'parent': -1}
q_goal = {'coord': np.array([24, 15, 5]), 'cost': 0, 'parent': -1}
nodes = [q_start]
start_location = [5,16,3]
create_sphere(start_location, 255, 0, 0)
goal_location = [24,15,5]
create_sphere(goal_location, 0, 255, 0)

# Setup plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([bxmin, bxmax])
ax.set_ylim([bymin, bymax])
ax.set_zlim([bzmin, bzmax])

# Draw obstacle
for obstacle in obstacles:
    xmin, ymin, zmin,xmax,ymax, zmax = obstacle

    verts = [[(xmin, ymin, zmin), (xmin, ymax, zmin), (xmax, ymax, zmin), (xmax, ymin, zmin)],
            [(xmin, ymin, zmin), (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymin, zmin)],
            [(xmin, ymin, zmin), (xmin, ymin, zmax), (xmin, ymax, zmax), (xmin, ymax, zmin)],
            [(xmax, ymin, zmin), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmax, ymax, zmin)],
            [(xmin, ymax, zmin), (xmin, ymax, zmax), (xmax, ymax, zmax), (xmax, ymax, zmin)],
            [(xmin, ymin, zmax), (xmin, ymax, zmax), (xmax, ymax, zmax), (xmax, ymin, zmax)]]


    ax.add_collection3d(Poly3DCollection(verts, facecolors='y', linewidths=1, edgecolors='r'))

ax.scatter(start_location[0], start_location[1], start_location[2], c='r', marker='o', s=100, label='Start')
ax.scatter(goal_location[0], goal_location[1], goal_location[2], c='g', marker='o', s=100, label='Goal')


def dist_3d(q1, q2):
    return np.linalg.norm(q1 - q2)


def in_obstacle(point_coord, obs):
    x, y, z = point_coord
    for o in obs:
        xmin, ymin, zmin, xmax, ymax, zmax = o
        if xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax:
            return True
    return False


def no_collision(n2, n1, o):
    M = 20  # Number of points to check along the path
    for i in np.linspace(0, 1, M):
        point_coord = n2 * (1 - i) + n1 * i
        if in_obstacle(point_coord, o):
            return False
    return True



def steer(qr, qn, val, eps):
    if val >= eps:
        return qn + ((qr - qn) * eps) / dist_3d(qr, qn)
    else:
        return qr


def rewire(new_node, nodes, radius, obstacles):
    new_idx = len(nodes) - 1  # index of the new_node as it is appended last
    for idx, node in enumerate(nodes):
        if idx == new_idx:  # skip the comparison between new_node and itself
            continue
        if dist_3d(new_node['coord'], node['coord']) < radius:
            potential_cost = new_node['cost'] + dist_3d(new_node['coord'], node['coord'])
            if potential_cost < node['cost'] and no_collision(new_node['coord'], node['coord'], obstacles):
                # Update Parent
                node['parent'] = new_idx  # the index of new_node
                # Update Cost
                node['cost'] = potential_cost
                

for i in range(numNodes):
    print(i)
    q_rand = np.array([np.random.uniform(bxmin, bxmax), np.random.uniform(bymin, bymax), np.random.uniform(bzmin, bzmax)])
    ax.scatter(*q_rand, c='b', marker='x')

    ndist = [dist_3d(node['coord'], q_rand) for node in nodes]
    idx = np.argmin(ndist)
    q_near = nodes[idx]

    q_new_coord = steer(q_rand, q_near['coord'], ndist[idx], EPS)
    if no_collision(q_new_coord, q_near['coord'], obstacles):
        # ax.plot(*zip(q_near['coord'], q_new_coord), color='k')

        q_new = {'coord': q_new_coord, 'cost': dist_3d(q_new_coord, q_near['coord']) + q_near['cost'], 'parent': idx}
        nodes.append(q_new)
        rewire(q_new, nodes, r, obstacles)
        distance_to_goal = dist_3d(q_new_coord, q_goal['coord'])
        print("Distance to goal:", distance_to_goal)
        if distance_to_goal <= EPS:
            print("Path found to goal")
            if no_collision(q_new_coord, q_goal['coord'], obstacles):
                nodes.append({'coord': q_goal['coord'], 'cost': dist_3d(q_new_coord, q_goal['coord']) + q_new['cost'], 'parent': len(nodes) - 1})
            break

# Find path
path_found = False
path =[]
idx_q_end = np.argmin([dist_3d(node['coord'], q_goal['coord']) for node in nodes])
q_end = nodes[idx_q_end]
if no_collision(q_end['coord'], q_goal['coord'], obstacles):
    nodes.append({'coord': q_goal['coord'], 'cost': 0, 'parent': idx_q_end})
    while q_end['parent'] != -1:
        path.append(q_end['coord'])
        start = nodes[q_end['parent']]
        ax.plot(*zip(q_end['coord'], start['coord']), color='r', linewidth=4)
        plt.draw()
        plt.pause(0.001)
        q_end = start
    path.append(q_start['coord'])
    path_found = True  # Set the flag to True since a path is found
path.reverse()
# Check the flag after the loop
if not path_found:
    print("No collision-free path found to goal")

plt.show()

print(path)