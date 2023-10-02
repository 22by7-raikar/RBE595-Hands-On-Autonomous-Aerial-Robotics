import random
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.interpolate import CubicSpline
# import bpy

# map_file = "/home/ankush/Desktop/YourDirectoryID_p2a/src/sample_maps/map4.txt"

# Define constants
bxmin = 0
bymin = 0
bzmin = 0
bxmax = 600
bymax = 600
bzmax = 600

# def create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b):

#     bpy.ops.mesh.primitive_cube_add(scale=(xmax-xmin, ymax-ymin, zmax-zmin))
#     block = bpy.context.active_object
#     block.location.x = (xmax+xmin) / 2
#     block.location.y = (ymax+ymin) / 2
#     block.location.z = (zmax+zmin) / 2
#     material = bpy.data.materials.new(name="BlockMaterial")
#     block.data.materials.append(material)
#     material.diffuse_color = (r / 255, g / 255, b / 255, 1)

# def create_sphere(location, r, g, b):

#     bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0, location=location)
#     sphere = bpy.context.active_object
#     material = bpy.data.materials.new(name="SphereMaterial")
#     sphere.data.materials.append(material)
#     material.diffuse_color = (r / 255, g / 255, b / 255, 1)

# def create_boundary(xmin, ymin, zmin, xmax, ymax, zmax, transparency):

#     bpy.ops.mesh.primitive_cube_add(scale=(xmax-xmin, ymax-ymin, zmax-zmin))
#     boundary = bpy.context.active_object
#     boundary.location.x = (xmax+xmin) / 2
#     boundary.location.y = (ymax+ymin) / 2
#     boundary.location.z = (zmax+zmin) / 2
#     material = bpy.data.materials.new(name="BoundaryMaterial")
#     boundary.data.materials.append(material)
#     material.diffuse_color = (0, 0, 0, 1)
#     material.use_nodes = True
#     principled_bsdf = material.node_tree.nodes.get('Principled BSDF')
#     if principled_bsdf:
#         principled_bsdf.inputs['Alpha'].default_value = 1 - transparency

# def clear_scene():

#     bpy.ops.object.select_all(action='DESELECT')
#     bpy.ops.object.select_by_type(type='MESH')
#     bpy.ops.object.delete()

# clear_scene()

# with open(map_file) as file:

#     obstacles = []
#     lines = file.readlines()

#     for line in lines:

#         line_parts = line.strip().split()
#         if len(line_parts) > 0:

#             if line_parts[0] == "block" and len(line_parts) == 10:

#                 xmin, ymin, zmin, xmax, ymax, zmax, r, g, b = map(float, line_parts[1:])
#                 create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b)
#                 obstacles.append([xmin, ymin, zmin, xmax, ymax, zmax])

#             elif line_parts[0] == "boundary" and len(line_parts) == 7:

#                 bxmin, bymin, bzmin, bxmax, bymax, bzmax = map(float, line_parts[1:])
#                 xmin, ymin, zmin, xmax, ymax, zmax = map(float, line_parts[1:])
#                 transparency = 0.344  # Set the desired transparency value
#                 create_boundary(xmin, ymin, zmin, xmax, ymax, zmax, transparency)


epsilon = 20

numNodes = 2000

class Node:
    def __init__(self, coord, cost=0, parent=None):
        self.coord = coord
        self.cost = cost
        self.parent = parent


start_location = [5, 16, 3]
goal_location = [500, 500, 500]
q_start = Node(start_location)
q_goal = Node(goal_location)
nodes = [q_start]
        
# create_sphere(start_location, 255, 0, 0)
# create_sphere(goal_location, 0, 255, 0)

# Define a function to compute quintic spline coefficients for a segment

def dist_3d(q1, q2):
    return math.sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2 + (q1[2] - q2[2])**2)

def steer(qr, qn, val, eps):
    if val >= eps:
        qnew = [
            qn[0] + ((qr[0] - qn[0]) * eps) / dist_3d(qr, qn),
            qn[1] + ((qr[1] - qn[1]) * eps) / dist_3d(qr, qn),
            qn[2] + ((qr[2] - qn[2]) * eps) / dist_3d(qr, qn)
        ]
        
    else:
        qnew = qr
    return qnew

def is_line_collision_with_obstacle(start_point, end_point, obstacles = None):
    # Extract obstacle coordinates
    # for obstacle in obstacles:

    print(start_point, end_point)

    min_x, max_x, min_y, max_y, min_z, max_z = 250, 450, 250, 450, 250, 450
# bymin = 0
# bzmin = 0
# bxmax = 
# bymax = 480
# bzmax = 400

    # Check if the line is outside the bounding box of the obstacle
    if (start_point[0] < min_x and end_point[0] < min_x) or \
    (start_point[0] > max_x and end_point[0] > max_x) or \
    (start_point[1] < min_y and end_point[1] < min_y) or \
    (start_point[1] > max_y and end_point[1] > max_y) or \
    (start_point[2] < min_z and end_point[2] < min_z) or \
    (start_point[2] > max_z and end_point[2] > max_z):
        # The line is completely outside the bounding box
        return False

    # Check if the line is completely inside the bounding box
    if (min_x <= start_point[0] <= max_x and
        min_x <= end_point[0] <= max_x and
        min_y <= start_point[1] <= max_y and
        min_y <= end_point[1] <= max_y and
        min_z <= start_point[2] <= max_z and
        min_z <= end_point[2] <= max_z):
        # The line is completely inside the bounding box
        return True

    # Check if the line intersects the bounding box faces
    # along the X-axis
    if (min_x <= start_point[0] <= max_x or
        min_x <= end_point[0] <= max_x):
        return True

    # along the Y-axis
    if (min_y <= start_point[1] <= max_y or
        min_y <= end_point[1] <= max_y):
        return True

    # along the Z-axis
    if (min_z <= start_point[2] <= max_z or
        min_z <= end_point[2] <= max_z):
        return True

    # The line does not intersect any of the bounding box faces
    return False

def interpolate_quintic_spline(coefficients, t_values):
    # Initialize arrays to store interpolated values
    x_values = np.zeros_like(t_values)
    y_values = np.zeros_like(t_values)
    z_values = np.zeros_like(t_values)

    # Interpolate for each time step
    for i, t in enumerate(t_values):
        x_values[i] = np.dot(coefficients[0], [1, t, t**2, t**3, t**4, t**5])
        y_values[i] = np.dot(coefficients[1], [1, t, t**2, t**3, t**4, t**5])
        z_values[i] = np.dot(coefficients[2], [1, t, t**2, t**3, t**4, t**5])

    return x_values, y_values, z_values


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
obstacle = [250, 250, 250, 450, 450, 450]
ax.bar3d(obstacle[0], obstacle[2], obstacle[1], obstacle[3] - obstacle[0], obstacle[4] - obstacle[2], obstacle[5] - obstacle[1], shade=True, color=(0.5, 0.5, 0.5,0.5))

for i in range(1, numNodes + 1):
    q_rand = [random.uniform(bxmin, bxmax), random.uniform(bymin, bymax), random.uniform(bzmin, bzmax)]
    # ax.scatter(q_rand[0], q_rand[1], q_rand[2], c='b', marker='x')

    for j, node in enumerate(nodes):
        if node.coord == q_goal.coord:
            break

    ndist = [dist_3d(node.coord, q_rand) for node in nodes]
    val, idx = min((val, idx) for (idx, val) in enumerate(ndist))
    q_near = nodes[idx]

    q_new_coord = steer(q_rand, q_near.coord, val, epsilon)
    ax.plot([q_near.coord[0], q_new_coord[0]], [q_near.coord[1], q_new_coord[1]], [q_near.coord[2], q_new_coord[2]], 'k', linewidth=2)

    if is_line_collision_with_obstacle(q_near.coord, q_rand, obstacles = None) == False:
        q_new = Node(q_new_coord)
        q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost

        q_nearest = [node for node in nodes if dist_3d(node.coord, q_new.coord) <= 5 and is_line_collision_with_obstacle(q_new.coord, node.coord, obstacles= None)==False]

        q_min = q_near
        C_min = q_new.cost

    for q in q_nearest:
        if q.cost + dist_3d(q.coord, q_new.coord) < C_min and is_line_collision_with_obstacle(q_new.coord, q.coord, obstacles = None)==False:
            q_min = q
            C_min = q.cost + dist_3d(q.coord, q_new.coord)
            ax.plot([q_min.coord[0], q_new.coord[0]], [q_min.coord[1], q_new.coord[1]], [q_min.coord[2], q_new.coord[2]], 'g')

    for j, node in enumerate(nodes):
        if node.coord == q_min.coord:
            q_new.parent = j

    nodes.append(q_new)

D = [dist_3d(node.coord, q_goal.coord) for node in nodes]

val, idx = min((val, idx) for (idx, val) in enumerate(D))
q_final = nodes[idx]
q_goal.parent = idx
q_end = q_goal
nodes.append(q_goal)

while q_end.parent is not None:
    start = q_end.parent
    ax.plot([q_end.coord[0], nodes[start].coord[0]], [q_end.coord[1], nodes[start].coord[1]], [q_end.coord[2], nodes[start].coord[2]], 'r', linewidth=4)
    q_end = nodes[start]

plt.show()

def compute_quintic_spline_coefficients(p0, p1, t0, t1):
    dt = t1 - t0
    a0 = p0
    a1 = 0
    a2 = 0
    a3 = (2 * p1 - 2 * p0) / dt**3
    a4 = (-3 * p1 + 3 * p0) / dt**4
    a5 = (p1 - p0) / dt**5
    return [a0, a1, a2, a3, a4, a5]


waypoints = []
times = [0.0]  # Start at t=0

for node in nodes:
    waypoints.append(node.coord)
    times.append(times[-1] + 1.0)


spline_coefficients = []

for i in range(len(waypoints) - 1):
    
    p0 = waypoints[i]
    p1 = waypoints[i + 1]

    t0 = times[i]
    t1 = times[i + 1]
    
    # Convert p0 and p1 to lists
    p0_list = [p0[0], p0[1], p0[2]]
    p1_list = [p1[0], p1[1], p1[2]]

    coefficients_x = compute_quintic_spline_coefficients(p0_list[0], p1_list[0], t0, t1)
    coefficients_y = compute_quintic_spline_coefficients(p0_list[1], p1_list[1], t0, t1)
    coefficients_z = compute_quintic_spline_coefficients(p0_list[2], p1_list[2], t0, t1)
    
    spline_coefficients.append([coefficients_x, coefficients_y, coefficients_z])


# Generate the quintic spline trajectory
t_step = 0.01  # Time step for evaluation
spline_trajectory = []

for i in range(len(waypoints) - 1):
    
    t = np.arange(times[i], times[i + 1], t_step)
    coefficients = spline_coefficients[i]
    
    x_traj, y_traj, z_traj = interpolate_quintic_spline(coefficients, t)
    spline_trajectory.append(np.column_stack((x_traj, y_traj, z_traj)))

spline_trajectory = np.vstack(spline_trajectory)

x_traj = spline_trajectory[:, 0]
y_traj = spline_trajectory[:, 1]
z_traj = spline_trajectory[:, 2]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the quintic spline trajectory
ax.plot(x_traj, y_traj, z_traj, label='Quintic Spline Trajectory', linewidth=2)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.legend()
plt.show()
