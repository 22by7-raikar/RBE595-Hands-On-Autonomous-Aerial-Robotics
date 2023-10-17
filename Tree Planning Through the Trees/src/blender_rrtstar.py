import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random
import math
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
import bpy
import csv

d_goal = 0.5
numNodes = 10000 
r = 0.2
map_file = "/home/anuj/Desktop/sbachimanchi_p2a/TestSetP2b/maps/map1.txt"

robot_width = 0.5
robot_length = 0.5
robot_height = 0.5
bloat_width = robot_width/2
bloat_length = robot_length/2
bloat_height = robot_height/2

def create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b, robot_width, robot_length, robot_height):
    bloat_width = robot_width/2
    bloat_length = robot_length/2
    bloat_height = robot_height/2
    bpy.ops.mesh.primitive_cube_add(scale=((xmax-xmin)/2 + (bloat_width), (ymax-ymin)/2 + (bloat_length), (zmax-zmin)/2 + (bloat_height)))
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


def create_nodes_spheres(path_found):
    for i in range(1, len(path)):
        create_sphere(path[i], 0, 0, 255, radius=0.1)
        create_cylinder(path[i], path[i -1])
        
        
def clear_scene():
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()


clear_scene()


with open(map_file) as file:
    obstacles = []
    obs=[]
    lines = file.readlines()

    for line in lines:
        line_parts = line.strip().split()
    
        if len(line_parts) > 0:
    
            if line_parts[0] == "block" and len(line_parts) == 10:
                xmin, ymin, zmin, xmax, ymax, zmax, r, g, b = map(float, line_parts[1:])
                create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b, robot_width = 0.3, robot_length = 0.3, robot_height = 0.5)

                obstacles.append([xmin-bloat_width, ymin-bloat_length, zmin-bloat_height, xmax+bloat_width, ymax+bloat_length, zmax+bloat_height])
                obs.append([xmin, ymin, zmin, xmax, ymax, zmax])
    
            elif line_parts[0] == "boundary" and len(line_parts) == 7:
                bxmin, bymin, bzmin, bxmax, bymax, bzmax = map(float, line_parts[1:])
                transparency = 0.344  
                create_boundary(bxmin, bymin, bzmin, bxmax, bymax, bzmax, transparency)


# Initialize
start_location = [0, 0, 0.6]
goal_location = [-0.59, 6.81, 0]
q_start = {'coord': np.array(start_location), 'cost': 0, 'parent': -1}
q_goal = {'coord': np.array(goal_location), 'cost': 0, 'parent': -1}

nodes = [q_start]
create_sphere(start_location, 255, 0, 0, radius=1)
create_sphere(goal_location, 0, 255, 0, radius=1)

# Setup plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([bxmin, bxmax])
ax.set_ylim([bymin, bymax])
ax.set_zlim([bzmin, bzmax])

# Draw obstacle
for obstacle in obs:
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
        
        if (xmin <= x) and (x <= xmax) and (ymin <= y) and (y <= ymax) and (zmin <= z) and (z <= zmax):
            return True
        
    return False


def no_collision(n2, n1, o):
    M = 100    
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
    new_idx = len(nodes) - 1

    for idx, node in enumerate(nodes):

        if idx == new_idx:
            continue

        if dist_3d(new_node['coord'], node['coord']) < radius:
            potential_cost = new_node['cost'] + dist_3d(new_node['coord'], node['coord'])

            if potential_cost < node['cost'] and no_collision(new_node['coord'], node['coord'], obstacles):
                node['parent'] = new_idx 
                node['cost'] = potential_cost 
                

for i in range(numNodes):
    q_rand = np.array([np.random.uniform(bxmin, bxmax), np.random.uniform(bymin, bymax), np.random.uniform(bzmin, bzmax)])
    ndist = [dist_3d(node['coord'], q_rand) for node in nodes]
    idx = np.argmin(ndist)
    q_near = nodes[idx]

    q_new_coord = steer(q_rand, q_near['coord'], ndist[idx], d_goal)

    if no_collision(q_new_coord, q_near['coord'], obstacles):
        q_new = {'coord': q_new_coord, 'cost': dist_3d(q_new_coord, q_near['coord']) + q_near['cost'], 'parent': idx}
        nodes.append(q_new)
        
        rewire(q_new, nodes, r, obstacles)
        distance_to_goal = dist_3d(q_new_coord, q_goal['coord'])

        if distance_to_goal <= d_goal:
            print("Path found to goal")
            if no_collision(q_new_coord, q_goal['coord'], obstacles):
                nodes.append({'coord': q_goal['coord'], 'cost': dist_3d(q_new_coord, q_goal['coord']) + q_new['cost'], 'parent': len(nodes) - 1})
            
            break

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
    path_found = True 

path.reverse()

if not path_found:
    print("No collision-free path found to goal")

def point_to_line_distance(point, line_start, line_end):
    line_start = np.array(line_start)
    line_end = np.array(line_end)
    point = np.array(point)

    if np.all(line_start == line_end):
        return np.linalg.norm(point - line_start)

    line_vec = line_end - line_start
    point_vec = point - line_start
    line_length = np.linalg.norm(line_vec)
    projection = np.dot(point_vec, line_vec / line_length)

    if projection <= 0:
        return np.linalg.norm(point - line_start)

    if projection >= line_length:
        return np.linalg.norm(point - line_end)

    perpendicular_vec = point_vec - projection * (line_vec / line_length)
    return np.linalg.norm(perpendicular_vec)

def prune_path(path, obstacles):
    pruned_path = [path[0]]  # Initialize with the start point
    i = 0

    while i < len(path) - 1:
        j = i + 2
        while j < len(path):
            if not any(in_obstacle(point, obstacles) for point in path[i + 1:j]):
                pruned_path.append(path[j - 1])
                i = j - 1
                break
            j += 1
        if j == len(path):
            pruned_path.append(path[i + 1])
            i += 1

    if not np.array_equal(pruned_path[-1], path[-1]):
        pruned_path.append(path[-1])

    return np.array(pruned_path)


def calc_path(path_taken, avgvel, total_time):
    
    t0 = 0
    i = 0 
    x_d, y_d, z_d = [], [], []
    x_d_dot, y_d_dot, z_d_dot = [], [], []
    x_d_ddot, y_d_ddot, z_d_ddot = [], [], []
    spline_path = []
    

    time_array = np.arange(0, total_time, 1)   

    for t in time_array:       
        
        x0,y0,z0 = path_taken[i][0],path_taken[i][1],path_taken[i][2]
        xf,yf,zf = path_taken[i+1][0],path_taken[i+1][1],path_taken[i+1][2]
        # print ("this is i ", i,"this is the initial point:", x0, y0,z0,"this is the final point:", xf, yf,zf)

        if i == 0:
            tf = math.ceil(dist_3d(path_taken[i+1], path_taken[i])/avgvel)
    

        A = np.array([[1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5)],
                                [0, 1, 2 * t0, 3 * pow(t0,2), 4 * pow(t0,3), 5 * pow(t0,4)],
                                [0, 0, 2, 6 * t0, 12 * pow(t0,2), 20 * pow(t0,3)],
                                [1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5)],
                                [0, 1, 2 * tf, 3 * pow(tf,2), 4 * pow(tf,3), 5 * pow(tf,4)],
                                [0, 0, 2, 6 * tf, 12 * pow(tf,2), 20 * pow(tf,3)]]) 
        
        B  = np.array([[x0,0,0,xf,0,0],
            [y0,0,0,yf,0,0],
            [z0,0,0,zf,0,0]])
            
        ax = np.linalg.solve(A,B[0])
        ay = np.linalg.solve(A,B[1])
        az = np.linalg.solve(A,B[2])
    
        x_new = ax[0] + ax[1] * t + ax[2] * pow(t, 2) + ax[3] * pow(t, 3) + ax[4] * pow(t, 4) + ax[5] * pow(t, 5)
        y_new = ay[0] + ay[1] * t + ay[2] * pow(t, 2) + ay[3] * pow(t, 3) + ay[4] * pow(t, 4) + ay[5] * pow(t, 5)
        z_new = az[0] + az[1] * t + az[2] * pow(t, 2) + az[3] * pow(t, 3) + az[4] * pow(t, 4) + az[5] * pow(t, 5)

        x_new = np.clip(x_new, bxmin, bxmax)
        y_new = np.clip(y_new, bymin, bymax)
        z_new = np.clip(z_new, bzmin, bzmax)

        x_d.append(x_new)
        y_d.append(y_new)
        z_d.append(z_new)
        
        # Velocity
        x_d_dot.append(ax[1] + 2 * ax[2]*t + 3 * ax[3] * pow(t, 2) + 4*ax[4] * pow(t, 3) + 5 * ax[5] * pow(t, 4))
        y_d_dot.append(ay[1] + 2 * ay[2]*t + 3 * ay[3] * pow(t, 2) + 4*ay[4] * pow(t, 3) + 5 * ay[5] * pow(t, 4))
        z_d_dot.append(az[1] + 2 * az[2]*t + 3 * az[3] * pow(t, 2) + 4*az[4] * pow(t, 3) + 5 * az[5] * pow(t, 4))
        
        # Acceleration
        x_d_ddot.append(2 * ax[2] + 6 * ax[3] * t + 12 * ax[4] * pow(t, 2) + 20 * ax[5] * pow(t, 3))
        y_d_ddot.append(2 * ay[2] + 6 * ay[3] * t + 12 * ay[4] * pow(t, 2) + 20 * ay[5] * pow(t, 3))
        z_d_ddot.append(2 * az[2] + 6 * az[3] * t + 12 * az[4] * pow(t, 2) + 20 * az[5] * pow(t, 3))

        if t == tf:
            i = i+1
            t0 = tf
            tf = tf + math.ceil(dist_3d(path_taken[i+1], path_taken[i])/avgvel)

    with open("/home/anuj/Desktop/sbachimanchi_p2a/TestSetP2b/maps/final_traj.csv", mode="w", newline="") as file:
        writer = csv.writer(file)

        # Writing each list as a row
        writer.writerow(x_d)
        writer.writerow(y_d)
        writer.writerow(z_d)
        writer.writerow(x_d_dot)
        writer.writerow(y_d_dot)
        writer.writerow(z_d_dot)
        writer.writerow(x_d_ddot)
        writer.writerow(y_d_ddot)
        writer.writerow(z_d_ddot)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for obstacle in obs:
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

    ax.plot(x_d, y_d, z_d, label='parametric curve')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

    return spline_path
   
def calculate_time_taken(path_taken, avgvel):
    tf = 0 
     
    for i in range(len(path_taken)-1):
        tf = tf + math.ceil(dist_3d(path_taken[i+1],path_taken[i])/avgvel)

    return tf

avgvel = 0.2
pruned_path = prune_path(path,obstacles)
time_taken = calculate_time_taken(pruned_path, avgvel)
final_path = calc_path(pruned_path, avgvel, time_taken)
