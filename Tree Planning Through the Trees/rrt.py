import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random
import math
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
import bpy
import csv

EPS = 1
numNodes =5000 #5000
r = 50  # Radius to look for neighbors
map_file = "/home/ankush/Desktop/YourDirectoryID_p2a/src/sample_maps/map1.txt"

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

robot_width = 0.3
robot_length = 0.3
robot_height = 0.5
bloat_width = robot_width/2
bloat_length = robot_length/2
bloat_height = robot_height/2

with open(map_file) as file:
    obstacles = []
    lines = file.readlines()
    for line in lines:
        line_parts = line.strip().split()
        if len(line_parts) > 0:
            if line_parts[0] == "block" and len(line_parts) == 10:
                xmin, ymin, zmin, xmax, ymax, zmax, r, g, b = map(float, line_parts[1:])
                create_block(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b, robot_width = 0.3, robot_length = 0.3, robot_height = 0.5)
                obstacles.append([xmin-bloat_width, ymin-bloat_length, zmin-bloat_height, xmax+bloat_width, ymax+bloat_length, zmax+bloat_height])
            elif line_parts[0] == "boundary" and len(line_parts) == 7:
                bxmin, bymin, bzmin, bxmax, bymax, bzmax = map(float, line_parts[1:])
                transparency = 0.344  # Set the desired transparency value
                create_boundary(bxmin, bymin, bzmin, bxmax, bymax, bzmax, transparency)
                

# Initialize
q_start = {'coord': np.array([5, 18, 3]), 'cost': 0, 'parent': -1}
q_goal = {'coord': np.array([6, -2, 3]), 'cost': 0, 'parent': -1}
nodes = [q_start]
start_location = [5,18,3]
create_sphere(start_location, 255, 0, 0, radius=1)
goal_location = [6,-2,3]
create_sphere(goal_location, 0, 255, 0, radius=1)

# Setup plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlim([bxmin, bxmax])
# ax.set_ylim([bymin, bymax])
# ax.set_zlim([bzmin, bzmax])

# Draw obstacle
#for obstacle in obstacles:
#    xmin, ymin, zmin,xmax,ymax, zmax = obstacle

#    verts = [[(xmin, ymin, zmin), (xmin, ymax, zmin), (xmax, ymax, zmin), (xmax, ymin, zmin)],
#            [(xmin, ymin, zmin), (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymin, zmin)],
#            [(xmin, ymin, zmin), (xmin, ymin, zmax), (xmin, ymax, zmax), (xmin, ymax, zmin)],
#            [(xmax, ymin, zmin), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmax, ymax, zmin)],
#            [(xmin, ymax, zmin), (xmin, ymax, zmax), (xmax, ymax, zmax), (xmax, ymax, zmin)],
#            [(xmin, ymin, zmax), (xmin, ymax, zmax), (xmax, ymax, zmax), (xmax, ymin, zmax)]]


    # ax.add_collection3d(Poly3DCollection(verts, facecolors='y', linewidths=1, edgecolors='r'))

# ax.scatter(start_location[0], start_location[1], start_location[2], c='r', marker='o', s=100, label='Start')
# ax.scatter(goal_location[0], goal_location[1], goal_location[2], c='g', marker='o', s=100, label='Goal')


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
    M = 100  # Number of points to check along the path
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
#    print(i)
    q_rand = np.array([np.random.uniform(bxmin, bxmax), np.random.uniform(bymin, bymax), np.random.uniform(bzmin, bzmax)])
    # ax.scatter(*q_rand, c='b', marker='x')

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
#        print("Distance to goal:", distance_to_goal)
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
        # ax.plot(*zip(q_end['coord'], start['coord']), color='r', linewidth=4)
        # plt.draw()
        # plt.pause(0.001)
        q_end = start
    path.append(q_start['coord'])
    path_found = True  # Set the flag to True since a path is found
path.reverse()
# Check the flag after the loop
if not path_found:
    print("No collision-free path found to goal")

# plt.show()

print(path)
        
create_nodes_spheres(path)

def calculate_time_taken(path):
        avgvel = 2
        tf = 0 
        for i in range(len(path)-1):
            x0,y0,z0 = path[i][0],path[i][1],path[i][2]
            xf,yf,zf = path[i+1][0],path[i+1][1],path[i+1][2]
            tf = tf + math.ceil(dist_3d(path[i+1],path[i])/avgvel)
        return tf
time_taken = calculate_time_taken(path)


def calc_path(path, avgvel,total_time):
    
    t0 = 0
    i = 0 

    x_d = []
    y_d = []
    z_d= []
    x_d_dot = []
    y_d_dot = []
    z_d_dot = []
    x_d_ddot = []
    y_d_ddot = []
    z_d_ddot = []
    

    
    for t in range(int(time_taken)+1):        
        
            x0,y0,z0 = path[i][0],path[i][1],path[i][2]
            xf,yf,zf = path[i+1][0],path[i+1][1],path[i+1][2]

            if i == 0:
                tf = dist_3d(path[i+1],path[i])/avgvel
            else:
                tf = tf + dist_3d(path[i+1],path[i])/avgvel

            A = np.array([[1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5)],
                                    [0, 1, 2 * t0, 3 * pow(t0,2), 4 * pow(t0,3), 5 * pow(t0,4)],
                                    [0, 0, 2, 6 * t0, 12 * pow(t0,2), 20 * pow(t0,3)],
                                    [1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5)],
                                    [0, 1, 2 * tf, 3 * pow(tf,2), 4 * pow(tf,3), 5 * pow(tf,4)],
                                    [0, 0, 2, 6 * tf, 12 * pow(tf,2), 20 * pow(tf,3)]]) 
            
            if i == 0:
                B  = np.array([[x0,0,0,xf,2,0],
                    [y0,0,0,yf,2,0],
                    [z0,0,0,zf,2,0]])
                
            elif i == (len(path)-1):
                B  = np.array([[x0,2,0,xf,0,0],
                    [y0,2,0,yf,0,0],
                    [z0,2,0,zf,0,0]])
                
            else:
                B  = np.array([[x0,2,0,xf,2,0],
                    [y0,2,0,yf,2,0],
                    [z0,2,0,zf,2,0]])
            
            ax = np.linalg.solve(A,B[0])
            ay = np.linalg.solve(A,B[1])
            az = np.linalg.solve(A,B[2])

            x_d.append(ax[0] + ax[1] * t + ax[2] * pow(t,2) + ax[3] * pow(t,3) + ax[4] * pow(t,4) + ax[5] * pow(t,5))
            y_d.append(ay[0] + ay[1] * t + ay[2] * pow(t,2) + ay[3] * pow(t,3) + ay[4] * pow(t,4) + ay[5] * pow(t,5))
            z_d.append(az[0] + az[1] * t + az[2] * pow(t,2) + az[3] * pow(t,3) + az[4] * pow(t,4) + az[5] * pow(t,5))
            
            # Velocity
            x_d_dot.append(ax[1] + 2 * ax[2]*t + 3 * ax[3] * pow(t,2) + 4*ax[4] * pow(t,3) + 5 * ax[5] * pow(t,4))
            y_d_dot.append(ay[1] + 2 * ay[2]*t + 3 * ay[3] * pow(t,2) + 4*ay[4] * pow(t,3) + 5 * ay[5] * pow(t,4))
            z_d_dot.append(az[1] + 2 * az[2]*t + 3 * az[3] * pow(t,2) + 4*az[4] * pow(t,3) + 5 * az[5] * pow(t,4))
            
            # Acceleration
            x_d_ddot.append(2 * ax[2] + 6 * ax[3] * t + 12 * ax[4] * pow(t,2) + 20 * ax[5] * pow(t,3))
            y_d_ddot.append(2 * ay[2] + 6 * ay[3] * t + 12 * ay[4] * pow(t,2) + 20 * ay[5] * pow(t,3))
            z_d_ddot.append(2 * az[2] + 6 * az[3] * t + 12 * az[4] * pow(t,2) + 20 * az[5] * pow(t,3))

            if t == tf:
                i = i+1
                t0 = tf
            # writer.writerow([x_d, y_d, z_d, x_d_dot, y_d_dot, z_d_dot, x_d_ddot, y_d_ddot, z_d_ddot])
            # print(x_d,y_d,z_d,x_d_dot,y_d_dot,z_d_dot,x_d_ddot,y_d_ddot,z_d_ddot)

    with open("/home/ankush/Desktop/your_file.csv", mode="w", newline="") as file:
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





calc_path(path, 2, time_taken)
