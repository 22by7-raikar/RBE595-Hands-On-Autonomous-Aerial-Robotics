import bpy
import heapq
import numpy as np
import math
import random
import matplotlib.pyplot as plt

class Node:
    def __init__(self, row, col, tub):

        self.row = row        # x - coordinate
        self.col = col        # y - coordinate
        self.tub = tub        # z - coordinate
        self.cost = 0.0       # cost
        self.distance = 0

        #for using heapqueue
    def __lt__(self, other):
        return self.distance < other.distance

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



start_location = (5, 16, 3)
create_sphere(start_location, 255, 0, 0)
start = Node(start_location[0], start_location[1], start_location[2])

goal_location = (24, 15, 5)
create_sphere(goal_location, 0, 255, 0)
goal = Node(goal_location[0], goal_location[1], goal_location[2])

from mathutils import Vector

def visualize_path(vertices, sphere_radius=0.1, cylinder_radius=0.05):
    
    # Deselect all objects in the scene
    bpy.ops.object.select_all(action='DESELECT')

    # Clear existing mesh objects
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

    # creating spheres for nodes
    spheres = []
    for i, node in enumerate(vertices):
        node_curr = (node.row,node.col,node.tub)
        bpy.ops.mesh.primitive_uv_sphere_add(radius=sphere_radius, location=node_curr)
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
        
    print("Visualization Completed")

#nodes = [(5, 16, 3), (10, 26, 13)]

#visualize_path(nodes)

def dis(point1, point2):
    return math.sqrt(((point1.row - point2.row)**2) + ((point1.col - point2.col)**2) + ((point1.tub - point2.tub)**2)) 


def get_new_random_point(xmin, xmax, ymin, ymax, zmin, zmax, goal_bias):
    
    xnew = np.random.randint(xmin, xmax)
    ynew = np.random.randint(ymin, ymax)
    znew = np.random.randint(zmin, zmax)

    if np.random.uniform(0,1) > goal_bias:
        return (xnew, ynew, znew)
    
    else:
        return goal_location
        

def get_nearest_node(vertices, point):

    distance = []

    for i in vertices:
        #finding the distance 
        d = dis(point, i) 
        #pushing the node in the queue based on the distance
        heapq.heappush(distance, (d,i))
    
    #popping the nearest node from the heap 
    _, near_node = heapq.heappop(distance)

    return near_node


def get_direction(point, nn):

    diff_x = point.row - nn.row
    diff_y = point.col - nn.col
    diff_z = point.tub - nn.tub

    diff_xy = math.sqrt((diff_x**2) + (diff_y**2))
    ang1 = math.atan2(diff_z, diff_xy)
    ang2 = math.atan2(diff_y, diff_x)
    
    return ang1,ang2


def generate_directed_point(negh,ang1,ang2,step_size):

    negh.row += step_size * math.cos(ang1) * math.cos(ang2) 
    negh.col += step_size * math.cos(ang1) * math.sin(ang2) 
    negh.tub += step_size * math.sin(ang1)
    
    return negh


def check_valid(point):

    if point.row >= bxmax:
        point.row = bxmax-1

    if point.row <= bxmin:
        point.row = bxmin+1

    if point.col >= bymax:
        point.col = bymax-1

    if point.col <= bymin:
        point.col= bymin+1

    if point.tub >= bzmax:
        point.tub = bzmax-1
    
    if point.tub <= bzmin:
        point.tub = bzmin+1

    return point


def check_point_obs(obstacles, point):

    for j in obstacles:
        if (j[0] < point[0] < j[3]) and (j[1] < point[1] < j[4]) and (j[2] < point[2] < j[5]):
            return True
        
    return False


def check_collision(obstacles, nn, new_node):
    
    check = 300
    #finding out the step increment
    incx = (nn.row - new_node.row)/check
    incy = (nn.col - new_node.col)/check
    incz = (nn.tub - new_node.tub)/check

    #initializing the x and y points
    xpt = nn.row
    ypt = nn.col
    zpt = nn.tub

    for i in range(check):
        #check if the point is an obstacle

        if  check_point_obs(obstacles, (xpt, ypt, zpt)):
            return False
                
        #incrementing x and y 
        xpt = xpt+incx
        ypt = ypt+incy
        zpt = zpt+incz
    
    return True


def get_neighbors(newpt, neighbor_size):
    negh = [i for i in vertices if dis(i, newpt) < neighbor_size]        
    return negh


def rewire(new_node, neighbors, obstacles):

    neighbor_heap = [(n.cost + dis(n, new_node), n) for n in neighbors]
    heapq.heapify(neighbor_heap)

    # Finding the neighbor with the minimum cost
    while True: 

        min_cost, min_cost_neighbor = heapq.heappop(neighbor_heap)

        #check for the collision of the node
        if check_collision(obstacles, min_cost_neighbor, new_node):

            new_node.cost = min_cost
            new_node.parent = min_cost_neighbor 

        break

    # Rewire the remaining neighbors
    while neighbor_heap:
        _, neighbor = heapq.heappop(neighbor_heap)
        potential_cost = new_node.cost + dis(new_node, neighbor) 
        
        if neighbor.cost > potential_cost and check_collision(neighbor, new_node):

            neighbor.parent = new_node
            neighbor.cost = potential_cost

    return neighbor_heap


start_cost = 0
ext_step = 15
goal_bias = 0.1    #0.075
step_size = 1
found = False
neighbor_size = 20 #10
max_dist = 0.5     #1

vertices = []
vertices.append(start)

for i in range(1000):
    # print("Im on the ith iteration", i )
    
    new_point = get_new_random_point(bxmin, bxmax, bymin, bymax, bzmin, bzmax, goal_bias)
    new_point = Node(new_point[0], new_point[1], new_point[2])

    near_vertex = get_nearest_node(vertices, new_point)
    
    if dis(near_vertex,new_point)<= max_dist and new_point.row != goal.row and new_point.col != goal.col and new_point.tub != goal.tub:
        newpoint = new_point

    else:
        dir1, dir2 = get_direction(new_point,near_vertex)
        newpoint = generate_directed_point(near_vertex, dir1, dir2, step_size) 
    
    newpoint = check_valid(newpoint)

    #dissn = dis(newpoint, near_vertex)

    if check_collision(obstacles, near_vertex, newpoint):
        new_parent = near_vertex
        new_cost = near_vertex.cost + dis(newpoint, near_vertex)

        neighbors = get_neighbors(newpoint, neighbor_size)

        if len(neighbors) == 0:
            continue

        for n in neighbors:
            if check_collision(obstacles,newpoint,n) and n.cost+dis(n,newpoint) < new_cost:
                new_parent = n
                new_cost = n.cost+dis(n,newpoint)
        
        newpoint.parent = new_parent
        newpoint.cost = new_cost
        
        rewire(newpoint, neighbors, obstacles)
        vertices.append(newpoint)
        print("Im getting appended:",newpoint.row,newpoint.col,newpoint.tub)

    else:
        continue
    
    dgoal = dis(newpoint, goal)
  
    if dgoal < 10:
        found = True
        goal.parent = newpoint
        disg = dis(near_vertex, newpoint)
        goal.cost = newpoint.cost + disg
        new_neighbors = get_neighbors(goal, neighbor_size)

        if len(new_neighbors) == 0:
            print("len of negh is 0 - 2")
            continue

        rewire(goal, new_neighbors, obstacles)
        vertices.append(goal)

    else:
        print("Still finding")
        continue
    
    
if found:
        
        steps = len(vertices) - 2
        length = goal.cost
        print("It took %d nodes to find the current path" %steps)
        print("The path length is %.2f" %length)
        visualize_path(vertices)

        nodes = []
        for v in vertices:
            nodes.append((v.row,v.col,v.tub))
            #print("\n", v.row,v.col,v.tub,"\n")
            
else:
    print("No path found")
    

fig = plt.figure()

# create 3d axis on the figure
ax = fig.add_subplot(111, projection='3d')

x = [pt[0] for pt in nodes]
y = [pt[1] for pt in nodes]
z = [pt[2] for pt in nodes]

# scatter plot
ax.scatter(x, y, z)

# set axis labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# display the plot


plt.savefig('/home/ankush/Desktop/plot2.png')



