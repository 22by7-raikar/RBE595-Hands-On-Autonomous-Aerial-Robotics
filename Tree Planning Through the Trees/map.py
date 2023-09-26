import bpy
import heapq
import numpy as np
import math

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

vertices = []
vertices.append(start)

goal_location = (24, 16, 3)
create_sphere(goal_location, 0, 255, 0)
goal = Node(goal_location[0], goal_location[1], goal_location[2])


def dis(point1, point2):
    return math.sqrt(((point1.row - point2.row)**2) + ((point1.col - point2.col)**2) + ((point1.tub - point2.tub)**2)) 


def get_random_point(xmin, xmax, ymin, ymax, zmin, zmax, goal_bias):
    
    xnew = np.random.randint(xmin, xmax)
    ynew = np.random.randint(ymin, ymax)
    znew = np.random.randint(zmin, zmax)
    if np.random.uniform(0,1) > goal_bias:
            return (xnew,ynew,znew)
    
    else:
        return goal_location
        

def get_nearest_node(vertices, point):

    distance = []

    for i in vertices:
        #finding the distance 
        # d = math.sqrt(math.pow(i.row-point[0],2)+math.pow(i.col-point[1],2))
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
        if (point[0] > j[0] or point[1] < j[3]) and (point[1] > j[1] or point[1] < j[4]) and (point[2] > j[2] or point[2] < j[5]):
            return True


def check_node_obs(obstacles, point):

    for j in obstacles:
        if (point.row > j[0] or point.col < j[3]) and (point.col > j[1] or point.col < j[4]) and (point.tub > j[2] or point.tub < j[5]):
            return True
        

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
        # print(obstacles)
        # print("#####################")
        # print(len(obstacles))
        # print("#####################")
        # print(xpt, ypt, zpt)
        # exit(0)
        # p = (0, 0, 0)
        # p[0] = xpt
        # p[1] = ypt
        # p[2] = zpt

        if check_point_obs(obstacles, (xpt, ypt, zpt)) == 0:
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
        if check_collision(obstacles, min_cost_neighbor, new_node) == True:

            new_node.cost = min_cost
            new_node.parent = min_cost_neighbor 

        break

    # Rewire the remaining neighbors
    while neighbor_heap:

        cost, neighbor = heapq.heappop(neighbor_heap)
        potential_cost = new_node.cost + dis(new_node, neighbor) 
        
        if neighbor.cost > potential_cost and check_collision(neighbor, new_node) == True:

            neighbor.parent = new_node
            neighbor.cost = potential_cost

    return neighbor_heap


start_cost = 0
ext_step = 15
goal_bias = 0.075
step_size = bxmax - bxmin
found = False
neighbor_size = 10

for i in range(1000):
    
    point = get_random_point(bxmin, bxmax, bymin, bymax, bzmin, bzmax, goal_bias)
    # point_node = Node(point[0], point[1], point[2])
    
    #if there is an obstacle end the loop to get another point 
    if check_point_obs(obstacles, point):
        continue           

    point = Node(point[0], point[1], point[2])

    #Generating Neighbours
    nn = get_nearest_node(vertices, point)
    dir1, dir2 = get_direction(point,nn)
    newpoint = generate_directed_point(nn, dir1, dir2, step_size)
    newpoint = check_valid(newpoint)
    
    if check_node_obs(obstacles, newpoint):
        continue           

    # newpoint = Node(newpoint[0], newpoint[1], newpoint[2])
    dissn = dis(newpoint, nn)

    if check_collision(obstacles, nn, newpoint):
        newpoint.parent = nn
        newpoint.cost = nn.cost + dis(newpoint, nn)

        neighbors = get_neighbors(newpoint, neighbor_size)
        if len(neighbors) == 0:
            continue

        # neighbors = rewire(newpt, neighbors)
        rewire(newpoint, neighbors, obstacles)
        vertices.append(newpoint)

    else:
        continue
    
    dgoal = dis(newpoint, goal)

    if dgoal < 8:

        found = True
        goal.parent = newpoint
        disg = dis(nn, newpoint)
        goal.cost = newpoint.cost + dgoal
        new_neighbors = get_neighbors(goal, neighbor_size)

        if len(new_neighbors) == 0:
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

    else:
        print("No path found")




