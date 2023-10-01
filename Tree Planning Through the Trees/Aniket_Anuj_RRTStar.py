import matplotlib.pyplot as plt
import numpy as np
import math
import random
import bpy

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



start_coord = [5,16,3]
create_sphere(start_coord, 255, 0, 0)

goal_coord = (24, 15, 5)
create_sphere(goal_coord, 0, 255, 0)


goal_bias = 0.1
max_dist = 20
goal_bias_star = 0.1
max_dist_star = 10
bxmin = 0
bxmax = 10
bymin = 0
bymax = 10
bzmin = 0
bzmax = 10

vertices = []
vertices.append(start_coord)

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.tub = tub  # coordinate
        self.parent = None  # parent node
        self.cost = 0.0  # cost

start = Node(start_coord[0], start_coord[1], start_coord[2])  # start node
goal = Node(goal_coord[0], goal_coord[1], goal_coord[2])  # goal node


def outside_obstacles(point):
    if obstacles[0] < point.row < obstacles[3] and obstacles[1] < point.col < obstacles[4] and obstacles[2] < point.col < obstacles[5]:
        return False
    
    return True


def dis(node1, node2):
    """Calculate the euclidean distance between two nodes
    arguments:
        node1 - node 1
        node2 - node 2
    return:
        euclidean distance between two nodes
    """
    return math.dist([node1.row, node1.col, node1.tub], [node2.row, node2.col, node2.tub])

def check_collision(obstaclesss, node1, node2):
    """Check if the path between two nodes collide with obstaclesss
    arguments:
        node1 - node 1
        node2 - node 2

    return:
        True if the new node is valid to be connected
    """
    row_div = np.linspace(node1.row, node2.row)  # default number of divisions is 50
    col_div = np.linspace(node1.col, node2.col)
    tub_div = np.linspace(node1.tub, node2.tub)

    line = zip(row_div, col_div, tub_div)

    for point in line:
        if outside_obstacles(point):
            return False

    return True

def get_new_point(goal_bias):
    """Choose the goal or generate a random point
    arguments:
        goal_bias - the possibility of choosing the goal instead of a random point
    return:
        point - the new point
    """
    t = random.random()  # Generate a (random)probability value in 0 to 1 range
    new_point = Node(random.uniform(-bxmin, bxmax), random.uniform(bymin, bymax), random.uniform(-bzmin, bzmax))

    return goal if t <= goal_bias else new_point

    """ALTERNATIVELY
    return np.random.choice([self.goal, random_node], p=[goal_bias, 1 - goal_bias])
    """

def get_nearest_node(point, vertices):
    """Find the nearest node in self.vertices with respect to the new point
    arguments:
        point - the new point
    return:
        the nearest node
    """
    min_dist = math.inf

    for vertex in vertices:
        if dis(vertex, point) < min_dist:
            min_dist = dis(vertex, point)
            nearest_node = vertex

    return nearest_node

def extend(node1, node2, max_dist):
    dx = node2.row - node1.row
    dy = node2.col - node1.col
    dz = node2.tub - node1.tub

    dist = dis(node1, node2)

    factor = max_dist / dist
    
    x_step, y_step, z_step = dx * factor, dy * factor, dz * factor
    
    x = node1.row + x_step
    y = node1.col + y_step
    z = node1.tub + z_step

    # Check if out of map
    if x < bxmin:
        x = bxmin
    elif x > bxmax:
        x = bxmax - 1
    if y < bymin:
        y = bymin
    elif y > bymax:
        y = bymax - 1
    if z < bzmin:
        z = bzmin
    elif z > bzmax:
        z = bzmax - 1

    new_node = Node(x, y, z)
    new_node.parent = node1
    new_node.cost = node1.cost + dis(new_node, node1)

    return new_node

def get_neighbors(vertices, new_node, neighbor_size):
    """Get the neighbors that are within the neighbor distance from the node
    arguments:
        new_node - a new node
        neighbor_size - the neighbor distance
    return:
        neighbors - a list of neighbors that are within the neighbor distance
    """
    neighbors = []
    for vertex in vertices:
        if dis(vertex, new_node) < neighbor_size:
            neighbors.append(vertex)
    return neighbors

def rewire(new_node, neighbors):
    """Rewire the new node and all its neighbors
    arguments:
        new_node - the new node
        neighbors - a list of neighbors that are within the neighbor distance from the node
    Rewire the new node if connecting to a new neighbor node will give the least cost.
    Rewire all the other neighbor nodes.
    """
    for neighbor in neighbors:
        new_cost = new_node.cost + dis(neighbor, new_node)
        if new_cost < neighbor.cost and check_collision(new_node, neighbor):
            neighbor.cost = new_cost
            neighbor.parent = new_node

# def RRT_star():
#     '''RRT* search function
#     arguments:
#         n_pts - number of points try to sample,
#                 not the number of final sampled points
#         neighbor_size - the neighbor distance

#     In each step, extend a new node if possible, and rewire the node and its neighbors
#     '''

    # In each step,
    # get a new point,
    # get its nearest node,
    # extend the node and check collision to decide whether to add or drop,
    # if added, rewire the node and its neighbors,
    # and check if reach the neighbor region of the goal if the path is not found.
n_pts=1000
neighbor_size=20

for n in range(n_pts):
    # Sample new point and get its nearest neighbor in the tree
    new_point = get_new_point(goal_bias_star)
    near_vertex = get_nearest_node(new_point)

    # Find the node to extend in the direction of new node
    extendable = True if (dis(near_vertex, new_point) <= max_dist_star) and \
        (new_point.row != goal.row) and (new_point.col != goal.col)  and (new_point.tub != goal.tub) else False

    step_node = new_point if extendable else extend(near_vertex, new_point, max_dist=max_dist_star)

    if check_collision(near_vertex, step_node):
        neighbors = get_neighbors(step_node, neighbor_size)
        min_node = near_vertex
        min_cost = near_vertex.cost + dis(near_vertex, step_node)

        for neighbor in neighbors:
            if check_collision(neighbor, step_node) and (
                    neighbor.cost + dis(neighbor, step_node)) < min_cost:
                min_node = neighbor
                min_cost = neighbor.cost + dis(neighbor, step_node)

        step_node.parent = min_node
        step_node.cost = min_cost
        vertices.append(step_node)
        rewire(step_node, neighbors)

    # Check for neighbors of goal node and connect if there's a neighbor with lower cost than current goal cost
    # This method keeps exploring the tree, even if goal is reached to find a better path to the goal node
    goal_neighbors = get_neighbors(goal, neighbor_size)

    for neighbor in goal_neighbors:
        if check_collision(neighbor, goal) and (
                neighbor.cost + dis(neighbor, goal)) < goal.cost:
            goal.parent = neighbor
            goal.cost = neighbor.cost + dis(neighbor, goal)
            found = True

# Output
if found:
    vertices.append(goal)
    steps = len(vertices) - 2
    length = goal.cost
    print("It took %d nodes to find the current path" % steps)
    print("The path length is %.2f" % length)
else:
    print("No path found")
