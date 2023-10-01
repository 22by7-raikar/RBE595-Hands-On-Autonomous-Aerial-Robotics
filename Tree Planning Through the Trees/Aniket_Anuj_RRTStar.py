import matplotlib.pyplot as plt
import numpy as np
import math
import random

goal_bias = 0.1
max_dist = 20
goal_bias_star = 0.1
max_dist_star = 10
start = [5,12,13]
goal = [7,24,25]
bxmin = 0
bxmax = 10
bymin = 0
bymax = 10
bzmin = 0
bzmax = 10



# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node
        self.cost = 0.0  # cost

start = Node(start[0], start[1])  # start node
goal = Node(goal[0], goal[1])  # goal node


def dis(node1, node2):
    """Calculate the euclidean distance between two nodes
    arguments:
        node1 - node 1
        node2 - node 2
    return:
        euclidean distance between two nodes
    """
    return math.dist([node1.row, node1.col, node1.tub], [node2.row, node2.col, node2.tub])

def check_collision(obstacles, node1, node2):
    """Check if the path between two nodes collide with obstacles
    arguments:
        node1 - node 1
        node2 - node 2

    return:
        True if the new node is valid to be connected
    """
    ### YOUR CODE HERE ###
    row_div = np.linspace(node1.row, node2.row)  # default number of divisions is 50
    col_div = np.linspace(node1.col, node2.col)
    tub_div = np.linspace(node1.tub, node2.tub)

    line = zip(row_div, col_div, tub_div)
    for point in line:
        if outside_obstacle(point):
            return False

    return True

def get_new_point(goal_bias):
    """Choose the goal or generate a random point
    arguments:
        goal_bias - the possibility of choosing the goal instead of a random point
    return:
        point - the new point
    """
    ### YOUR CODE HERE ###
    t = random.random()  # Generate a (random)probability value in 0 to 1 range
    new_point = Node(np.random.randint(-bxmin, bxmax), np.random.randint(bymin, bymax), np.random.randint(-bzmin, bzmax))

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
    ### YOUR CODE HERE ###
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
    ### YOUR CODE HERE ###
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
    ### YOUR CODE HERE ###
    for neighbor in neighbors:
        new_cost = new_node.cost + dis(neighbor, new_node)
        if new_cost < neighbor.cost and check_collision(new_node, neighbor):
            neighbor.cost = new_cost
            neighbor.parent = new_node

def RRT_star(n_pts=1000, neighbor_size=20):
    '''RRT* search function
    arguments:
        n_pts - number of points try to sample,
                not the number of final sampled points
        neighbor_size - the neighbor distance

    In each step, extend a new node if possible, and rewire the node and its neighbors
    '''
    # Remove previous result
    # self.init_map()
    print(" --------- RRT* Algorithm ---------- ")

    ### YOUR CODE HERE ###

    # In each step,
    # get a new point,
    # get its nearest node,
    # extend the node and check collision to decide whether to add or drop,
    # if added, rewire the node and its neighbors,
    # and check if reach the neighbor region of the goal if the path is not found.

    for n in range(n_pts):
        # Sample new point and get its nearest neighbor in the tree
        new_point = get_new_point(goal_bias_star)
        near_vertex = get_nearest_node(new_point)

        # Find the node to extend in the direction of new node
        extendable = True if (dis(near_vertex, new_point) <= max_dist_star) and (
                new_point.row != self.goal.row) and (new_point.col != self.goal.col) else False

        step_node = new_point if extendable else self.extend(near_vertex, new_point, max_dist=max_dist_star)

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
        goal_neighbors = self.get_neighbors(self.goal, neighbor_size)

        for neighbor in goal_neighbors:
            if self.check_collision(neighbor, self.goal) and (
                    neighbor.cost + self.dis(neighbor, self.goal)) < self.goal.cost:
                self.goal.parent = neighbor
                self.goal.cost = neighbor.cost + self.dis(neighbor, self.goal)
                self.found = True

    # Output
    if self.found:
        self.vertices.append(self.goal)
        steps = len(self.vertices) - 2
        length = self.goal.cost
        print("It took %d nodes to find the current path" % steps)
        print("The path length is %.2f" % length)
    else:
        print("No path found")

    print(" -------------------------------- ")

    # Draw result
    self.draw_map()
