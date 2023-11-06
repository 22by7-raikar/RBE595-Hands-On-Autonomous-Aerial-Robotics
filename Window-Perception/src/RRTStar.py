import numpy as np
# import matplotlib.pyplot as plt

# Constants
class Node:
    def __init__(self, coord):
        self.coord = coord
        self.parent = None  # parent node
        self.cost = 0.0  # cost

# Initialize
class RRTstar:
    def __init__(self, map_array, start, goal, boundary_start, boundary_end, obstacles=None)):
        self.start = Node(coord=np.array(start))
        self.goal = Node(coord=np.array(goal))
        self.vertices = []  # list of nodes
        self.found = False  # found flag
        self.nodes = [self.start]
        self.obstacles = obstacles
        self.bs = boundary_start
        self.be = boundary_end
        self.path = []

    def dist_3d(self, q1, q2):
        return np.linalg.norm(q1 - q2)

    def in_obstacle(self, point_coord, o):
        x, y, z = point_coord
        x0, y0, z0, dx, dy, dz = o

        return x0 <= x <= x0 + dx and y0 <= y <= y0 + dy and z0 <= z <= z0 + dz

    def no_collision(self, n2, n1, o):
        M = 20

        for i in np.linspace(0, 1, M):
            point_coord = n2 * (1 - i) + n1 * i

            if self.in_obstacle(point_coord, o):
                return False

        return True

    def steer(self, qr, qn, val, eps):

        if val >= eps:
            return qn + ((qr - qn) * eps) / self.dist_3d(qr, qn)
        
        return qr

    def rewire(self, new_node, nodes, radius, obstacle):
        new_idx = len(self.nodes) - 1

        for idx, node in enumerate(self.nodes):
            if idx == new_idx:
                continue

            if self.dist_3d(new_node, node.coord) < radius:
                potential_cost = new_node.cost + self.dist_3d(new_node, node.coord)

                if potential_cost < node.cost and self.no_collision(new_node, node.coord, obstacle):
                    node.parent = new_idx
                    node.cost = potential_cost

    def RRT(self, numNodes=500):
        EPS = 10
        r = 50  # Radius to look for neighbors

        for i in range(numNodes):
            q_rand = np.array([np.random.uniform(self.bs[0], self.be[0]), np.random.uniform(self.bs[1], self.be[1]), np.random.uniform(self.bs[2], self.be[2])])
            # q_rand = np.array([np.random.rand() * self.be[0], np.random.rand() * self.be[1], np.random.rand() * self.be[2]])
            
            ndist = [self.dist_3d(node.coord, q_rand) for node in self.nodes]
            idx = np.argmin(ndist)
            q_near = self.nodes[idx]
            q_new_coord = self.steer(q_rand, q_near.coord, ndist[idx], EPS)

            if self.no_collision(q_new_coord, q_near.coord, self.obstacles):
                q_new = Node(q_new_coord)
                q_new_cost = q_near.cost + self.dist_3d(q_new_coord, q_near.coord)
                q_new.parent = idx
                q_new.cost = q_new_cost
                self.nodes.append(q_new)
                self.rewire(q_new, self.nodes, r, self.obstacles)

                if self.dist_3d(q_new.coord, self.goal.coord) < EPS:
                    self.found = True
                    break

        if self.found:
            idx_q_end = len(self.nodes) - 1
            q_end = self.nodes[idx_q_end]

            while q_end.parent is not None:
                start = self.nodes[q_end.parent]
                self.path.append(q_end.coord)
                # plt.plot([q_end.coord[0], start.coord[0]], [q_end.coord[1], start.coord[1]], color='r', linewidth=4)
                q_end = start
            
            self.path.append(self.start.coord)
            self.path = self.path[::-1]
            
            return self.path    

        else:
            print("No collision-free path found to the goal")

            return []

# Example usage
if __name__ == '__main__':
    obstacles = [(5, 5, 1, 5, 5, 5)]  # Example obstacle, format: (x0, y0, z0, dx, dy, dz)
    rrt_star = RRTstar(map_array = None, start=(0, 0, 0), goal=(40, 30, 3), boundary_start = (0, 0, 0), boundary_end = (45, 36, 6), obstacles=obstacles)
    rrt_star.RRT()

# Import the RRTstar class from your rrt_star script
# from rrt_star import RRTstar

# def calculate_path():
#     obstacles = [(5, 5, 1, 5, 5, 5)]  # Example obstacle, format: (x0, y0, z0, dx, dy, dz)
    
#     # Define your start and goal coordinates
#     start = (0, 0, 0)
#     goal = (40, 30, 3)
    
#     # Create an instance of the RRTstar class with the appropriate parameters
#     rrt_star = RRTstar(map_array=None, start=start, goal=goal, boundary_start=(0, 0, 0), boundary_end=(45, 36, 6), obstacles=obstacles)

#     # Call the RRT method to get the path
#     path = rrt_star.RRT()

#     if path:
#         print("Found a path:", path)
#     else:
#         print("No collision-free path found to the goal")
