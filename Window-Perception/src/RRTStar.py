# import numpy as np
# import time
# import csv
# import math
# import os
# # import matplotlib.pyplot as plt
# # from rrt import calculate_time_taken, prune_path, calc_path


# # Constants
# class Node:
#     def __init__(self, coord):
#         self.coord = coord
#         self.parent = None  # parent node
#         self.cost = 0.0  # cost

# # Initialize
# class RRTstar:
#     def __init__(self, map_array, start, goal, boundary_start, boundary_end, obstacles=None):
#         self.start = Node(coord=np.array(start))
#         self.goal = Node(coord=np.array(goal))
#         self.vertices = []  # list of nodes
#         self.found = False  # found flag
#         self.nodes = [self.start]
#         self.obstacles = obstacles
#         self.bs = boundary_start
#         self.be = boundary_end
#         self.path = []

#     def dist_3d(self, q1, q2):
#         return np.linalg.norm(q1 - q2)

#     def in_obstacle(self, point_coord, o):
#         x, y, z = point_coord
#         x0, y0, z0, dx, dy, dz = o

#         return x0 <= x <= x0 + dx and y0 <= y <= y0 + dy and z0 <= z <= z0 + dz

#     def no_collision(self, n2, n1, o):
#         M = 20

#         for i in np.linspace(0, 1, M):
#             point_coord = n2 * (1 - i) + n1 * i

#             if self.in_obstacle(point_coord, o):
#                 return False

#         return True

#     def steer(self, qr, qn, val, eps):

#         if val >= eps:
#             return qn + ((qr - qn) * eps) / self.dist_3d(qr, qn)
        
#         return qr

#     def rewire(self, new_node, nodes, radius, obstacle):
#         new_idx = len(self.nodes) - 1

#         for idx, node in enumerate(self.nodes):
#             if idx == new_idx:
#                 continue

#             if self.dist_3d(new_node, node.coord) < radius:
#                 potential_cost = new_node.cost + self.dist_3d(new_node, node.coord)

#                 if potential_cost < node.cost and self.no_collision(new_node, node.coord, obstacle):
#                     node.parent = new_idx
#                     node.cost = potential_cost

#     def prune_path(self, path, obstacles):
#         pruned_path = [path[0]]  # Initialize with the start point
#         i = 0

#         while i < len(path) - 1:
#             j = i + 2
#             while j < len(path):
#                 if not any(self.in_obstacle(point, obstacles) for point in path[i + 1:j]):
#                     # No obstacles between path[i] and path[j], remove intermediate waypoints
#                     pruned_path.append(path[j - 1])
#                     i = j - 1
#                     break
#                 j += 1
#             if j == len(path):
#                 # If no clear path found, move to the next waypoint
#                 pruned_path.append(path[i + 1])
#                 i += 1

#         # Add the goal point if it's not already in the pruned path
#         if not np.array_equal(pruned_path[-1], path[-1]):
#             pruned_path.append(path[-1])

#         return np.array(pruned_path)

#     def calculate_time_taken(self, path):
#         avgvel = 2
#         tf = 0 
        
#         for i in range(len(path)-1):

#             x0,y0,z0 = path[i][0],path[i][1],path[i][2]
#             xf,yf,zf = path[i+1][0],path[i+1][1],path[i+1][2]

#             tf = tf + math.ceil(self.dist_3d(path[i+1],path[i])/avgvel)

#         return tf
    
#     def calc_path(self, path, avgvel, total_time):
    
#         t0 = 0
#         i = 0 

#         x_d = []
#         y_d = []
#         z_d = []
#         x_d_dot = []
#         y_d_dot = []
#         z_d_dot = []
#         x_d_ddot = []
#         y_d_ddot = []
#         z_d_ddot = []

        
#         for t in np.arange(0, total_time , 0.05):       
            
#                 x0,y0,z0 = path[i][0],path[i][1],path[i][2]
#                 xf,yf,zf = path[i+1][0],path[i+1][1],path[i+1][2]
#                 print ("this is i ", i,"this is the initial point:", x0, y0,z0,"this is the final point:", xf, yf,zf)

#                 if i == 0:
#                     tf = math.ceil(self.dist_3d(path[i+1],path[i])/avgvel)
            

#                 A = np.array([[1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5)],
#                                         [0, 1, 2 * t0, 3 * pow(t0,2), 4 * pow(t0,3), 5 * pow(t0,4)],
#                                         [0, 0, 2, 6 * t0, 12 * pow(t0,2), 20 * pow(t0,3)],
#                                         [1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5)],
#                                         [0, 1, 2 * tf, 3 * pow(tf,2), 4 * pow(tf,3), 5 * pow(tf,4)],
#                                         [0, 0, 2, 6 * tf, 12 * pow(tf,2), 20 * pow(tf,3)]]) 
                
#                 if i == 0:
#                     B  = np.array([[x0,0,0,xf,2,0],
#                         [y0,0,0,yf,2,0],
#                         [z0,0,0,zf,2,0]])
                    
#                 elif i >= (len(path)-2):
#                     B  = np.array([[x0,2,0,xf,0,0],
#                         [y0,2,0,yf,0,0],
#                         [z0,2,0,zf,0,0]])
                    
#                 else:
#                     B  = np.array([[x0,2,0,xf,2,0],
#                         [y0,2,0,yf,2,0],
#                         [z0,2,0,zf,2,0]])
                
#                 ax = np.linalg.solve(A,B[0])
#                 ay = np.linalg.solve(A,B[1])
#                 az = np.linalg.solve(A,B[2])

#                 x_d.append(ax[0] + ax[1] * t + ax[2] * pow(t,2) + ax[3] * pow(t,3) + ax[4] * pow(t,4) + ax[5] * pow(t,5))
#                 y_d.append(ay[0] + ay[1] * t + ay[2] * pow(t,2) + ay[3] * pow(t,3) + ay[4] * pow(t,4) + ay[5] * pow(t,5))
#                 z_d.append(az[0] + az[1] * t + az[2] * pow(t,2) + az[3] * pow(t,3) + az[4] * pow(t,4) + az[5] * pow(t,5))
                
#                 # Velocity
#                 x_d_dot.append(ax[1] + 2 * ax[2]*t + 3 * ax[3] * pow(t,2) + 4*ax[4] * pow(t,3) + 5 * ax[5] * pow(t,4))
#                 y_d_dot.append(ay[1] + 2 * ay[2]*t + 3 * ay[3] * pow(t,2) + 4*ay[4] * pow(t,3) + 5 * ay[5] * pow(t,4))
#                 z_d_dot.append(az[1] + 2 * az[2]*t + 3 * az[3] * pow(t,2) + 4*az[4] * pow(t,3) + 5 * az[5] * pow(t,4))
                
#                 # Acceleration
#                 x_d_ddot.append(2 * ax[2] + 6 * ax[3] * t + 12 * ax[4] * pow(t,2) + 20 * ax[5] * pow(t,3))
#                 y_d_ddot.append(2 * ay[2] + 6 * ay[3] * t + 12 * ay[4] * pow(t,2) + 20 * ay[5] * pow(t,3))
#                 z_d_ddot.append(2 * az[2] + 6 * az[3] * t + 12 * az[4] * pow(t,2) + 20 * az[5] * pow(t,3))

#                 if t == tf:
#                     i = i+1
#                     t0 = tf
#                     tf = tf + math.ceil(self.dist_3d(path[i+1],path[i])/avgvel)
#                 # writer.writerow([x_d, y_d, z_d, x_d_dot, y_d_dot, z_d_dot, x_d_ddot, y_d_ddot, z_d_ddot])
#                 # print(x_d,y_d,z_d,x_d_dot,y_d_dot,z_d_dot,x_d_ddot,y_d_ddot,z_d_ddot)

#         timestamp = int(time.time())
#         csv_file_name = f"trajectory_{timestamp}.csv"
#         csv_file_path = os.path.join("/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/trajectories/", csv_file_name)

#         with open(csv_file_path, mode="w", newline="") as file:
#             writer = csv.writer(file)

#             # Writing each list as a row
#             writer.writerow(x_d)
#             writer.writerow(y_d)
#             writer.writerow(z_d)
#             writer.writerow(x_d_dot)
#             writer.writerow(y_d_dot)
#             writer.writerow(z_d_dot)
#             writer.writerow(x_d_ddot)
#             writer.writerow(y_d_ddot)
#             writer.writerow(z_d_ddot)

#         return csv_file_path

#     def RRT(self, avgvel = 2, numNodes=500, r = 50, EPS = 10):

#         #Get the path
#         for i in range(numNodes):
#             q_rand = np.array([np.random.uniform(self.bs[0], self.be[0]), np.random.uniform(self.bs[1], self.be[1]), np.random.uniform(self.bs[2], self.be[2])])
#             # q_rand = np.array([np.random.rand() * self.be[0], np.random.rand() * self.be[1], np.random.rand() * self.be[2]])
            
#             ndist = [self.dist_3d(node.coord, q_rand) for node in self.nodes]
#             idx = np.argmin(ndist)
#             q_near = self.nodes[idx]
#             q_new_coord = self.steer(q_rand, q_near.coord, ndist[idx], EPS)

#             if self.no_collision(q_new_coord, q_near.coord, self.obstacles):
#                 q_new = Node(q_new_coord)
#                 q_new_cost = q_near.cost + self.dist_3d(q_new_coord, q_near.coord)
#                 q_new.parent = idx
#                 q_new.cost = q_new_cost
#                 self.nodes.append(q_new)
#                 self.rewire(q_new, self.nodes, r, self.obstacles)

#                 if self.dist_3d(q_new.coord, self.goal.coord) < EPS:
#                     self.found = True
#                     break

#         if self.found:
#             idx_q_end = len(self.nodes) - 1
#             q_end = self.nodes[idx_q_end]
#             self.path = []
#             # trajectory = []

#             while q_end.parent is not None:
#                 self.path.append(q_end.coord)
#                 start = self.nodes[q_end.parent]
#                 q_end = start
            
#             self.path.append(self.start.coord)
#             self.path.reverse()

#             path_found = True  # Set the flag to True since a path is found          
            
#             obs_coll_pru_path = self.prune_path(self.path)
#             time_taken = self.calculate_time_taken(obs_coll_pru_path)
#             trajectory_file = self.calc_path(self.path, avgvel, time_taken)  # Calculate the trajectory from q_end to start
            
#             return trajectory_file    

#         else:
#             print("No collision-free path found to the goal")
#             return None

# # Example usage
# if __name__ == '__main__':
#     # obstacles = [(5, 5, 1, 5, 5, 5)]  # Example obstacle, format: (x0, y0, z0, dx, dy, dz)
#     rrt_star = RRTstar(map_array = None, start=(0, 0, 0), goal=(40, 30, 3), boundary_start = (0, 0, 0), boundary_end = (45, 36, 6), obstacles=obstacles)
#     rrt_star.RRT()


import numpy as np
import time
import csv
import math
import os

class Node:
    
    def __init__(self, coord):
        self.coord = coord
        self.parent = None
        self.cost = 0.0


class RRTstar:
    
    def __init__(self, start, goal, boundary, obstacles=None):
        
        self.start = Node(coord=np.array(start))
        self.goal = Node(coord=np.array(goal))
        self.nodes = [self.start]
        self.obstacles = obstacles
        self.boundary = boundary
        self.path = []


    def dist_3d(self, q1, q2):
        
        return np.linalg.norm(q1 - q2)


    def in_obstacle(self, point_coord, obstacle):
        
        x, y, z = point_coord
        x0, y0, z0, dx, dy, dz = obstacle
        
        return x0 <= x <= x0 + dx and y0 <= y <= y0 + dy and z0 <= z <= z0 + dz


    def no_collision(self, n2, n1, obstacle):
        
        M = 20
        
        for i in np.linspace(0, 1, M):
            point_coord = n2 * (1 - i) + n1 * i
        
            if self.in_obstacle(point_coord, obstacle):
                return False
        
        return True
    

    def steer(self, qr, qn, val, eps):
        
        if val >= eps:
            return qn + ((qr - qn) * eps) / self.dist_3d(qr, qn)
        return qr
    

    def rewire(self, new_node, radius, obstacle):
        
        new_idx = len(self.nodes) - 1
        
        for idx, node in enumerate(self.nodes):
        
            if idx == new_idx:
                continue
        
            if self.dist_3d(new_node.coord, node.coord) < radius:

                potential_cost = new_node.cost + self.dist_3d(new_node.coord, node.coord)
        
                if potential_cost < node.cost and self.no_collision(new_node.coord, node.coord, obstacle):
                    
                    node.parent = new_idx
                    node.cost = potential_cost


    def prune_path(self, path):
        
        pruned_path = [path[0]]
        i = 0

        while i < len(path) - 1:
            j = i + 2
         
            while j < len(path):

                if not any(self.in_obstacle(point, self.obstacles) for point in path[i + 1:j]):

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


    def calculate_time_taken(self, path, avgvel):
        
        tf = 0
        
        for i in range(len(path) - 1):

            tf += math.ceil(self.dist_3d(path[i + 1], path[i]) / avgvel)
        
        return tf

    def calc_path(self, path, avgvel, total_time):
        
        t0 = 0
        i = 0
        x_d, y_d, z_d, x_d_dot, y_d_dot, z_d_dot, x_d_ddot, y_d_ddot, z_d_ddot = [], [], [], [], [], [], [], [], []
        
        for t in np.arange(0, total_time, 0.05):

            x0, y0, z0 = path[i][0], path[i][1], path[i][2]
            xf, yf, zf = path[i + 1][0], path[i + 1][1], path[i + 1][2]
        
            if i == 0:

                tf = math.ceil(self.dist_3d(path[i + 1], path[i]) / avgvel)
            
            A = np.array([[1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5)],
                          [0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4)],
                          [0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3)],
                          [1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5)],
                          [0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4)],
                          [0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3)]])
            
            if i == 0:

                B = np.array([[x0, 0, 0, xf, 2, 0],
                              [y0, 0, 0, yf, 2, 0],
                              [z0, 0, 0, zf, 2, 0]])
        
            elif i >= (len(path) - 2):
                
                B = np.array([[x0, 2, 0, xf, 0, 0],
                              [y0, 2, 0, yf, 0, 0],
                              [z0, 2, 0, zf, 0, 0]])
        
            else:
                
                B = np.array([[x0, 2, 0, xf, 2, 0],
                              [y0, 2, 0, yf, 2, 0],
                              [z0, 2, 0, zf, 2, 0]])
            
            ax = np.linalg.solve(A, B[0])
            ay = np.linalg.solve(A, B[1])
            az = np.linalg.solve(A, B[2])
            
            x_d.append(ax[0] + ax[1] * t + ax[2] * pow(t, 2) + ax[3] * pow(t, 3) + ax[4] * pow(t, 4) + ax[5] * pow(t, 5))
            y_d.append(ay[0] + ay[1] * t + ay[2] * pow(t, 2) + ay[3] * pow(t, 3) + ay[4] * pow(t, 4) + ay[5] * pow(t, 5))
            z_d.append(az[0] + az[1] * t + az[2] * pow(t, 2) + az[3] * pow(t, 3) + az[4] * pow(t, 4) + az[5] * pow(t, 5))
            
           
            x_d_dot.append(ax[1] + 2 * ax[2] * t + 3 * ax[3] * pow(t, 2) + 4 * ax[4] * pow(t, 3) + 5 * ax[5] * pow(t, 4))
            y_d_dot.append(ay[1] + 2 * ay[2] * t + 3 * ay[3] * pow(t, 2) + 4 * ay[4] * pow(t, 3) + 5 * ay[5] * pow(t, 4))
            z_d_dot.append(az[1] + 2 * az[2] * t + 3 * az[3] * pow(t, 2) + 4 * az[4] * pow(t, 3) + 5 * az[5] * pow(t, 4))


            x_d_ddot.append(2 * ax[2] + 6 * ax[3] * t + 12 * ax[4] * pow(t, 2) + 20 * ax[5] * pow(t, 3))
            y_d_ddot.append(2 * ay[2] + 6 * ay[3] * t + 12 * ay[4] * pow(t, 2) + 20 * ay[5] * pow(t, 3))
            z_d_ddot.append(2 * az[2] + 6 * az[3] * t + 12 * az[4] * pow(t, 2) + 20 * az[5] * pow(t, 3))

            if t == tf:
                i = i + 1
                t0 = tf
                tf = tf + math.ceil(self.dist_3d(path[i + 1], path[i]) / avgvel)

        # Save the trajectory to a CSV file
        timestamp = int(time.time())
        csv_file_name = f"trajectory_{timestamp}.csv"
        csv_file_path = os.path.join("/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/trajectories/", csv_file_name)

        with open(csv_file_path, mode="w", newline="") as file:

            writer = csv.writer(file)
            writer.writerow(x_d)
            writer.writerow(y_d)
            writer.writerow(z_d)
            writer.writerow(x_d_dot)
            writer.writerow(y_d_dot)
            writer.writerow(z_d_dot)
            writer.writerow(x_d_ddot)
            writer.writerow(y_d_ddot)
            writer.writerow(z_d_ddot)

        return csv_file_path

    def RRT(self, avgvel=2, numNodes=500, r=50, EPS=10):
        
        for i in range(numNodes):
        
            q_rand = np.array([np.random.uniform(self.boundary[0], self.boundary[3]),
                              np.random.uniform(self.boundary[1], self.boundary[4]),
                              np.random.uniform(self.boundary[2], self.boundary[5])])

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
                self.rewire(q_new, r, self.obstacles)

                if self.dist_3d(q_new.coord, self.goal.coord) < EPS:
                    self.found = True
                    break

        if self.found:
        
            idx_q_end = len(self.nodes) - 1
            q_end = self.nodes[idx_q_end]
            self.path = []

            while q_end.parent is not None:

                self.path.append(q_end.coord)
                start = self.nodes[q_end.parent]
                q_end = start

            self.path.append(self.start.coord)
            self.path.reverse()

            path_found = True

            obs_coll_pru_path = self.prune_path(self.path)
            time_taken = self.calculate_time_taken(obs_coll_pru_path, avgvel)
            trajectory_file = self.calc_path(self.path, avgvel, time_taken)

            return trajectory_file
        
        else:

            print("No collision-free path found to the goal")
            return None

# Example usage
if __name__ == '__main__':
    # Example usage:
    obstacles = [(5, 5, 1, 5, 5, 5)]  # Example obstacle, format: (x0, y0, z0, dx, dy, dz)
    rrt_star = RRTstar(start=(0, 0, 0), goal=(40, 30, 3), boundary=(0, 0, 0, 45, 36, 6), obstacles=obstacles)
    rrt_star.RRT()
