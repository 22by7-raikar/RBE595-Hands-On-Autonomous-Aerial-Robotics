import random
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define constants
x_max = 640
y_max = 480
z_max = 400
EPS = 20
numNodes = 1000

class Node:
    def __init__(self, coord, cost=0, parent=None):
        self.coord = coord
        self.cost = cost
        self.parent = parent

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

q_start = Node([0, 0, 0])
q_goal = Node([640, 400, 180])
nodes = [q_start]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(1, numNodes + 1):
    q_rand = [random.uniform(0, x_max), random.uniform(0, y_max), random.uniform(0, z_max)]
    # ax.scatter(q_rand[0], q_rand[1], q_rand[2], c='b', marker='x')

    for j, node in enumerate(nodes):
        if node.coord == q_goal.coord:
            break

    ndist = [dist_3d(node.coord, q_rand) for node in nodes]
    val, idx = min((val, idx) for (idx, val) in enumerate(ndist))
    q_near = nodes[idx]

    q_new_coord = steer(q_rand, q_near.coord, val, EPS)
    # ax.plot([q_near.coord[0], q_new_coord[0]], [q_near.coord[1], q_new_coord[1]], [q_near.coord[2], q_new_coord[2]], 'k', linewidth=2)

    q_new = Node(q_new_coord)
    q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost

    q_nearest = [node for node in nodes if dist_3d(node.coord, q_new.coord) <= 50]

    q_min = q_near
    C_min = q_new.cost

    for q in q_nearest:
        if q.cost + dist_3d(q.coord, q_new.coord) < C_min:
            q_min = q
            C_min = q.cost + dist_3d(q.coord, q_new.coord)
            # ax.plot([q_min.coord[0], q_new.coord[0]], [q_min.coord[1], q_new.coord[1]], [q_min.coord[2], q_new.coord[2]], 'g')

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
