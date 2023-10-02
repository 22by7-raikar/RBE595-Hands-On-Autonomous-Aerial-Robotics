import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Constants
x_max = 640
y_max = 480
z_max = 400
EPS = 10
numNodes = 500 #5000
r = 50  # Radius to look for neighbors

# Initialize
q_start = {'coord': np.array([200, 150, 100]), 'cost': 0, 'parent': -1}
q_goal = {'coord': np.array([450, 350, 260]), 'cost': 0, 'parent': -1}
nodes = [q_start]

# Setup plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([0, x_max])
ax.set_ylim([0, y_max])
ax.set_zlim([0, z_max])

# Draw obstacle
obstacle = np.array([220, 200, 150, 200, 140, 100])
x0, y0, z0, dx, dy, dz = obstacle

verts = [ [(x0, y0, z0), (x0, y0 + dy, z0), (x0 + dx, y0 + dy, z0), (x0 + dx, y0, z0)],
          [(x0, y0, z0), (x0, y0, z0 + dz), (x0 + dx, y0, z0 + dz), (x0 + dx, y0, z0)],
          [(x0, y0, z0), (x0, y0, z0 + dz), (x0, y0 + dy, z0 + dz), (x0, y0 + dy, z0)],
          [(x0 + dx, y0, z0), (x0 + dx, y0, z0 + dz), (x0 + dx, y0 + dy, z0 + dz), (x0 + dx, y0 + dy, z0)],
          [(x0, y0 + dy, z0), (x0, y0 + dy, z0 + dz), (x0 + dx, y0 + dy, z0 + dz), (x0 + dx, y0 + dy, z0)],
          [(x0, y0, z0 + dz), (x0, y0 + dy, z0 + dz), (x0 + dx, y0 + dy, z0 + dz), (x0 + dx, y0, z0 + dz)]]

ax.add_collection3d(Poly3DCollection(verts, facecolors='y', linewidths=1, edgecolors='r', alpha=.25))


def dist_3d(q1, q2):
    return np.linalg.norm(q1 - q2)


def in_obstacle(point_coord, o):
    x, y, z = point_coord
    x0, y0, z0, dx, dy, dz = o
    return x0 <= x <= x0 + dx and y0 <= y <= y0 + dy and z0 <= z <= z0 + dz


def no_collision(n2, n1, o):
    M = 20  # Number of points to check along the path
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

def rewire(new_node, nodes, radius, obstacle):
    new_idx = len(nodes) - 1  # index of the new_node as it is appended last
    for idx, node in enumerate(nodes):
        if idx == new_idx:  # skip the comparison between new_node and itself
            continue
        if dist_3d(new_node['coord'], node['coord']) < radius:
            potential_cost = new_node['cost'] + dist_3d(new_node['coord'], node['coord'])
            if potential_cost < node['cost'] and no_collision(new_node['coord'], node['coord'], obstacle):
                # Update Parent
                node['parent'] = new_idx  # the index of new_node
                # Update Cost
                node['cost'] = potential_cost
                

for i in range(numNodes):
    print(i)
    q_rand = np.array([np.random.rand() * x_max, np.random.rand() * y_max, np.random.rand() * z_max])
    ax.scatter(*q_rand, c='b', marker='x')

    ndist = [dist_3d(node['coord'], q_rand) for node in nodes]
    idx = np.argmin(ndist)
    q_near = nodes[idx]

    q_new_coord = steer(q_rand, q_near['coord'], ndist[idx], EPS)
    if no_collision(q_new_coord, q_near['coord'], obstacle):
        ax.plot(*zip(q_near['coord'], q_new_coord), color='k')
        plt.draw()
        plt.pause(0.001)

        q_new = {'coord': q_new_coord, 'cost': dist_3d(q_new_coord, q_near['coord']) + q_near['cost'], 'parent': idx}
        nodes.append(q_new)
        # After adding a new node in your main loop, call rewire function
        rewire(q_new, nodes, r, obstacle)

        if dist_3d(q_new_coord, q_goal['coord']) < EPS:
            break

# Find path
idx_q_end = np.argmin([dist_3d(node['coord'], q_goal['coord']) for node in nodes])
q_end = nodes[idx_q_end]
if no_collision(q_end['coord'], q_goal['coord'], obstacle):
    nodes.append({'coord': q_goal['coord'], 'cost': 0, 'parent': idx_q_end})
    while q_end['parent'] != -1:
        start = nodes[q_end['parent']]
        ax.plot(*zip(q_end['coord'], start['coord']), color='r', linewidth=4)
        plt.draw()
        plt.pause(0.001)
        q_end = start
else:
    print("No collision-free path found to goal")

plt.show()
