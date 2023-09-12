import scipy 
from scipy import io
import sys
import os
import math
import numpy as np
from rotplot import rotplot
import matplotlib
import matplotlib.pyplot as plt
import yaml
from helpers import *
import cv2

curr_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(curr_dir, 'cfg.yaml')) as file:
    try:
        cfg = yaml.safe_load(file)
    except yaml.YAMLError as exception:
        print(exception)


def convert(imu_vals, imu_params):
    sx, sy, sz = imu_params[0]
    bax, bay, baz = imu_params[1]   

    imu_vals = imu_vals.T

    ax, ay, az = [], [], []
    wx, wy, wz = [], [], []
    calc_bias = 0

    imu_acc, imu_gyro = [], []

    for row in imu_vals:
        ax = acc_conv(row[0],bax,sx)
        ay = acc_conv(row[1],bay,sy)
        az = acc_conv(row[2],baz,sz)

        if calc_bias == 0:
            bgz = np.mean(imu_vals[:500, 3])
            bpx = np.mean(imu_vals[:500, 4])
            bpy = np.mean(imu_vals[:500, 5])
            calc_bias = 1

        wz = ang_conv(row[3],bgz)
        wx = ang_conv(row[4],bpx)
        wy = ang_conv(row[5],bpy)
        imu_acc.append([ax, ay, az])
        imu_gyro.append([wx, wy, wz])

    return imu_acc, imu_gyro


def gyro(init_rot_mat, t, imu_g):
    a, b, c  = conv2ER(init_rot_mat)
    rg.append(a)
    pg.append(b)
    yg.append(c)
    rpyg.append([a, b, c])

    for i in range(len(t)-1):
        dt = t[i+1] - t[i]
        a = a + imu_g[i][0] * dt
        b = b + imu_g[i][1] * dt
        c = c + imu_g[i][2] * dt

        rg.append(a)
        pg.append(b)
        yg.append(c)

        rpyg.append([a, b, c])
        og.append(rpy2rotmat(a,b,c))
    
    return rg, pg, yg, og, rpyg


def acc(t, imu_a):
    for i in range(len(t)):
        a = math.atan2(imu_a[i][1], math.sqrt(imu_a[i][0]**2 + imu_a[i][2]**2))
        b = math.atan2(imu_a[i][0], math.sqrt(imu_a[i][1]**2 + imu_a[i][2]**2))
        c = math.atan2(math.sqrt(imu_a[i][0]**2 + imu_a[i][1]**2), imu_a[i][2])

        ra.append(a)
        pa.append(b)
        ya.append(c)

        rpya.append([a, b, c])
        oa.append(rpy2rotmat(a,b,c))
    
    return ra, pa, ya, oa, rpya


def complementary(rg, pg, yg, ra, pa, ya, af, ac):
    ralp = ra[0]
    palp = pa[0]
    yalp = ya[0]

    rghp = rg[0]
    pghp = pg[0]
    yghp = yg[0]

    rc.append(ra[0] + rg[0])
    pc.append(pa[0] + pg[0])
    yc.append(ya[0] + yg[0])

    rpyc.append([rc[0], pc[0], yc[0]])

    for i in range(len(imu_timestamps)-1):  
        # Low pass acc -> Stable initially, A-reimu_aings not good at higher values``
        ralp = (1 - af) * ra[i+1] + af * ralp
        palp = (1 - af) * pa[i+1] + af * palp
        yalp = (1 - af) * ya[i+1] + af * yalp

        #High pass gyro -> G-reimu_aings Stable later, faster speeds i.e, @ higher values
        rghp = (1-af) * rg[i+1] + (1 - af) * (rg[i+1] - rghp)
        pghp = (1-af) * pg[i+1] + (1 - af) * (pg[i+1] - pghp)
        yghp = (1-af) * yg[i+1] + (1 - af) * (yg[i+1] - yghp)

        a = (1 -ac) * rghp + ac * ralp
        b = (1 -ac) * pghp + ac * palp
        c = (1 -ac) * yghp + ac * yalp

        rc.append(a)
        pc.append(b)
        yc.append(c)

        rpyc.append([a, b, c]) 

    return rc, pc, yc, rpyc


def orientation_from_gt(gt):
    rgt, pgt, ygt = [], [], []
    for x in gt:
        a, b, c = conv2ER(x.T)
        rgt.append(a)
        pgt.append(b)
        ygt.append(c)
    
    return rgt, pgt, ygt


def gt2quatconv_norm(rpya, rpyg, imu_a):
    
    norm_imu_a, norm_aq, norm_gq = [], [], []

    rpya2quat = conv2quat(rpya)
    rpyg2quat = conv2quat(rpyg)
    

    for i in imu_a:
        d = (i[0]**2 + i[1]**2 + i[2]**2)
        norm_imu_a.append([i[0]/d, i[1]/d, i[2]/d])

    for i in rpya2quat:
        norm_aq.append(quat_norm(i))

    for j in rpyg2quat:
        norm_gq.append(quat_norm(j))

    return norm_imu_a, norm_aq, norm_gq


def madgwick(beta, imu_g):

    rm, pm, ym, rpym = [], [], [], []
    q_est = []
    q_est.append(np.array([1,0,0,0]))
    q_esti = q_est[0]

    g = quat2euler(q_esti)
    rpym.append(g)
    rm.append(g[0])
    pm.append(g[1])
    ym.append(g[2])

    for i in range(len(imu_time) - 1):
        gq_est = quat_mult(half_qn(quat_norm(q_esti)), [0,imu_g[i][0],imu_g[i][1],imu_g[i][1]])
        
        del_f1 = np.array([[-2*q_esti[2],   2*q_esti[3],    -2*q_esti[0],   2*q_esti[1]],
                           [ 2*q_esti[1],   2*q_esti[0],     2*q_esti[3],   2*q_esti[2]],
                           [ 0,             -4*q_esti[1],   -4*q_esti[2],   0]]).T
        
        del_f2 = np.array([[2*(q_esti[1] * q_esti[3] - q_esti[0] * q_esti[2]) - imu_a[i][0]],
                           [2*(q_esti[0] * q_esti[1] + q_esti[2] * q_esti[3]) - imu_a[i][1]],
                           [2*((1/2)    - q_esti[1]**2           - q_esti[2]**2 )   - imu_a[i][2]]])

        del_f = np.squeeze(np.dot(del_f1,del_f2))
        norm_delf = math.sqrt(del_f[0]**2+del_f[1]**2+del_f[2]**2+del_f[3]**2)
        
        del_ft = del_f.T
        norm_del_ft = np.array([del_ft[0]/norm_delf, del_ft[1]/norm_delf, del_ft[2]/norm_delf, del_ft[3]/norm_delf])

        fuse = gq_est - beta * (norm_del_ft)

        q_esti = quat_norm(q_esti) + fuse*(imu_time[i+1]-imu_time[i]) 

        g = quat2euler(q_esti)
        rpym.append(g)
        rm.append(g[0])
        pm.append(g[1])
        ym.append(g[2])
    
    return rm, pm, ym, rpym


if __name__ == '__main__':
    imu_path = cfg['imu']['main_path']
    filename = cfg['imu']['file_name']
    imu_path = os.path.expanduser(os.path.join(imu_path, filename))
    imu = io.loadmat(imu_path)

    imu_vals = imu.get('vals')
    imu_ts = imu.get('ts') 

    imu_timestamps = imu_ts.T
    # (['__header__', '__version__', '__globals__', 'vals', 'ts'])

    bs_path = cfg['params']['main_path']
    bs_file = cfg['params']['file_name']
    bs_path = os.path.expanduser(os.path.join(bs_path, bs_file))
    bs = io.loadmat(bs_path)

    imu_params = bs['IMUParams']
    #(['__header__', '__version__', '__globals__', 'IMUParams'])

    v_path = cfg['vicon']['main_path']
    v_file = cfg['vicon']['file_name']
    v_path = os.path.expanduser(os.path.join(v_path, v_file))
    v = io.loadmat(v_path) 

    gt = v.get('rots')
    gt_ts = v.get('ts')
    gt_time = gt_ts.T
    #['__header__', '__version__', '__globals__', 'rots', 'ts'])

    imu_a, imu_g = convert(imu_vals, imu_params)
    rg, pg, yg = [], [], []
    rpyg, og = [], []
    
    ra, pa, ya = [], [], []
    rpya, oa = [], []

    rc, pc, yc = [], [], []
    rpyc = []

    rgt, pgt, ygt = [], [], []

    #Lets begin with orientation from VICON GT data
    init_rot_mat = gt[:, :, 0]
    imu_timestamps = imu_ts.T

    rg, pg, yg, og, rpyg = gyro(init_rot_mat, imu_timestamps, imu_g)
    ra, pa, ya, oa, rpya = acc(imu_timestamps, imu_a)   

    af = cfg['filter_weight']['low_high']
    ac = cfg['filter_weight']['complementary']

    rc, pc, yc, rpyc = complementary(rg, pg, yg, ra, pa, ya, af, ac)

    imu_time = imu_ts[0]
    gt_time = gt_ts[0]

    rgt, pgt, ygt = orientation_from_gt(gt.T)

    norm_imu_a, norm_aq, norm_gq = gt2quatconv_norm(rpya, rpyg, imu_a)

    beta = cfg['filter_weight']['madgwick_trust_factor']

    rm, pm, ym, rpym = madgwick(beta, imu_g)

    # fig, axarr = plt.subplots(3, 1)
    # axarr[0].plot(imu_time, rg, label = 'gyro', color = 'red')
    # axarr[0].plot(imu_time, ra, label = 'acc', color = 'blue')
    # axarr[0].plot(imu_time, rc, label = 'comp', color = 'green')
    # axarr[0].plot(imu_time, rm, label = 'madg', color = 'cyan')
    # axarr[0].plot(gt_time, rgt, label = 'vicon', color = 'black')
    # axarr[0].set_title('Time vs Roll')

    # axarr[1].plot(imu_time, pg, label = 'gyro', color = 'red')
    # axarr[1].plot(imu_time, pa, label = 'acc', color = 'blue')
    # axarr[1].plot(imu_time, pc, label = 'comp', color = 'green')
    # axarr[1].plot(imu_time, pm, label = 'madg', color = 'cyan')
    # axarr[1].plot(gt_time, pgt, label = 'vicon', color = 'black')
    # axarr[1].set_title('Time vs Pitch')

    # axarr[2].plot(imu_time, yg, label = 'gyro', color = 'red')
    # axarr[2].plot(imu_time, ya, label = 'acc', color = 'blue')
    # axarr[2].plot(imu_time, yc, label = 'comp', color = 'green')
    # axarr[2].plot(imu_time, ym, label = 'madg', color = 'cyan')
    # axarr[2].plot(gt_time, ygt, label = 'vicon', color = 'black')
    # axarr[2].set_title('Time vs Yaw')

    # plt.tight_layout()
    # plt.legend()
    # plt.show()
#############################################################################################

sv = []

sv.append([1,0,0,0,0,0,0])


P = np.diag(np.concatenate((np.ones(3), np.ones(3))))

# Create Q, the process noise covariance matrix
Q = 5e-8 * np.block([[np.ones((3, 3)) + np.eye(3), np.zeros((3, 3))],
                     [np.zeros((3, 3)), np.ones((3, 3)) + np.eye(3)]])

R = np.vstack(np.hstack((np.ones((3, 3)) + np.eye(3)), np.zeros((3, 3))), np.hstack(np.zeros((3, 3), (np.ones((3, 3)) + np.eye(3)))))  
S = np.linalg.cholesky((P+Q))

root2S = S * math.sqrt(12) # Can take 6 for numerical stability

np.random.seed(42)

w = np.hstack((-root2S, root2S))
W = w.T

# def find_sigma_pts():
sigma_points = []

for i in range(len(imu_time) - 1):  
    
    dt = imu_time[i+1] - imu_time[i]  
    sigma_pts_proj = []

    for j in W:
        
        curr_sv = sv[0]
        W_i = j
        W_q = W_i[:3]
        W_w  = W_i[3:]

        # angle =  math.sqrt(W_i[0]**2 + W_i[1]**2 + W_i[2]**2)
        n_quat = quat_vec2quat(W_i)
        n_ori = quat_mult(quat_norm(curr_sv[:4]), n_quat)

        new_W = [n_ori[0], n_ori[1], n_ori[2], n_ori[3], curr_sv[3] + W_w[0], curr_sv[4] + W_w[1], curr_sv[5] + W_w[2]] 
        sigma_points.append(new_W)

    for k in sigma_points:
        delta_sigma_quat = omega_vec2quat(k[4:],dt)
        proj_mult_quat = quat_mult(quat_norm(k[:4]), delta_sigma_quat)
        proj_sigma_sv = [proj_mult_quat[0],proj_mult_quat[1],proj_mult_quat[2],proj_mult_quat[3],k[4],k[5],k[6]]
        sigma_pts_proj.append(proj_sigma_sv)


    for l in sigma_pts_proj:
        mean_sigma_pts = np.mean(l, axis = 0)
        cov_sigma_pts = np.cov(sigma_points) 
        
# print(len(sigma_points[0]))

####################################################
#Uncomment the code below to visualize the ROTPLOT.#
####################################################

# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# height, width = 480, 640  # or whatever size you want
# video = cv2.VideoWriter('my_video.mp4', fourcc, 30, (width, height))
# matplotlib.use('Agg')
# for i in range(len(resti)):
#     myAxis = resti[i]
#     plt.figure(figsize=(6.4, 4.8))  # Default size, you can adjust
#     rotplot(myAxis)
#     plt.draw()
#     plt.pause(0.01)  # Pause to make sure plot is rendered

#     # Convert plot to image frame
#     plt.savefig('temp.png')
#     frame = cv2.imread('temp.png')
#     frame = cv2.resize(frame, (width, height))
    
#     # Write frame to video
#     video.write(frame)
 
#     plt.close()

# # Close the video file
# video.release()

# # Remove the temporary file
# os.remove('temp.png')
