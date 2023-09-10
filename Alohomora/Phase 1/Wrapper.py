import scipy 
from scipy import io
import sys
import os
import math
import numpy as np
from rotplot import rotplot
import matplotlib.pyplot as plt
import yaml
from helpers import conv2ER, rpy2rotmat, ang_conv, acc_conv

curr_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(curr_dir, 'cfg.yaml')) as file:
    try:
        cfg = yaml.safe_load(file)
    except yaml.YAMLError as exception:
        print(exception)


def convert(imu_vals, imu_params):
    sx, sy, sz = imu_params[0]
    bax, bay, baz = imu_params[1]   

    imu_vals = imu_vals.transpose()

    ax, ay, az = [], [], []
    wx, wy, wz = [], [], []
    calc_bias = 0

    imu_vec, imu_acc, imu_gyro = [], [], []  

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
        imu_vec.append([ax, ay, az, wx, wy, wz])

    return imu_vec, imu_acc, imu_gyro


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
    ralf = ra[0]
    palf = pa[0]
    yalf = ya[0]

    rghf = rg[0]
    pghf = pg[0]
    yghf = yg[0]

    rc.append(ra[0] + rg[0])
    pc.append(pa[0] + pg[0])
    yc.append(ya[0] + yg[0])

    rpyc.append([rc[0], pc[0], yc[0]])

    for i in range(len(imu_timestamps)-1):  
        # Low pass acc -> Stable initially, A-readings not good at higher values
        ralf = (1 - af) * ra[i+1] + af * ralf
        palf = (1 - af) * pa[i+1] + af * palf
        yalf = (1 - af) * ya[i+1] + af * yalf

        #High pass gyro -> G-readings Stable later, faster speeds i.e, @ higher values
        rghf = (1-af) * rg[i+1] + (1 - af) * (rg[i+1] - rghf)
        pghf = (1-af) * pg[i+1] + (1 - af) * (pg[i+1] - pghf)
        yghf = (1-af) * yg[i+1] + (1 - af) * (yg[i+1] - yghf)

        a = (1 -ac) * rghf + ac * ralf
        b = (1 -ac) * pghf + ac * palf
        c = (1 -ac) * yghf + ac * yalf

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

if __name__ == '__main__':
    imu_path = cfg['imu']['main_path']
    filename = cfg['imu']['file_name']
    imu_path = os.path.expanduser(os.path.join(imu_path, filename))
    imu = io.loadmat(imu_path)

    imu_vals = imu.get('vals')
    imu_ts = imu.get('ts') 

    imu_timestamps = imu_ts.transpose()
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
    gt_time = gt_ts.transpose()
    #['__header__', '__version__', '__globals__', 'rots', 'ts'])

    imu_complete, imu_a, imu_g = convert(imu_vals, imu_params)
    rg, pg, yg = [], [], []
    rpyg, og = [], []
    
    ra, pa, ya = [], [], []
    rpya, oa = [], []

    rc, pc, yc = [], [], []
    rpyc = []

    #Lets begin with orientation from VICON GT data
    init_rot_mat = gt[:, :, 0]
    imu_timestamps = imu_ts.transpose()

    rg, pg, yg, og, rpyg = gyro(init_rot_mat, imu_timestamps, imu_g)
    ra, pa, ya, oa, rpya = acc(imu_timestamps, imu_a)   


    af = cfg['filter_weight']['low_high']
    ac = cfg['filter_weight']['complementary']

    rc, pc, yc, rpyc = complementary(rg, pg, yg, ra, pa, ya, af, ac)

    roll_gt, pitch_gt, yaw_gt = [], [], []
    imu_time = imu_ts[0]
    gt_time = gt_ts[0]

    roll_gt, pitch_gt, yaw_gt = orientation_from_gt(gt.transpose())
   
    fig, axarr = plt.subplots(3, 1)
    axarr[0].plot(imu_time, rg, label = 'gyro', color = 'red')
    axarr[0].set_title('Time vs Roll')
    axarr[0].plot(gt_time, roll_gt, label = 'vicon', color = 'blue')
    axarr[1].plot(imu_time, pg, label = 'gyro', color = 'red')
    axarr[1].set_title('Time vs Pitch')
    axarr[1].plot(gt_time, pitch_gt, label = 'vicon', color = 'blue')
    axarr[2].plot(imu_time, yg, label = 'gyro', color = 'red')
    axarr[2].set_title('Time vs Yaw')
    axarr[2].plot(gt_time, yaw_gt, label = 'vicon', color = 'blue')

    axarr[0].plot(imu_time, rc, label = 'cf', color = 'green')
    axarr[1].plot(imu_time, pc, label = 'cf', color = 'green')
    axarr[2].plot(imu_time, yc, label = 'cf', color = 'green')
    axarr[0].plot(imu_time, ra, label = 'acc', color = 'black')
    axarr[1].plot(imu_time, pa, label = 'acc', color = 'black')
    axarr[2].plot(imu_time, ya, label = 'acc', color = 'black')

    plt.tight_layout()
    plt.legend()
    plt.show()
