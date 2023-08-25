from scipy import io
import sys
import os
import math
import numpy as np
from rotplot import rotplot
import matplotlib.pyplot as plt

def acc_conv(acc, b, s):
    return (float(acc+b)/float(s))

def ang_conv(angle, b):
    return (3300/1023) * (math.pi/180) * 0.3 * (angle - b)

def convert(imu_vals, imu_params):
    sx, sy, sz = imu_params[0]          # = [-0.00941012, -0.00944606,  0.00893549]
    bax, bay, baz = imu_params[1]       # =[4.81660203,  4.72727773, -4.42103827]
                                        # print(imu_vals.shape) = #6 x 5645
    imu_vals = imu_vals.transpose()
                                        # print(imu_vals.shape) = #5645 x 6
    ax, ay, az = [], [], []
    wx, wy, wz = [], [], []
    calc_bias = 0
                                        # print(imu_vals[:, 5])
    imu_vec = []                        #5645 x 6 with updated values
    imu_acc = []
    imu_gyro = []
    for row in imu_vals:
        ax.append(acc_conv(row[0],bax,sx))
        ay.append(acc_conv(row[1],bay,sy))
        az.append(acc_conv(row[2],baz,sz))

        if calc_bias == 0:
            bgz = np.mean(imu_vals[:500, 3])
            bpx = np.mean(imu_vals[:500, 4])
            bpy = np.mean(imu_vals[:500, 5])
            calc_bias = 1

        wz.append(ang_conv(row[3],bgz))
        wx.append(ang_conv(row[4],bpx))
        wy.append(ang_conv(row[5],bpy))
        imu_acc.append([ax, ay, az])
        imu_gyro.append([wx, wy, wz])
        imu_vec.append([ax, ay, az, wz, wx, wy])
    
    return imu_vec, imu_acc, imu_gyro

def rpy_to_euler(r,p,y):
    cy = math.cos(y)
    sy = math.sin(y)
    cp = math.cos(p)
    sp = math.sin(p)
    cr = math.cos(r)
    sr = math.sin(r)

    # Define the rotation matrix R
    R = [[cy*cp, -sy*cr + cy*sp*sr, sy*sr + cy*sp*cr],
        [sy*cp, cy*cr + sy*sp*sr, -cy*sr + sy*sp*cr],
        [-sp, cp*sr, cp*cr]]

    return R

imu_path = "~/Desktop/YourDirectoryID_p0/Phase1/Data/Train/IMU"
filename = "imuRaw1.mat"
imu_path = os.path.expanduser(os.path.join(imu_path, filename))
imu = io.loadmat(imu_path)
imu_vals = imu.get('vals')          # print((imu_vals.shape)) #6x5645
imu_ts = imu.get('ts')              # print(imu_ts.shape) 5645
imu_time = imu_ts.transpose()
# (['__header__', '__version__', '__globals__', 'vals', 'ts'])

bs_path = "~/Desktop/YourDirectoryID_p0/Phase1/" 
bs_file = "IMUParams.mat"
bs_path = os.path.expanduser(os.path.join(bs_path, bs_file))
bs = io.loadmat(bs_path)            # print(len(bs))   #4
imu_params = bs['IMUParams']        # print(imu_params.shape) #2 x 3
#(['__header__', '__version__', '__globals__', 'IMUParams'])

v_path = "~/Desktop/YourDirectoryID_p0/Phase1/Data/Train/Vicon" 
v_file = "viconRot1.mat"
v_path = os.path.expanduser(os.path.join(v_path, v_file))
v = io.loadmat(v_path)      # print(len(v)) #5    
gt = v.get('rots')    # print(vicon_gt.shape) #(3, 3, 5561)
gt_ts = v.get('ts')
gt_time = gt_ts.transpose()
#['__header__', '__version__', '__globals__', 'rots', 'ts'])

imu_ag, imu_a, imu_g = convert(imu_vals, imu_params)
print(imu_g)
orientation_cf, orientation_gyro, orientation_acc = [], [], []
rpy_gyro, rpy_acc = [],[]

#initial orientation
def convert_to_rpy(vio):
    pitch = math.asin(vio[0][2])
    roll = math.atan2(-vio[1][2], vio[2][2])
    yaw = math.atan2(-vio[0][1], vio[0][0])
    return roll, pitch, yaw

init_rot_mat = gt[:, :, 0] #3x3x5561

def only_gyro(init_rot_mat, imu_ts, imu_g):
    rpys_init = convert_to_rpy(init_rot_mat)
    rpy_vec = []
    orientations = []
    roll = rpys_init[0]
    pitch = rpys_init[1]
    yaw = rpys_init[2]

    steps = imu_ts[0] -1
    time = imu_ts[0]

    for i in range(len(steps)):
        dt = time[i + 1] - time[i]
        print(imu_g[i][0])
        roll = roll + imu_g[i][0] * dt
        pitch = pitch + imu_g[i][1] * dt
        yaw = yaw + imu_g[i][2] * dt
        rpy_vec.append([roll, pitch, yaw])
        
        orientation = rpy_to_euler(roll, pitch, yaw) 
        orientations.append(orientation)
    
    return orientations, rpy_vec


def only_acc(imu_ts, imu_a):
    #assume that the IMU is only rotating
    steps = imu_ts[0]
    orientations = []
    rpy_vec = []
    roll, pitch, yaw = [], [], []

    for i in range(len(steps)):
        roll = math.atan2(imu_a[i][1], math.sqrt(imu_a[i][0]**2 + imu_a[i][2]**2))
        pitch = math.atan2(imu_a[i][0], math.sqrt(imu_a[i][1]**2 + imu_a[i][2]**2))
        # Calculating yaw with Z-axis as gravity direction
        yaw = math.atan2(math.sqrt(imu_a[i][0]**2 + imu_a[i][1]**2), imu_a[i][2])
        rpy_vec.append([roll, pitch, yaw])

        #Converting Roll, Pitch, Yaw to 
        orientation = rpy_to_euler(roll, pitch, yaw)
    
    orientations.append(orientation)
    return orientations, rpy_vec

def complementary(orientation_gyro, orientation_acc, rpy_gyro, rpy_acc, alpha):
    orientations = []
    for i in range(len(ts)-1): 
        roll = (1-alpha)*rpy_gyro[i][0] + alpha*rpy_acc[i][0]
        pitch = (1-alpha)*rpy_gyro[i][1] + alpha*rpy_acc[i][1]
        yaw = (1-alpha)*rpy_gyro[i][2] + alpha*rpy_acc[i][2] 
        orientation = rpy_to_euler(roll,pitch,yaw)
        orientations.append(orientation) 
        return orientations

alpha = 0.25
orientation_gyro, rpy_gyro = only_gyro(init_rot_mat, imu_ts, imu_g)
orientation_acc, rpy_acc = only_gyro(imu_ts, imu_a)
orientation_cf = complementary(orientation_gyro, orientation_acc, rpy_gyro, rpy_acc, alpha)
print(orientation_cf)
