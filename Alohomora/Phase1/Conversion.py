from scipy import io
import os
import math
import numpy as np

def acc_conv(acc, b, s):
    return float(acc+b)/s

def ang_conv(angle, b):
    w = (3300/1023) * (math.pi/180) * 0.3 * (angle - b)

def convert(imu_vals, imu_params):
    sx, sy, sz = imu_params[0]          # = [-0.00941012, -0.00944606,  0.00893549]
    bax, bay, baz = imu_params[1]       #=[4.81660203,  4.72727773, -4.42103827]
    imu_vals = imu_vals.transpose()
    # print(imu_vals.shape)
    ax, ay, az = [], [], []
    wx, wy, wz = [], [], []
    calc_bias = 0

    for row in imu_vals:
        # print(len(row[3]))
        # exit(1)
        ax.append(acc_conv(row[0],bax,sx))
        ay.append(acc_conv(row[1],bay,sy))
        az.append(acc_conv(row[2],baz,sz))

        if calc_bias == 0:
            bgz = np.mean(row[3])
            print(row[3])
            bpx = np.mean(row[4])
            bpy = np.mean(row[5])
            calc_bias = 1

        wz.append(ang_conv(row[3],bgz))
        wx.append(ang_conv(row[4],bpx))
        wy.append(ang_conv(row[5],bpy))

    return ax, ay, az, wx, wy, wz

def reading():
    imu_path = "~/Desktop/YourDirectoryID_p0/Phase1/Data/Train/IMU"
    filename = "imuRaw1.mat"
    imu_path = os.path.expanduser(os.path.join(imu_path, filename))
    imu = io.loadmat(imu_path)
    # (['__header__', '__version__', '__globals__', 'vals', 'ts'])
    imu_vals = imu.get('vals')          # print((imu_vals.shape)) #6x5645
    imu_ts = imu.get('ts')              # print(imu_ts.shape) 5645
    bs_path = "~/Desktop/YourDirectoryID_p0/Phase1/" 
    #(['__header__', '__version__', '__globals__', 'IMUParams'])
    bs_file = "IMUParams.mat"
    bs_path = os.path.expanduser(os.path.join(bs_path, bs_file))
    bs = io.loadmat(bs_path)            # print(len(bs))   #4
    imu_params = bs['IMUParams']        # print(imu_params.shape) #2 x 3
    

    v_path = "~/Desktop/YourDirectoryID_p0/Phase1/Data/Train/Vicon" 
    #['__header__', '__version__', '__globals__', 'rots', 'ts'])
    v_file = "viconRot1.mat"
    v_path = os.path.expanduser(os.path.join(v_path, v_file))
    v = io.loadmat(v_path)      # print(len(v)) #5
    gt = v.get('rots')    # print(vicon_gt.shape) #(3, 3, 5561)
    gt_ts = v.get('ts')

    ax,ay,az,wx,wy,wz = convert(imu_vals, imu_params)

    return ax, ay, az, wx, wy, wz, gt, imu_ts, gt_ts

if __name__ == "__main__":
    reading()








