from scipy import io
import sys
import os
import math
import numpy as np
from rotplot import rotplot
import matplotlib.pyplot as plt
from Conversion import reading

def complementary(gyro_acc, accl_acc):
    pass

def convert_to_rpy(vio):
    pitch = np.arcsin(vio[0, 2])
    roll = np.arctan2(-vio[1, 2], vio[2, 2])
    yaw = np.arctan2(-vio[0, 1], vio[0, 0])
    return roll, pitch, yaw

def only_gyro(ang_rates, init_rot_mat, imu_ts):
    orientations = []
    rpys_init = convert_to_rpy(init_rot_mat)
    roll = rpys_init[0]
    pitch = rpys_init[1]
    yaw = rpys_init[2]
    rpys = np.array([roll, pitch, yaw])

    for i in range(len(imu_ts) - 1):
        dt = imu_ts[i + 1] - imu_ts[i]
        roll = roll + ang_rates[i][0] * dt
        pitch = pitch + ang_rates[i][1] * dt
        yaw = yaw + ang_rates[i][2] * dt
        
        rpys = np.append(rpys, [roll, pitch, yaw])
        
        orientation = rpy_to_euler(roll, pitch, yaw)  # Assuming you have this function
        orientations.append(orientation)

    return orientations, rpys

def only_acc(accls, imu_time):
    #assume that the IMU is only rotating
    orientations = []

    for i in range(len(imu_time)):
        roll = np.arctan2(accls[i][1], np.sqrt(accls[i][0]**2 + accls[i][2]**2))
        pitch = np.arctan2(accls[i][0], np.sqrt(accls[i][1]**2 + accls[i][2]**2))
        # Calculating yaw with Z-axis as gravity direction
        yaw = np.arctan2(np.sqrt(accls[i][0]**2 + accls[i][1]**2), accls[i][2])
        if i == 0:
            rpys = np.array([roll, pitch, yaw])
        
        rpys = np.append(rpys, [roll, pitch, yaw])

        #Converting Roll, Pitch, Yaw to 
        orientation = rpy_to_euler(rpys[0], rpys[1], rpys[2])
    
    orientations.append(orientation)
    return orientations, rpys

def rpy_to_euler(roll,pitch,yaw):
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)

    # Define the rotation matrix R
    R = np.array([
        [cy*cp, -sy*cr + cy*sp*sr, sy*sr + cy*sp*cr],
        [sy*cp, cy*cr + sy*sp*sr, -cy*sr + sy*sp*cr],
        [-sp, cp*sr, cp*cr]
    ])

    return R

def plotting():
    ax, ay, az, wx, wy, wz, gt, imu_time, gt_time = reading()
    ad = np.empty([len(ax), 3])
    gd = np.empty([len(wx), 3])

    for i in range(len(ax)):
        ad[i] = np.array([ax[i], ay[i], az[i]]) #5645*3
        gd[i] = np.array([wx[i], wy[i], wz[i]])

    init_rot_mat = gt[:, :, 0] #3x3x5561

    gyro_or, g_rpys = only_gyro(gd, init_rot_mat, imu_time)
    accl_or, a_rpys = only_acc(ad, imu_time)
    v_rpys = convert_to_rpy(init_rot_mat)

    for i in range(gt.shape[2]):
        v_rpys = np.append(v_rpys, [convert_to_rpy(gt[:,:,i])])

    # final = complementary(gyro_acc, accl_acc) #To be filled
    imu_time  = imu_time.transpose()
    gt_time  = gt_time.transpose()
    fig, axarr = plt.subplots(3, 1)
    axarr[0].plot(imu_time, g_rpys[0], label = 'gyro', color = 'red')
    axarr[0].plot(gt_time, v_rpys[0], label = 'vicon', color = 'blue')
    axarr[0].plot(imu_time, a_rpys[0], label = 'acc', color = 'green')
    # axarr[0].plot(imu_time, cfroll, label = 'cf', color = 'purple')
    # axarr[0].set_title('Time vs Roll')
    # axarr[1].plot(imu_time, g_pitch, label = 'gyro', color = 'red')
    # axarr[1].plot(gt_time, vicp, label = 'vicon', color = 'blue')
    # axarr[1].plot(imu_time, accp, label = 'acc', color = 'green')
    # axarr[1].plot(imu_time, cfpitch, label = 'cf', color = 'purple')
    # axarr[1].set_title('Time vs Pitch')
    # axarr[2].plot(imu_time, g_yaw, label = 'gyro', color = 'red')
    # axarr[2].plot(gt_time, vicy, label = 'vicon', color = 'blue')
    # axarr[2].plot(imu_time, accp, label = 'vicon', color = 'green')
    # axarr[2].plot(imu_time, cfyaw, label = 'cf', color = 'purple')
    # axarr[2].set_title('Time vs Yaw')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plotting()
 


