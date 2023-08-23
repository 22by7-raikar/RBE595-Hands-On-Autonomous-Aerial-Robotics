from scipy import io
import sys
import os
from math import pi,atan2, sqrt
import numpy as np
from rotplot import rotplot
import matplotlib.pyplot as plt

def acc_convert(a,b,s):
    return (a+b)/s 

def omg_convert(omega, bias):
    return(3300/1023 * pi/180 * 0.3 * (omega - bias))

def rot_matrix(roll,pitch,yaw):
    Rz = [
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ]
    

    Ry =[
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ]
    
    Rx = [
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ]
    
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R


#loading the Vicon data
vicon_path = '/home/ankush/Desktop/YourDirectoryID_p0/Phase1/Data/Train/Vicon/viconRot1.mat'
vx = io.loadmat(vicon_path)
vvalst = vx['rots']
vvals = vvalst.transpose()
vts = vx['ts']
#print(vvals)

#loading the IMU data
imu_path = '/home/ankush/Desktop/YourDirectoryID_p0/Phase1/Data/Train/IMU/imuRaw1.mat'
ix = io.loadmat(imu_path)
i_vals = ix['vals']
ivals = i_vals.transpose()
its = ix['ts']

#loading the IMU parameters
para_path = '/home/ankush/Desktop/YourDirectoryID_p0/Phase1/IMUParams.mat'
px = io.loadmat(para_path)
parax = px['IMUParams']
#scale values 
S = parax[0]
#bias values 
B = parax[1]
#print(S)

#bias to convert omega
bgx =[]
bgy = []
bgz= []
for i in range(100):
    bgx.append(ivals[i][3])
    bgy.append(ivals[i][4])
    bgz.append(ivals[i][5])
biasx = sum(bgx)/len(bgx)
biasy = sum(bgy)/len(bgy)
biasz = sum(bgz)/len(bgz)

#print(vts)

mivals= []
for j in ivals:
    ax = acc_convert(j[0],B[0],S[0])
    ay = acc_convert(j[1],B[1],S[1])
    az = acc_convert(j[2],B[2],S[2])
    wx = omg_convert(j[3],biasx,)
    wy = omg_convert(j[4],biasy)
    wz = omg_convert(j[5],biasz)
    p = [ax,ay,az,wx,wy,wz]
    mivals.append(p)

print(mivals[0])
print(ivals[0])
print(S)
print(B)

#orientation from gyro data
gd = []
for gdata in mivals:
    gd.append(gdata[3:])

g_orientation = []

#initial orientation
g_orientation.append(vvals[0])

ts = its[0]
print(len(ts))
print(gd[2][2])
for i in range((len(ts)-1)):
    roll = gd[i+1][0]*(ts[i+1]-ts[i])
    pitch = gd[i+1][1]*(ts[i+1]-ts[i])
    yaw = gd[i+1][2]*(ts[i+1]-ts[i])
    orientation = np.array(rot_matrix(roll,pitch,yaw))
    g_orientation.append(orientation)

print((g_orientation[2]))

#orientation from accelerometer 
ad = []
for adata in mivals:
    ad.append(adata[:3])
#print(ad)

a_orientation = []
for ti in range(len(ts)-1):
    aroll = atan2(ad[i][1],(sqrt(ad[i][0]**2 + ad[i][2])))
    apitch = atan2(-ad[i][0],(sqrt(ad[i][1]**2 + ad[i][2])))
    ayaw = atan2((sqrt(ad[i][0]**2 + ad[i][1])),ad[i][2])
    orientation = np.array(rot_matrix(roll,pitch,yaw))
    a_orientation.append(orientation)

print(a_orientation[2])


REye = np.eye(3)
myAxis = a_orientation[2]
RTurn = g_orientation[2]
rotplot(RTurn, myAxis)
plt.show()




