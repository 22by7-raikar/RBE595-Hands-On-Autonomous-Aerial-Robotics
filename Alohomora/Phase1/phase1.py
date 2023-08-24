from scipy import io
import sys
import os
import math
from math import pi,atan2, sqrt
import numpy as np
from rotplot import rotplot
import matplotlib.pyplot as plt


def acc_convert(a,b,s):
    return (a+b)/s 

def omg_convert(omega, bias):
    return((3300/1023) * (pi/180) * 0.3 * (omega - bias))

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

# def eul_angles(R):
#     yaw = np.arctan2(R[1][0], R[0][0])
#     pitch = np.arctan2(-R[2][0], np.sqrt(R[0][0]**2 + R[1] [0]**2))
#     roll = np.arctan2(R[2][1], R[2][2])
#     return roll,pitch,yaw

def rotationMatrixToEulerAngles(R) :
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

    


#loading the Vicon data
vicon_path = '/home/ankush/Desktop/YourDirectoryID_p0/Phase1/Data/Train/Vicon/viconRot1.mat'
vx = io.loadmat(vicon_path)
vvalst = vx['rots']
#print(",,,,,,,,,,,,,,,,,,,,,,,,,,,,,",vvalst.shape)
vvals = vvalst.transpose()
vtst = vx['ts']
vts = vtst[0]
#print(vvals)


vicr = []
vicp = []
vicy = []

for r in vvals:
    #print(r)
    roll, p, y = rotationMatrixToEulerAngles(r.transpose())
    vicr.append(roll)
    #print(roll,p,y)
    vicp.append(p)
    vicy.append(y)
    

#vts = np.atleast_1d(vts)

# fig, axarr = plt.subplots(3, 1)
# axarr[0].plot(vts, vicr)
# axarr[0].set_title('Time vs Roll')
# axarr[1].plot(vts, vicp)
# axarr[1].set_title('Time vs Pitch')
# axarr[2].plot(vts, vicy)
# axarr[2].set_title('Time vs Yaw')
# # plt.plot(vts, vicr)
# # plt.xlabel('Time')
# # plt.ylabel('Vicon Roll')
# # plt.title('Vicon Data')
# #plt.grid(True)
# plt.tight_layout()
# plt.show()


#loading the IMU data
imu_path = '/home/ankush/Desktop/YourDirectoryID_p0/Phase1/Data/Train/IMU/imuRaw1.mat'
ix = io.loadmat(imu_path)
i_vals = ix['vals']
print("ivals:", i_vals.shape)
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
for i in range(500):
    bgx.append(ivals[i][4])
    bgy.append(ivals[i][5])
    bgz.append(ivals[i][3])
biasx = sum(bgx)/len(bgx)
biasy = sum(bgy)/len(bgy)
biasz = sum(bgz)/len(bgz)


#print(biasz)

#print(ivals[1])

mivals= []
for j in ivals:
    ax = acc_convert(j[0],B[0],S[0])
    ay = acc_convert(j[1],B[1],S[1])
    az = acc_convert(j[2],B[2],S[2])
    wx = omg_convert(j[4],biasx,)
    wy = omg_convert(j[5],biasy)
    wz = omg_convert(j[3],biasz)
    p = [ax,ay,az,wx,wy,wz]
    mivals.append(p)

# print(mivals[0])
# print(ivals[0])
# print(S)
# print(B)

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
gyro_rpy = []

#Use Slerp 


g_roll = []
g_pitch = []
g_yaw = []

grr, gpp , gyy = rotationMatrixToEulerAngles(vvals[0])
roll = grr
pitch = gpp
yaw = gyy
g_roll.append(grr)
g_pitch.append(gpp)
g_yaw.append(gyy)

for i in range((len(ts)-1)):
    roll = roll + gd[i][0]*(ts[i+1]-ts[i])
    pitch = pitch + gd[i][1]*(ts[i+1]-ts[i])
    yaw = yaw + gd[i][2]*(ts[i+1]-ts[i])
    g_roll.append(roll)
    g_pitch.append(pitch)
    g_yaw.append(yaw)
    orientation = rot_matrix(roll,pitch,yaw)
    g_orientation.append(orientation)

print((g_orientation[2]))





#orientation from accelerometer 
ad = []
for adata in mivals:
    ad.append(adata[:3])
    #print(adata)
    #print(adata[:3])

#print(ad)

a_orientation = []
acc_rpy = []
accr = []
accp = []
accy = []

for ti in range(len(ts)):
    aroll = atan2(ad[i][1],(sqrt(ad[i][0]**2 + ad[i][2]**2)))
    apitch = atan2(-ad[i][0],(sqrt(ad[i][1]**2 + ad[i][2]**2)))
    ayaw = atan2((sqrt(ad[i][0]**2 + ad[i][1]**2)),ad[i][2])
    accr.append(aroll)
    accp.append(apitch)
    accy.append(ayaw)
    orientation = rot_matrix(roll,pitch,yaw)
    a_orientation.append(orientation)






#complementary filter
alpha = 0.2
cf_orientation = []

roll_a = accr[0]
pitch_a = accp[0]
yaw_a = accy[0]
roll_g = g_roll[0]
pitch_g = g_pitch[0]
yaw_g = g_yaw[0]
cf_orientation.append([accr[0]+g_roll[0], accp[0] + g_pitch[0], accy[0] + g_yaw[0]])


for i in range(len(ts)-1):
    roll_g = (1-alpha)*g_roll[i] + (1 - alpha) * (g_roll[i+1] - roll_g)
    pitch_g = (1-alpha)*g_pitch[i] + (1 - alpha) * (g_pitch[i+1] - pitch_g)
    yaw_g = (1-alpha)*g_yaw[i] + (1 - alpha) * (g_yaw[i+1] - yaw_g)
    roll_a = (1-alpha)*accr[i+1] + alpha*roll_a
    pitch_a = (1-alpha)*accp[i+1] + alpha*pitch_a
    yaw_a = (1-alpha)*accy[i+1] + alpha*yaw_a
    cf_orientation.append([(1-alpha)*roll_g + alpha*roll_a, (1-alpha)*pitch_g + alpha*pitch_a, (1-alpha)*yaw_g+alpha*yaw_a])    

cfroll = []
cfpitch = []
cfyaw = []
for g in cf_orientation:
    print(g)
    cfroll.append(g[0])
    cfpitch.append(g[1])
    cfyaw.append(g[2])



print(cf_orientation[1])

fig, axarr = plt.subplots(3, 1)
axarr[0].plot(ts, g_roll, label = 'gyro', color = 'red')
axarr[0].plot(vts, vicr, label = 'vicon', color = 'blue')
axarr[0].plot(ts, accr, label = 'acc', color = 'green')
axarr[0].plot(ts, cfroll, label = 'cf', color = 'purple')
axarr[0].set_title('Time vs Roll')

axarr[1].plot(ts, g_pitch, label = 'gyro', color = 'red')
axarr[1].plot(vts, vicp, label = 'vicon', color = 'blue')
axarr[1].plot(ts, accp, label = 'acc', color = 'green')
axarr[1].plot(ts, cfpitch, label = 'cf', color = 'purple')
axarr[1].set_title('Time vs Pitch')

axarr[2].plot(ts, g_yaw, label = 'gyro', color = 'red')
axarr[2].plot(vts, vicy, label = 'vicon', color = 'blue')
axarr[2].plot(ts, accp, label = 'vicon', color = 'green')
axarr[2].plot(ts, cfyaw, label = 'cf', color = 'purple')
axarr[2].set_title('Time vs Yaw')
plt.tight_layout()
plt.show()
#print(a_orientation[2])


# REye = np.eye(3)
# myAxis = cf_orientation[1]
# RTurn = np.array([[np.cos(np.pi / 2), 0, np.sin(np.pi / 2)], [0, 1, 0], [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2)]])
# rotplot(RTurn, myAxis)
# plt.show()


