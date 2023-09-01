from scipy import io
import sys
import os
import math
from math import pi,atan2, sqrt, sin, cos
import numpy as np
from rotplot import rotplot
import matplotlib.pyplot as plt


def acc_convert(a,b,s):
    #return (a+b)/s
    return(( a*s +b )*9.81) 

def omg_convert(omega, bias):
    return((3300/1023) * (pi/180) * 0.3 * (omega - bias))

def quat_norm(q):
    # print("\n",q)
    d = sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2)
    return [q[0]/d, q[1]/d, q[2]/d, q[3]/d]

def convert_to_quat(euler):
    quat = []
    for e in euler:
        cr = cos(e[0] * 0.5)
        sr = sin(e[0] * 0.5)
        cp = cos(e[1] * 0.5)
        sp = sin(e[1] * 0.5)
        cy = cos(e[2] * 0.5)
        sy = sin(e[2] * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        quat.append([qw, qx, qy, qz])
    return quat 

def quat_to_euler(q):
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]

    # Calculate pitch (rotation around the x-axis)
    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1:
        pitch = math.pi / 2 if sin_pitch > 0 else -math.pi / 2
    else:
        pitch = math.asin(sin_pitch)

    # Calculate yaw (rotation around the z-axis)
    sin_yaw_cosp = 2.0 * (w * z + x * y)
    cos_yaw_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cosp, cos_yaw_cosp)

    # Calculate roll (rotation around the y-axis)
    sin_roll = 2.0 * (w * x - y * z)
    if abs(sin_roll) >= 1:
        roll = math.pi / 2 if sin_roll > 0 else -math.pi / 2
    else:
        roll = math.asin(sin_roll)

    return [roll, pitch, yaw]


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

def orientation_from_gyro(roll, pitch, yaw,ts):
    roll = roll + gd[i][0]*(ts[i+1]-ts[i])
    pitch = pitch + gd[i][1]*(ts[i+1]-ts[i])
    yaw = yaw + gd[i][2]*(ts[i+1]-ts[i])
    return roll, pitch, yaw

def orientation_from_acc(ad,ti):
    aroll = atan2(ad[ti][1],(sqrt(ad[ti][0]**2 + ad[ti][2]**2)))
    apitch = atan2(-ad[ti][0],(sqrt(ad[ti][1]**2 + ad[ti][2]**2)))
    ayaw = atan2((sqrt(ad[ti][0]**2 + ad[ti][1]**2)),ad[ti][2])
    return aroll, apitch, ayaw

def rotationMatrixToEulerAngles(R):
    yaw = np.arctan2(R[1][0], R[0][0])
    pitch = np.arctan2(-R[2][0], np.sqrt(R[0][0]**2 + R[1] [0]**2))
    roll = np.arctan2(R[2][1], R[2][2])
    return [roll,pitch,yaw]

def cf(cf_orientation,alpha,g_roll,g_pitch,g_yaw,roll_g,pitch_g,yaw_g,accr,accp,accy,roll_a,pitch_a,yaw_a,i,falpha):
    roll_g = (1-alpha)*g_roll[i] + (1 - alpha) * (g_roll[i+1] - roll_g)
    pitch_g = (1-alpha)*g_pitch[i] + (1 - alpha) * (g_pitch[i+1] - pitch_g)
    yaw_g = (1-alpha)*g_yaw[i] + (1 - alpha) * (g_yaw[i+1] - yaw_g)
    roll_a = (1-alpha)*accr[i+1] + alpha*roll_a
    pitch_a = (1-alpha)*accp[i+1] + alpha*pitch_a
    yaw_a = (1-alpha)*accy[i+1] + alpha*yaw_a
    cf_orientation.append([(1-falpha)*roll_g + falpha*roll_a, (1-falpha)*pitch_g + falpha*pitch_a, (1-falpha)*yaw_g+falpha*yaw_a])  
    return cf_orientation

def quat_mult(x,y):
    a1 = x[0]
    b1 = x[1]
    c1 = x[2]
    d1 = x[3]
    a2 = y[0]
    b2 = y[1]
    c2 = y[2] 
    d2 = y[3]

    
    if math.isnan(x[0]):
        exit(0)

    a = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
    b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
    c = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
    d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2

    return [a,b,c,d]

#loading the Vicon data
vicon_path = '/home/hasithab/Downloads/sbachimanchi_p0/Phase1/Data/Train/Vicon/viconRot6.mat'
vx = io.loadmat(vicon_path)
vvalst = vx['rots']
vvals = vvalst.transpose()
vtst = vx['ts']
vts = vtst[0]


vicr = []
vicp = []
vicy = []

for r in vvals:
    roll, p, y = rotationMatrixToEulerAngles(r.transpose())
    vicr.append(roll)
    vicp.append(p)
    vicy.append(y)


#loading the IMU data
imu_path = '/home/hasithab/Downloads/sbachimanchi_p0/Phase1/Data/Train/IMU/imuRaw6.mat'
ix = io.loadmat(imu_path)
i_vals = ix['vals']
ivals = i_vals.transpose()
its = ix['ts']
para_path = '/home/hasithab/Downloads/sbachimanchi_p0/Phase1/IMUParams.mat'
px = io.loadmat(para_path)
parax = px['IMUParams']
#scale values 
S = parax[0]
#bias values 
B = parax[1]

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

mivals= []
imu_g = []
for j in ivals:
    ax = acc_convert(j[0],B[0],S[0])
    ay = acc_convert(j[1],B[1],S[1])
    az = acc_convert(j[2],B[2],S[2])
    wx = omg_convert(j[4],biasx,)
    wy = omg_convert(j[5],biasy)
    wz = omg_convert(j[3],biasz)
    imu_g.append([wx,wy,wz]) 
    p = [ax,ay,az,wx,wy,wz]
    mivals.append(p)

#orientation from gyro data
gd = []
for gdata in mivals:
    gd.append(gdata[3:])

g_orientation = []

#initial orientation
g_orientation.append(vvals[0])

ts = its[0]
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
g_euler = []
g_euler.append([grr,gpp,gyy])
for i in range((len(ts)-1)):
    roll, pitch, yaw = orientation_from_gyro(roll, pitch, yaw,ts)  
    g_roll.append(roll)
    g_pitch.append(pitch)
    g_yaw.append(yaw)
    g_euler.append([roll,pitch,yaw])
    orientation = rot_matrix(roll,pitch,yaw)
    g_orientation.append(orientation)


#orientation from accelerometer 
ad = []
for adata in mivals:
    ad.append(adata[:3])

a_orientation = []
acc_rpy = []
accr = []
accp = []
accy = []
a_euler = []

for ti in range(len(ts)):
    aroll, apitch, ayaw = orientation_from_acc(ad,ti)
    accr.append(aroll)
    accp.append(apitch)
    accy.append(ayaw)
    a_euler.append([aroll,apitch,ayaw])
    orientation = rot_matrix(roll,pitch,yaw)
    a_orientation.append(orientation)

    
#complementary filter
alpha = 0.5
falpha = 0.2
cf_orientation = []

roll_a = accr[0]
pitch_a = accp[0]
yaw_a = accy[0]
roll_g = g_roll[0]    
yaw_g = g_yaw[0]
pitch_g = g_pitch[0]
cf_orientation.append([accr[0]+g_roll[0], accp[0] + g_pitch[0], accy[0] + g_yaw[0]])


for i in range(len(ts)-1):
    cf_orientation = cf(cf_orientation,alpha,g_roll,g_pitch,g_yaw,roll_g,pitch_g,yaw_g,accr,accp,accy,roll_a,pitch_a,yaw_a,i,falpha)
    

cfroll = []
cfpitch = []
cfyaw = []
for g in cf_orientation:
    cfroll.append(g[0])
    cfpitch.append(g[1])
    cfyaw.append(g[2])

# Step1: Obtain Sensor Measurements
#converting to orientation to quaternions
a_quat = convert_to_quat(a_euler) #Iat
g_quat = convert_to_quat(g_euler) #Iwt

anorm = []
for A in ad:    
    d = (A[0]**2+A[1]**2+A[2]**2)
    anorm.append([A[0]/d, A[1]/d, A[2]/d])

aqnorm = []
for quater in a_quat:
    aqnorm.append(quat_norm(quater))

gqnorm = []
for gquater in g_quat:
    gqnorm.append(quat_norm(gquater))

beta  = 0.09

qest = []
qest.append(np.array([1,0,0,0]))
qesti = qest[0]

def half_qn(q):
    return [q[0]/2, q[1]/2, q[2]/2, q[3]/2]

for i in range(len(ts) - 1):

    gqest = quat_mult(half_qn(quat_norm(qesti)), [0,gd[i][0],gd[i][1],gd[i][1]])
    
    del_f2 = np.array([[2*(qesti[1]*qesti[3] - qesti[0]*qesti[2]) - ad[i][0]],
              [2*(qesti[0]*qesti[1] + qesti[2]*qesti[3]) - ad[i][1]],
              [2*((1/2) - qesti[1]**2 - qesti[2]**2 ) - ad[i][2]]])
    
    del_f1 = np.array([[-2*qesti[2], 2*qesti[3], -2*qesti[0], 2*qesti[1]],
               [ 2*qesti[1], 2*qesti[0], 2*qesti[3], 2*qesti[2]],
               [ 0, -4*qesti[1],-4*qesti[2], 0]]).T
    if (i == 0):
        print(del_f2)

    del_f = np.squeeze(np.dot(del_f1,del_f2))
    norm_delf = sqrt(del_f[0]**2+del_f[1]**2+del_f[2]**2+del_f[3]**2)
    del_ft = del_f.T
    del_ftn = np.array([del_ft[0]/norm_delf, del_ft[1]/norm_delf, del_ft[2]/norm_delf, del_ft[3]/norm_delf])

    fuse = gqest - beta * (del_ftn)
    qesti = quat_norm(qesti) + fuse*(ts[i+1]-ts[i])
    qest.append((qesti))

oest =[]
for estimated in qest:
    oest.append(quat_to_euler(estimated))

mroll = []
mpitch = []
myaw = []
for g in oest:
    mroll.append(g[0])
    mpitch.append(g[1])
    myaw.append(g[2])

fig, axarr = plt.subplots(3, 1)
axarr[0].plot(ts, g_roll, label = 'gyro', color = 'red')
axarr[0].plot(vts, vicr, label = 'vicon', color = 'blue')
axarr[0].plot(ts, accr, label = 'acc', color = 'green')
axarr[0].plot(ts, cfroll, label = 'cf', color = 'purple')
axarr[0].plot(ts, mroll, label = 'mf', color = 'orange')
axarr[0].set_title('Time (sec) vs Roll (rad)')
axarr[0].legend()
axarr[1].plot(ts, g_pitch, label = 'gyro', color = 'red')
axarr[1].plot(vts, vicp, label = 'vicon', color = 'blue')
axarr[1].plot(ts, accp, label = 'acc', color = 'green')
axarr[1].plot(ts, cfpitch, label = 'cf', color = 'purple')
axarr[1].plot(ts, mpitch, label = 'mf', color = 'orange')
axarr[1].set_title('Time (sec) vs Pitch (rad)')
axarr[1].legend()
axarr[2].plot(ts, g_yaw, label = 'gyro', color = 'red')
axarr[2].plot(vts, vicy, label = 'vicon', color = 'blue')
axarr[2].plot(ts, accp, label = 'acc', color = 'green')
axarr[2].plot(ts, cfyaw, label = 'cf', color = 'purple')
axarr[2].plot(ts, myaw, label = 'mf', color = 'orange')
axarr[2].set_title('Time (sec) vs Yaw (rad)')
axarr[2].legend()
plt.tight_layout()
#plt.legend()
plt.show()
