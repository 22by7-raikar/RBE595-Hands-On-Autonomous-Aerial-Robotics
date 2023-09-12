import math
import numpy as np

def acc_conv(acc, b, s):
    return (float(acc+b)/float(s))  


def ang_conv(angle, b):
    return (3300/1023) * (math.pi/180) * 0.3 * (angle - b)

def rpy2rotmat(r,p,y):

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


def conv2ER(curr_rot):

    sy = math.sqrt(curr_rot[0, 0] ** 2 + curr_rot[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(curr_rot[2, 1], curr_rot[2, 2])
        pitch = math.atan2(-curr_rot[2, 0], sy)
        yaw = math.atan2(curr_rot[1, 0], curr_rot[0, 0])
        
    else:
        roll = math.atan2(-curr_rot[1, 2], curr_rot[1, 1])
        pitch = math.atan2(-curr_rot[2, 0], sy)
        yaw = 0

    return roll, pitch, yaw

def euler2quat(r):
    """
    Convert an Euler angle to a quaternion.
    https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
    
    Imathut
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """

    roll, pitch, yaw = r[0], r[1], r[2]
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qx, qy, qz, qw]

def quat_norm(q):

    d = math.sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2)
    return [q[0]/d, q[1]/d, q[2]/d, q[3]/d]

def conv2quat(euler):

    quat = []

    for e in euler:
        cr = math.cos(e[0] * 0.5)
        sr = math.sin(e[0] * 0.5)
        cp = math.cos(e[1] * 0.5)
        sp = math.sin(e[1] * 0.5)
        cy = math.cos(e[2] * 0.5)
        sy = math.sin(e[2] * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        quat.append([qw, qx, qy, qz])

    return quat 

def quat2euler(q):

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


def half_qn(q):
    return [q[0]/2, q[1]/2, q[2]/2, q[3]/2]

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

def vec2quat(vec):
    # Compute the angle and normalized axis of rotation
    angle = np.sqrt(np.sum(vec**2))
    ev = vec / angle

    # Compute the quaternion
    q = np.array([np.cos(angle / 2), np.sin(angle / 2) * ev[0], np.sin(angle / 2) * ev[1], np.sin(angle / 2) * ev[2]])
    
    # Handle NaN and Inf values by setting them to 0
    q[np.isnan(q)] = 0
    q[np.isinf(q)] = 0
    
    return q

def vec2quat2(vec,dt):
    # Compute the angle and normalized axis of rotation
    del_angle = np.sqrt(np.sum(vec**2))*dt
    ev = vec / np.sqrt(np.sum(vec**2))

    # Compute the quaternion
    q = np.array([np.cos(angle / 2), np.sin(angle / 2) * ev[0], np.sin(angle / 2) * ev[1], np.sin(angle / 2) * ev[2]])
    
    # Handle NaN and Inf values by setting them to 0
    q[np.isnan(q)] = 0
    q[np.isinf(q)] = 0
    
    return q