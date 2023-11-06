import math
import numpy as np

def acc_conv(acc, b, s):
    return (float(acc*s +b)*9.81)  


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
    # Check if the input quaternion is a zero quaternion
    
    d = np.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)

    if d != 0:
        return np.array([q[0] / d, q[1] / d, q[2] / d, q[3] / d])
    
    else:
        # Handle the case when d is zero (shouldn't normally happen if the input is valid)
        return np.array([1.0, 0.0, 0.0, 0.0])

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

    pitch = math.degrees(pitch)
    # Calculate yaw (rotation around the z-axis)
    sin_yaw_cosp = 2.0 * (w * z + x * y)
    cos_yaw_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(sin_yaw_cosp, cos_yaw_cosp))

    # Calculate roll (rotation around the y-axis)
    sin_roll = 2.0 * (w * x - y * z)

    if abs(sin_roll) >= 1:
        roll = math.pi / 2 if sin_roll > 0 else -math.pi / 2
    else:
        roll = math.asin(sin_roll)

    roll = math.degrees(roll)
    return [roll, pitch, yaw]
    # phi = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
    # theta = math.asin(2*(q[0]*q[2] - q[3]*q[1]))
    # psi = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
    
    # return [phi, theta, psi]

def half_qn(q):
    return [q[0]/2, q[1]/2, q[2]/2, q[3]/2]

def quat_mult(x,y):
    # print("this is x and y:", x , "and", y)
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

def quat_vec2quat(vec):
    # Calculate the squared magnitude of the vector

    norm_sq = np.sqrt(np.sum(vec**2))
    
    # Check if the magnitude is zero or contains NaN/Inf values
    if norm_sq == 0 or np.isnan(norm_sq) or np.isinf(norm_sq):
        return np.array([1.0, 0.0, 0.0, 0.0])
    
    # Calculate the magnitude
    angle = norm_sq
    
    # Normalize the vector
    ev = vec / angle

    q = np.array([np.cos(angle / 2), np.sin(angle / 2) * ev[0], np.sin(angle / 2) * ev[1], np.sin(angle / 2) * ev[2]])
    
    return quat_norm(q)

def omega_vec2quat(vec, dt):
    
    norm_sq = vec[0]**2 + vec[1]**2 + vec[2]**2
    
    # Check if the squared norm is zero or contains NaN/Inf values
    if norm_sq == 0 or np.isnan(norm_sq) or np.isinf(norm_sq):
        return np.array([1.0, 0.0, 0.0, 0.0])

    norm = np.sqrt(norm_sq)
    del_angle = norm * dt
    ev = vec / norm

    q = np.array([np.cos(del_angle / 2), np.sin(del_angle / 2) * ev[0], np.sin(del_angle / 2) * ev[1], np.sin(del_angle / 2) * ev[2]])

    # Handle NaN and Inf values by setting them to 0
    q[np.isnan(q)] = 0.0
    q[np.isinf(q)] = 0.0

    return q


def quat2vec(q):

    vec_norm = np.sqrt(q[1]**2+q[2]**2+q[3]**2)
    scal = q[0]

    angle = np.arctan2(vec_norm,scal)

    if vec_norm == 0:
        return np.array([0,0,0])
    
    axis_x = q[1] / vec_norm
    axis_y = q[2] / vec_norm
    axis_z = q[3] / vec_norm   

    axis_x *= angle*2
    axis_y *= angle*2
    axis_z *= angle*2

    magnaxis = np.array([axis_x, axis_y, axis_z])

    return magnaxis


def quat_inv(q):
    norm_sq = np.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)

    if norm_sq == 0:
        q_inv = [1, 0, 0, 0]
    else:
        norm_inv = 1.0 / norm_sq
        q_inv = [q[0] * norm_inv, -q[1] * norm_inv, -q[2] * norm_inv, -q[3] * norm_inv]

    return q_inv


def axis_angle2quat(a):

    q = [0.0] * 4
    angle = np.linalg.norm(a)
    
    if angle != 0:
        axis = a/angle
    
    else:
        q = [1.0, 0.0, 0.0, 0.0]
        return q

    q[0] = math.cos(angle/2)
    q[1:4] = axis*math.sin(angle/2)
    return q


def intrinsic_gd(q, qmean):
    # print("INTRINSIC GD")
    err = []
    axis =[]
    e = np.inf
    itr = 0 
    threshold = 0.01

    while True:

        for v in q:    

            e = quat_mult(quat_norm(v),quat_inv(qmean))

            err.append(quat2vec(e))

        mean_err = np.mean(err, axis = 0)

        mean_err_quat = axis_angle2quat(mean_err)
        qmean = quat_mult(quat_norm(mean_err_quat), quat_norm(qmean[:4]))

        mean_err_mag = np.sqrt(mean_err[0]**2+mean_err[1]**2+mean_err[2]**2)    

        if abs(mean_err_mag) < threshold or itr < 500:
            break
        itr+=1
    
    return qmean, err


def axis_to_quat(axis, angle=None):
    # axis: 3x1
    # angle: scalar (optional)
    
    if angle is None:
        angle = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)

        if angle == 0.0:
            return [1.0, 0.0, 0.0, 0.0]
            
        axis = [x / angle for x in axis]
    
    q0 = np.cos(angle / 2)
    magnitude = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    
    if magnitude == 0:
        return [1.0, 0.0, 0.0, 0.0]
    
    axis = [x / magnitude for x in axis]

    q_vec = np.sin(angle / 2) * np.array(axis)
    q = [q0, q_vec[0], q_vec[1], q_vec[2]]
    q = quat_norm(q)
    return q

        












            
