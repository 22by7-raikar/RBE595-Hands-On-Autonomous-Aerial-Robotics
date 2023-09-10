import math

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

def get_quaternion_from_euler(r):
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
    #coder
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qx, qy, qz, qw]