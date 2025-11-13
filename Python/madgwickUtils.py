import numpy as np
from scipy.spatial.transform import Rotation as R

def scipyToMadgwick(scipyQuat):
    w = scipyQuat[0]
    x = scipyQuat[1]
    y = scipyQuat[2]
    z = scipyQuat[3]
    madgwickQuat = np.array([x, y, z, w])
    return madgwickQuat

def madgwickToScipy(madgwickQuat):
    x = madgwickQuat[0]
    y = madgwickQuat[1]
    z = madgwickQuat[2]
    w = madgwickQuat[3]
    scipyQuat = np.array([w, x, y, z])
    return scipyQuat

def quatToEulers(quat):
    r = R.from_quat(quat)
    eulers = r.as_euler('YZX', degrees=False)
    return eulers