import numpy as np
from scipy.spatial.transform import Rotation as Rot


def pose_4x4_to_pos_quat(pose):
    """
    Convert pose, 4x4 format into pos and quat

    Args:
        pose: numpy array, 4x4
    Returns:
        pos: length-3 position
        quat: length-4 quaternion (w, x, y, z)

    """
    mat = pose[:3, :3]
    r = Rot.from_matrix(mat)
    quat = r.as_quat()
    pos = np.zeros([3])
    pos[0] = pose[0, 3]
    pos[1] = pose[1, 3]
    pos[2] = pose[2, 3]
    return pos, quat
