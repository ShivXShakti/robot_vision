import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose

def pose_from_T(T: np.ndarray) -> Pose:
    # T: 4x4
    Rmat = T[:3,:3]
    t = T[:3, 3]

    quat_xyzw = R.from_matrix(Rmat).as_quat()  # returns [x, y, z, w]
    p = Pose()
    p.position.x, p.position.y, p.position.z = t.tolist()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat_xyzw.tolist()
    return p

from tf_transformations import quaternion_from_euler

def rpy_to_quaternion(rpy):
    roll, pitch, yaw = rpy  # radians
    q = quaternion_from_euler(roll, pitch, yaw)  # [x, y, z, w]
    return q

print(f"quuat: {rpy_to_quaternion([0,0,140])}")