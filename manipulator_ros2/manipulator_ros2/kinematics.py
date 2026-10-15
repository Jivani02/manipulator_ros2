
import numpy as np

# UR5e standard DH table (lengths in metres)
D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
ALPHA = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]

# DH base frame is turned 180 deg about z compared with the ROS base_link
BASE_FLIP = np.diag([-1.0, -1.0, 1.0, 1.0])


def dh_step(theta, d, a, alpha):
    """4x4 transform from frame k-1 to frame k."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def fk(q):
    """Forward kinematics: six joint angles (rad) -> 4x4 pose of tool0 in base_link."""
    T = np.eye(4)
    for k in range(6):
        T = T @ dh_step(q[k], D[k], A[k], ALPHA[k])
    return BASE_FLIP @ T


if __name__ == '__main__':
    np.set_printoptions(suppress=True, precision=3)
    home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    T = fk(home)
    print('position:', T[:3, 3])
    print('rotation:')
    print(T[:3, :3])