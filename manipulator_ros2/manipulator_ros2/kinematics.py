
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

def jacobian(q, eps=1e-6):
    """3x6 position Jacobian by finite differences."""
    p0 = fk(q)[:3, 3]
    J = np.zeros((3, 6))
    for i in range(6):
        dq = np.array(q, dtype=float)
        dq[i] += eps
        J[:, i] = (fk(dq)[:3, 3] - p0) / eps
    return J

def ik_position(target, q0, tol=1e-4, max_iter=100, lam=0.05, step_limit=0.3):
    """Find joint angles whose tool position is `target` (x, y, z in base_link, metres)."""
    q = np.array(q0, dtype=float)
    for it in range(max_iter):
        e = target - fk(q)[:3, 3]
        if np.linalg.norm(e) < tol:
            return q, True
        J = jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), e)   # damped step
        n = np.linalg.norm(dq)                                         # length of the step
        if n > step_limit:
            dq = dq * step_limit/n                                  # BLANK: shorten it to step_limit
        q = q + dq
    return q, False                         # did not converge

if __name__ == '__main__':
    np.set_printoptions(suppress=True, precision=3)
    home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    T = fk(home)
    print('position:', T[:3, 3])
    print('rotation:')
    print(T[:3, :3])