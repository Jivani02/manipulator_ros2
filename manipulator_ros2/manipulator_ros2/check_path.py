import sys
import numpy as np
try:
    from manipulator_ros2.kinematics import ik_position
except ImportError:
    from kinematics import ik_position
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState



JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class PathChecker(Node):
    def __init__(self):
        super().__init__('path_check')
        self.msg = None
        self.create_subscription(JointState, '/joint_states', self.on_js,10)

    def on_js(self, msg):
        self.msg = msg

    def current_q(self):
        while self.msg is None:
            rclpy.spin_once(self, timeout_sec = 0.5)
        pos = dict(zip(self.msg.name,self.msg.position))
        return np.array ([pos [j] for j in JOINTS])


def main():
    q1 = np.radians([float(v) for v in sys.argv[1:7]])
    rclpy.init()
    nod = PathChecker()
    try:
        q0 = nod.current_q()
        dq = np.arctan2(np.sin(q1 - q0), np.cos(q1 - q0))
        q1n = q0 + dq
        print('q0 (current):', np.round(q0, 2))
        print('q1 (RViz goal):', np.round(q1, 2))
        print('q1 - q0:', np.round(q1 - q0, 2))
        print('q1 nearest:', np.round(q1n, 2))
        print('q1n - q0:', np.round(q1n - q0, 2))
        print('q1n in degrees:', np.round(np.degrees(q1n), 1))
    finally:
        nod.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()