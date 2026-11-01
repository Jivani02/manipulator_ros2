import sys
import numpy as np
import rclpy
try:
    from manipulator_ros2.move_arm import MoveArm
except ImportError:
    from move_arm import MoveArm
try:
    from manipulator_ros2.kinematics import fk, ik_position
except ImportError:
    from kinematics import fk, ik_position

Q_START = np.array([0.5, -1.2, 1.2, -1.6, -1.57, 0.0])
def wrap(q):
    return(np.array(q,dtype=float)+np.pi)%(2*np.pi)-np.pi

def main():
    if len(sys.argv) != 4:
        print('usage: move_to X Y Z (Meters)')
        return

    target = np.array([float(v) for v in sys.argv[1:4]])
    Z_MIN = 0.05
    if target[2] < Z_MIN:
        print(f'target z below {Z_MIN} m: close to the floor, no motion sent')
        return
    print('target:',target)
    result=ik_position(target,Q_START)
    q=result[0] if isinstance(result, tuple) else result
    q=wrap(q)
    print('joint_angles (rad): ',q)\

    err_mm = 1000 * np.linalg.norm(fk(q)[:3, 3] - target)
    print(f'IK error: {err_mm:.3f} mm')

    MAX_ERROR = 1.0
    if err_mm > MAX_ERROR:
        print("target not rechable")
        return
    
    rclpy.init()
    node = MoveArm()
    try: 
        node.go_to([float(v) for v in q], 5)
    
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
