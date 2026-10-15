import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')
        self._client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def go_to(self,position,second):
        self.get_logger().info('Waiting for the controller...')
        self._client.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names =JOINTS
        point = JointTrajectoryPoint()
        point.positions = position
        point.time_from_start = Duration(sec=second)
        goal.trajectory.points.append(point)
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Movement finished')
        return True

def main():
    args = [float(a) for a in sys.argv[1:]]
    if len(args) not in (0, 6):
        print('Give no values (home pose) or exactly 6 joint angles in radians.')
        return
    target = args if args else HOME
    rclpy.init()
    node = MoveArm()
    node.go_to(target, 5)
    node.destroy_node()
    rclpy.shutdown()