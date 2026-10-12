import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Home pose = the upright pose the arm has after spawning (rad),
# in the same order as JOINTS above.
HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
        )

    def go_to(self, positions, seconds):
        self.get_logger().info('Waiting for the controller...')
        self._client.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=seconds)
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
    rclpy.init()
    node = MoveArm()
    node.go_to(HOME, 5)
    node.destroy_node()
    rclpy.shutdown()