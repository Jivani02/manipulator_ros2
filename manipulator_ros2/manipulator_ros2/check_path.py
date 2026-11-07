import sys
import numpy as np
try:
    from manipulator_ros2.kinematics import ik_position
except ImportError:
    from kinematics import ik_position
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory



JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class PathChecker(Node):
    def __init__(self):
        super().__init__('path_check')
        self.msg = None
        self.create_subscription(JointState, '/joint_states', self.on_js,10)
        self.plan_client = self.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def on_js(self, msg):
        self.msg = msg

    def current_q(self):
        while self.msg is None:
            rclpy.spin_once(self, timeout_sec = 0.5)
        pos = dict(zip(self.msg.name,self.msg.position))
        return np.array ([pos [j] for j in JOINTS])

    def build_request(self, q0, q1n):
         req = GetMotionPlan.Request()
         mpr = req.motion_plan_request
         mpr.group_name = 'ur_manipulator'
         mpr.num_planning_attempts = 5
         mpr.allowed_planning_time = 5.0
         mpr.start_state.joint_state.name = JOINTS
         mpr.start_state.joint_state.position = [float(v) for v in q0]
         c = Constraints()
         for name, val in zip(JOINTS, q1n):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
         mpr.goal_constraints.append(c)
         return req

    def plan(self, req):
        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result().motion_plan_response
        return res

    def execute(self, traj):
        self.traj_client.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        send = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            print('goal rejected')
            return False
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        print('execution result code:', done.result().result.error_code)
        return True

def main():
    q1 = np.radians([float(v) for v in sys.argv[1:7]])
    rclpy.init()
    nod = PathChecker()
    try:
        nod.plan_client.wait_for_service()
        print('planning service ready')
        q0 = nod.current_q()
        dq = np.arctan2(np.sin(q1 - q0), np.cos(q1 - q0))
        q1n = q0 + dq
        print('q0 (current):', np.round(q0, 2))
        print('q1 (RViz goal):', np.round(q1, 2))
        print('q1 nearest:', np.round(q1n, 2))
        print('q1n in degrees:', np.round(np.degrees(q1n), 1))
        code_ok = False
        traj = None
        for label, goal in (('RViz goal q1', q1), ('nearest q1n', q1n)):
            req = nod.build_request(q0, goal)
            res = nod.plan(req)
            print(label, '-> code', res.error_code.val, ', points', len(res.trajectory.joint_trajectory.points))
            if label == 'nearest q1n':
                code_ok = res.error_code.val == 1
                traj = res.trajectory.joint_trajectory
        if code_ok:
            nod.execute(traj)
    finally:
        nod.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()