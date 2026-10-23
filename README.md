cd ~/ros2_ws/src/manipulator_ros2
cp README.md ~/README_backup.md
cat > README.md << 'EOF'
# manipulator_ros2

A UR5e robot arm in ROS 2 Jazzy and Gazebo. The goal is a simulated arm that sees an object, plans a motion, picks it up and places it, with measured accuracy.

## Status

- [x] 1. UR5e arm in simulation
- [x] 2. Joint control node (`move_arm`)
- [x] 3. Forward kinematics (own DH implementation), validated against the simulator's TF tree
- [ ] 4. Inverse kinematics: Jacobian and damped least-squares IK for the tool position are done; the `move_to x y z` command is in progress; an accuracy comparison tool is planned
- [ ] 5. MoveIt 2 with collision objects
- [ ] 6. Stereo camera object detection
- [ ] 7. Pick-and-place demo
- [ ] 8. Write-up

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `ur_simulation_gz` and `ur_description`
- NumPy

## Build and run

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# terminal 1: simulation
ros2 launch manipulator_ros2 sim.launch.py

# terminal 2: move the arm
ros2 run manipulator_ros2 move_arm                      # home pose
ros2 run manipulator_ros2 move_arm q1 q2 q3 q4 q5 q6    # six joint angles in radians, 5 s
```

`move_arm` is an action client for `FollowJointTrajectory`
(`/scaled_joint_trajectory_controller/follow_joint_trajectory`).

## Forward kinematics

`kinematics.py` computes the pose of `tool0` in `base_link` from six joint angles with the standard
Denavit-Hartenberg convention, `T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`.

| Joint | a (m) | d (m) | alpha |
|---|---|---|---|
| 1 | 0 | 0.1625 | 90 deg |
| 2 | -0.425 | 0 | 0 |
| 3 | -0.3922 | 0 | 0 |
| 4 | 0 | 0.1333 | 90 deg |
| 5 | 0 | 0.0997 | -90 deg |
| 6 | 0 | 0.0996 | 0 |

The negative `a` values come from the UR convention, where the x axes point against the arm direction.

### Frames

The robot description has `base_link` (the ROS root) and a fixed joint to `base`, a pure 180 degree yaw
without translation. `base` is the base frame of the DH convention. The raw DH chain therefore gives
the pose in `base`, and a constant 180 degree rotation about z (`BASE_FLIP`) converts it to `base_link`.
This was checked with `ros2 run tf2_ros tf2_echo base tool0` and `... base_link tool0`.

### Validation against TF

| Pose (joint angles in rad) | Position (m) | Result |
|---|---|---|
| home `[0, -1.57, 0, -1.57, 0, 0]` | (0.001, 0.233, 1.079) | matches TF to 3 decimals |
| test `[0.5, -1.0, 1.0, -1.5, 0.3, 0.2]` | (0.525, 0.547, 0.542) | position and rotation match TF to 3 decimals |

## Inverse kinematics

Position-only, numerical IK (`ik_position` in `kinematics.py`):

1. `e = target - fk(q)` is the position error of the tool.
2. `jacobian(q)` is the 3x6 position Jacobian, computed by finite differences. Column i is how the tool
   moves per radian of joint i.
3. The joint step is the damped least-squares solution
   `dq = J^T (J J^T + lambda^2 I)^-1 e` with `lambda = 0.05`, limited to 0.3 rad per iteration.
4. Repeat until the error is below 0.1 mm (at most 100 iterations).

**Why damping and a step limit.** The home pose (arm stretched upright) is a singular pose: the tool
can hardly move in y and z. With a plain pseudo-inverse, the IK reached the target from home
(error 0.09 mm) but with joint angles of about 3000 rad. With damping and the step limit, the same
target is reached with all joint angles below 2 rad.

|new = """Target: (0.5253, 0.5473, 0.5424) m, the tool position of the test pose.

| Start pose | Resulting joint angles (rad) | Position error |
|---|---|---|
| ready `[0, -1.2, 1.4, -1.8, -1.57, 0]` | `[0.611, -1.032, 0.914, -2.009, -1.435, 0]` | 0.080 mm |
| home `[0, -1.57, 0, -1.57, 0, 0]` | `[0.497, -0.977, 0.828, -1.312, -0.201, 0]` | 0.057 mm |

Limits: only the tool position is controlled (not the orientation), joint 6 stays unchanged, and
the solution found is the one near the start pose (a 6-axis arm usually has several solutions).

## Layout

```
manipulator_ros2/                  repository
  README.md
  manipulator_ros2/                ROS package (package.xml, setup.py, launch/, resource/)
    manipulator_ros2/              Python module
      move_arm.py                  FollowJointTrajectory client
      kinematics.py                DH forward kinematics, Jacobian, IK
```
EOF
git status --short
