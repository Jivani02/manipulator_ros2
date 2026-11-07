
# manipulator_ros2

A UR5e robot arm in ROS 2 Jazzy and Gazebo. The goal is a simulated arm that sees an object, plans a motion, picks it up and places it, with measured accuracy.

## Status

- [x] 1. UR5e arm in simulation
- [x] 2. Joint control node (`move_arm`)
- [x] 3. Forward kinematics (own DH implementation), validated against the simulator's TF tree
- [x] 4. Inverse kinematics: Jacobian and damped least-squares IK, `move_to x y z` confirmed against TF, accuracy test over a grid of targets
- [x] 5. MoveIt 2 with collision objects: planning and execution through RViz and the planning service, failed goals traced to long joint paths (`check_path`)
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
ros2 run manipulator_ros2 move_to x y z                 # tool position in metres, base_link frame
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

Target: (0.5253, 0.5473, 0.5424) m, the tool position of the test pose.

| Start pose | Resulting joint angles (rad) | Position error |
|---|---|---|
| ready `[0, -1.2, 1.4, -1.8, -1.57, 0]` | `[0.611, -1.032, 0.914, -2.009, -1.435, 0]` | 0.080 mm |
| home `[0, -1.57, 0, -1.57, 0, 0]` | `[0.497, -0.977, 0.828, -1.312, -0.201, 0]` | 0.057 mm |

Limits: only the tool position is controlled (not the orientation), joint 6 stays unchanged, and
the solution found is the one near the start pose (a 6-axis arm usually has several solutions).

## move_to: Cartesian target to arm motion

`move_to x y z` solves the IK from a fixed start pose, checks the result with the own forward
kinematics, and sends the joint angles to the arm only if the position error is below 1 mm.
Targets with z below 0.05 m are refused.

End-to-end test in the simulator (tool position read from `tf2_echo base_link tool0`):

| Target (m) | IK error | Tool position in TF |
|---|---|---|
| (0.4, 0.3, 0.4) | 0.066 mm | (0.400, 0.300, 0.400) |
| (0.4, -0.3, 0.4) | 0.066 mm | (0.400, -0.300, 0.400) |
| (0.5, 0.0, 0.3) | 0.006 mm | (0.500, -0.000, 0.300)|
| (0.3, 0.3, 0.5) | 0.025 mm | (0.300, 0.300, 0.500) |
| (-0.4, 0.3, 0.4) | 0.009 mm | (-0.400, 0.300, 0.400) |
| (0.6, 0.2, 0.2) | 0.033 mm | (0.600, 0.200, 0.200) |

Targets that were refused or exposed limits:

| Target (m) | Result |
|---|---|
| (1.2, 0.0, 0.4) | outside the 0.85 m reach, IK error 256.5 mm, no motion |
| (0.0, 0.0, 0.9) | IK did not converge (stretched pose near a singularity), error 33.7 mm, no motion |
| (0.4, 0.3, -0.1) | IK solved (0.007 mm), but the arm stopped at z = 0.021 m on the ground plane. The IK has no collision checking. This is the reason for step 5 (MoveIt 2 with collision objects). |

## Simulation setup

The arm stands on a table (top 1.2 x 0.9 m, about 0.70 m high). The table is part of the robot description
(`urdf/ur5e_table.urdf.xacro`, visible in RViz) and of the Gazebo world (`worlds/table.sdf`), with the
same dimensions. It is one link with five boxes (top and four legs), and the tabletop is 5 mm below the
arm base. Both choices avoid permanent contacts in MoveIt's collision model: separate touching links
and a base standing directly on the top were reported as collisions at rest. The tabletop is at
z = -0.005 m in `base_link`, so z = 0.1 means about 0.1 m above the table.

## IK accuracy test

`accuracy.py` solves the IK for a grid of targets with the same start pose as `move_to`, checks every
solution with the own forward kinematics, and saves all rows to a CSV file
(`x, y, z, error_mm, iterations, success`). A target counts as reached if the position error is below 1 mm.

| Test | Targets | Reached | Mean error | Max error | Iterations |
|---|---|---|---|---|---|
| Working area: r 0.25 to 0.75 m, z 0.1 to 0.7 m, step 0.1 m | 1092 | 1092 (100 %) | 0.030 mm | 0.100 mm | mean 11, max 30 |
| Wide area: r 0 to 1.0 m, z -0.3 to 1.0 m, step 0.2 m | 567 | 408 (72 %) | 0.034 mm | 0.100 mm | mean 12.9, max 76 |

Results: `results/ik_accuracy.csv` and `results/ik_accuracy_wide.csv`.

The 159 failures in the wide area are consistent with geometric limits of the arm:

- **Outer reach.** Targets at r = 1.0 m fail by 35 mm at the best height (shoulder height) and by more
  at other heights. The tool reaches about 0.96 m horizontally.
- **Base axis.** Targets with x = y = 0 fail with the same error of 33.7 mm at every height. This equals
  d4 - d6 = 0.1333 m - 0.0996 m = 0.0337 m, which is consistent with a small region around the base
  axis that the tool flange cannot reach.
- **Floor and ceiling.** Targets at z = 0.9 m (a few mm above the reachable height) and far targets
  at z = -0.3 m.

Not tested: collisions with the table or with the arm itself (planned with MoveIt 2), and retries from
other start poses.

## MoveIt 2

MoveIt 2 (`ur_moveit_config`) runs on top of the simulation. It loads the robot description from the
`/robot_description` topic, so the table of the simulation is part of its collision model.

```bash
# terminal 1: simulation without its own RViz
ros2 launch manipulator_ros2 sim.launch.py 

# terminal 2: MoveIt with the Motion Planning panel (after the arm stands in Gazebo)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true launch_rviz:=true
```

Checked in RViz: a goal inside reach is planned and executed, and the arm in Gazebo follows. A goal in which
a link touches the table is shown in red and planning fails.

### Goals that fail to plan

Some goals set in RViz (for example "Random Valid") are not planned, although the pose is reachable.
The cause is the joint representation of the goal. The UR5e joints may turn ±360°, so RViz can
choose joint values such as 295° for a pose that is also reached with −65°. OMPL plans in joint space
and has to turn the joint 385° from a start of −90°. This swings the arm through the table or the base,
and the search ends without a path. A longer planning time did not help. A goal inside the table or in a
self-collision is rejected immediately instead.

`check_path.py` reproduces this with MoveIt's planning service `/plan_kinematic_path`. It reads the
current joints from `/joint_states`, takes the six goal joints from the Joints tab of RViz (in degrees),
computes the nearest equivalent joints `q0 + wrap(q1 - q0)` and plans with both goals. If the nearest
goal is planned, it executes the trajectory through the `FollowJointTrajectory` action.

```bash
python3 check_path.py 245 295 72 162 43 -62
```

```text
q0 (current):    [ 0.   -1.57  0.   -1.57  0.    0.  ]
q1 (RViz goal):  [ 4.28  5.15  1.26  2.83  0.75 -1.08]
q1 nearest:      [-2.01 -1.13  1.26 -3.46  0.75 -1.08]
q1n in degrees:  [-115.  -65.   72. -198.   43.  -62.]
RViz goal q1 -> code 99999 , points 0
nearest q1n  -> code 1 , points 14
execution result code: 0
```

The same pose fails with the original joint values (code 99999, `FAILURE`) and is planned with the nearest
joints (14 trajectory points, 0.05 s). The arm reaches it in Gazebo.
## Layout

```
manipulator_ros2/                  repository
  README.md
  results/                         IK accuracy results (CSV)
  manipulator_ros2/                ROS package (package.xml, setup.py, resource/)
    launch/sim.launch.py           simulation with the arm on a table
    urdf/ur5e_table.urdf.xacro     UR5e on a table (robot description)
    worlds/table.sdf               Gazebo world with the table
    manipulator_ros2/              Python module
      move_arm.py                  FollowJointTrajectory client
      move_to.py                   x y z command: IK, check, motion
      kinematics.py                DH forward kinematics, Jacobian, IK
      accuracy.py                  IK accuracy test over a grid of targets
      check_path.py                MoveIt planning service: nearest joints, plan, execute
```
```

