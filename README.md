
# manipulator_ros2

A UR5e robot arm in ROS 2 Jazzy and Gazebo. The goal is a simulated arm that sees an object, plans a motion, picks it up and places it, with measured accuracy.

## Status

- [x] 1. UR5e arm in simulation
- [x] 2. Joint control node (`move_arm`)
- [x] 3. Forward kinematics (own DH implementation), validated against the simulator's TF tree
- [x] 4. Inverse kinematics: Jacobian and damped least-squares IK, `move_to x y z` confirmed against TF, accuracy test over a grid of targets
- [x] 5. MoveIt 2 with collision objects: planning and execution through RViz and the planning service, failed goals traced to long joint paths (`check_path`)
- [x] 6. RGB camera cube detection: position and yaw measured, test with several cubes open
- [ ] 7. Pick-and-place demo
- [ ] 8. Write-up

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `ur_simulation_gz` and `ur_description`
- NumPy
- OpenCV for Python (`python3-opencv`)

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

## Camera: cube detection

A fixed RGB camera above the table finds a red cube and gives its position and rotation in the
`base_link` frame. The method is classical computer vision (OpenCV), without machine learning.

Setup: the camera is part of `worlds/table.sdf` (1.0 m above the tabletop, looking straight down,
640 x 480, field of view 1.0 rad). `sim.launch.py` bridges the image and `camera_info` to ROS with
`ros_gz_bridge`. The cube is 5 cm wide.

Method (`detect.py`):

1. HSV threshold for red gives a mask.
2. The largest contour (at least 200 pixels, smaller blobs count as noise) gives the centre (u, v) and
   the angle through `cv2.minAreaRect`.
3. Back-projection with the intrinsics from `camera_info` and the known camera height `Z`:
   `x = CAM_X + (cy - v) * Z / fy` and `y = CAM_Y + (cx - u) * Z / fx`.
4. Yaw is `(90 - angle) mod 90` degrees. A cube looks the same every 90 degrees, so its yaw is only
   known modulo 90 degrees.

### Accuracy test

`cam_accuracy.py` places the cube at known positions with the Gazebo `set_pose` service, waits until it
has settled, detects it in a fresh frame, and compares the result with the true pose. All rows are
saved to a CSV file (`x, y, yaw, measured_x, measured_y, dx_mm, dy_mm, error_mm`, plus `yaw_error_deg`
in the last run). The test points are chosen in the main block: a grid (x 0.35 to 0.75 m, y -0.3 to
0.3 m, step 0.1 m, three yaw angles, 87 placements) or 50 random positions in the same area.

| Test | Placements | Detected | Mean error | Max error |
|---|---|---|---|---|
| Grid, plane at the top face (`Z` = 0.955 m) | 87 | 87 | 4.09 mm | 8.04 mm |
| Grid, `Z` = 0.975 m | 87 | 87 | 1.23 mm | 2.31 mm |
| 50 random positions, `Z` = 0.975 m | 50 | 50 | 1.21 mm | 2.28 mm |

Yaw error on the 50 random positions: mean 0.22 deg, maximum 1.72 deg.

Results: `results/camera_accuracy_top_face.csv`, `results/camera_accuracy.csv` and
`results/camera_accuracy_random.csv`.

### Why `Z` = 0.975 m

With the top-face height the error grew with the distance from the camera axis. The measured
positions were about 2 % too close to the axis (a true y of 0.3 m was measured as 0.2943 m), and the
error was close to 1 mm at the axis and 6 to 8 mm at the edges. This is a scale error: the visible
side face of the cube is part of the blob, so the blob centre lies below the top face, and using the top-face
height under-scales the pixel to metre conversion. A plane about in the middle of the cube (`Z` = 0.975 m)
removes the error. `Z` was fitted on the grid, so the 50 random positions are the validation.
The yaw has no effect on the position (differences below 1 mm).

Limits: one fixed camera and a known object height, so only objects lying on the table can be
located. One pixel is about 1.6 mm at the table. The yaw is known modulo 90 degrees. So far only the
red cube alone was tested, and only in simulation with constant lighting.

```bash
ros2 launch manipulator_ros2 sim.launch.py                  # terminal 1: simulation and camera bridge
python3 detect.py                                           # terminal 2: print the position of the red cube
python3 cam_accuracy.py                                     # terminal 2: accuracy test, writes the CSV
```
Run both scripts from `manipulator_ros2/manipulator_ros2/`.

## Layout

```
manipulator_ros2/                  repository
  README.md
  results/                         IK and camera accuracy results (CSV)
  manipulator_ros2/                ROS package (package.xml, setup.py, resource/)
    launch/sim.launch.py           simulation with the arm on a table, camera bridge
    urdf/ur5e_table.urdf.xacro     UR5e on a table (robot description)
    worlds/table.sdf               Gazebo world with the table, the camera and the cube
    manipulator_ros2/              Python module
      move_arm.py                  FollowJointTrajectory client
      move_to.py                   x y z command: IK, check, motion
      kinematics.py                DH forward kinematics, Jacobian, IK
      accuracy.py                  IK accuracy test over a grid of targets
      check_path.py                MoveIt planning service: nearest joints, plan, execute
      detect.py                    red cube detection from the top camera
      cam_accuracy.py              camera accuracy test (grid or random positions)
```

