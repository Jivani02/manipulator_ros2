# manipulator_ros2

A UR5e manipulator in ROS 2 Jazzy and Gazebo Sim, built step by step: simulation, joint control, kinematics, planning, perception and pick and place, with measured accuracy at every stage.

The goal is a simulated arm that sees an object, plans a motion, picks it and places it, with the error of each stage measured and documented.

Related project: [mobile_robot_ros2](https://github.com/Jivani02/mobile_robot_ros2) (mobile robot with SLAM, Nav2 and odometry calibration).

## Status

- [x] 1. UR5e in Gazebo Sim with RViz, started from one launch file
- [x] 2. Joint control node (`move_arm`): send the arm to any joint pose
- [x] 3. Forward kinematics from the DH table, validated against the simulator's TF
- [ ] 4. Inverse kinematics and an accuracy comparison tool
- [ ] 5. MoveIt 2 planning with collision objects
- [ ] 6. Object detection with a stereo camera
- [ ] 7. Pick and place demo
- [ ] 8. Write-up with measured results

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- Universal Robots ROS 2 packages: `ur_simulation_gz`, `ur_description`
- Python 3 with NumPy

## Build

```
cd ~/ros2_ws/src
git clone https://github.com/Jivani02/manipulator_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the simulation (Gazebo Sim and RViz):

```
ros2 launch manipulator_ros2 sim.launch.py
```

Move the arm in a second terminal. Without values it goes to the home pose (upright); with six joint angles in radians it goes to that pose in 5 seconds:

```
ros2 run manipulator_ros2 move_arm
ros2 run manipulator_ros2 move_arm 0.5 -1.0 1.0 -1.5 0.3 0.2
```

The node sends a `FollowJointTrajectory` goal to `/scaled_joint_trajectory_controller/follow_joint_trajectory`. Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.

## Forward kinematics

`manipulator_ros2/kinematics.py` computes the pose of `tool0` from six joint angles with the standard DH convention. The function `fk(q)` multiplies the six joint transforms and returns a 4x4 matrix in the `base_link` frame.

UR5e DH table (lengths in metres):

| Joint | alpha | a | d |
|---|---|---|---|
| 1 | 90 deg | 0 | 0.1625 |
| 2 | 0 | -0.425 | 0 |
| 3 | 0 | -0.3922 | 0 |
| 4 | 90 deg | 0 | 0.1333 |
| 5 | -90 deg | 0 | 0.0997 |
| 6 | 0 | 0 | 0.0996 |

The joint angle is theta for each joint (all offsets are zero).

**Frames.** The DH chain gives the pose in the UR base frame (`base`). In the robot model, `base_link` is that frame turned by 180 degrees about z, so `fk` applies this rotation at the end. I checked this with `ros2 run tf2_ros tf2_echo base_link base`, which shows a pure 180 degree yaw with no translation.

### Validation against the simulator

I compared `fk(q)` with the pose that TF reports (`tf2_echo base_link tool0`) after moving the simulated arm.

| Pose (joint angles in rad) | TF position (m) | `fk` position (m) |
|---|---|---|
| 0, -1.57, 0, -1.57, 0, 0 (home) | 0.001, 0.233, 1.079 | 0.001, 0.233, 1.079 |
| 0.5, -1.0, 1.0, -1.5, 0.3, 0.2 | 0.525, 0.547, 0.542 | 0.525, 0.547, 0.542 |

The rotation matrices agree as well (three decimals). The next step is an automatic tool that does this comparison for many poses and reports the error in millimetres.

## Repository layout

```
manipulator_ros2/            repository
  README.md
  manipulator_ros2/          ROS 2 package
    package.xml, setup.py
    launch/sim.launch.py     starts Gazebo Sim and RViz
    manipulator_ros2/
      move_arm.py            joint control node
      kinematics.py          DH forward kinematics
```

## Author

Hirenbhai Jivani, M.Eng. Mechatronics and Cyber-Physical Systems, Deggendorf Institute of Technology
GitHub: [Jivani02](https://github.com/Jivani02)
