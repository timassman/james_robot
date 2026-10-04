# james_robot
ROS2 bringup and description for James robot

| Package | Contents |
|---|---|
| `james_bringup` | Launch files to start the robot (arm, Roomba, sensors) |

Planned: `james_description` — URDF of the robot itself (Roomba, crate, arm, camera).

## Repositories

The split follows the TurtleBot3 repositories:

| Repository | Contents | Runs on | TurtleBot3 equivalent |
|---|---|---|---|
| [james_robot](https://github.com/timassman/james_robot) | Bringup and robot description | Jetson + laptop | `turtlebot3` |
| [james_perception](https://github.com/timassman/james_perception) | Bottle detection in the crate | Jetson | — |
| [james_simulations](https://github.com/timassman/james_simulations) | Gazebo models and worlds | Laptop only | `turtlebot3_simulations` |
| [james_docker](https://github.com/timassman/james_docker) | Docker image, container, Jetson setup | Jetson + laptop | — |

The container, devices and daily startup of the Jetson are described in the README of
james_docker.

---

## Robot arm (OpenManipulator-X)

Uses the official ROBOTIS packages (`open_manipulator_x_*`, humble branch) that are built
into the Docker image. `arm.launch.py` starts the hardware interface with its controllers
and MoveIt `move_group` on the Jetson; RViz runs on the laptop.

### Step by step

Prerequisite: the Jetson is on, its clock is synchronized and the container is running —
see "Daily startup" in the [james_docker](https://github.com/timassman/james_docker) README.

**1. Jetson, container terminal 1 — start the arm**

Power the arm first: 12 V supply + switch on the controller board.

```bash
docker exec -it robojames bash
# Testing a synced version? Build and source it first, see Development workflow below
ros2 launch james_bringup arm.launch.py
```

Expected output: five Dynamixels (ID 11–15, XM430-W350) answer the ping, `Dynamixel Hardware
Start!`, three controllers activated and `You can start planning now!`.
At startup the arm does not move: the current position is taken as the target before the
torque is enabled.

**2. Jetson, container terminal 2 — check**

```bash
docker exec -it robojames bash
ros2 topic hz /joint_states                  # ~100 Hz
ros2 param get /move_group use_sim_time      # Boolean value is: False
```

**3. Laptop — RViz**

Needs a workspace with the ROBOTIS humble packages (for the meshes and MoveIt config),
e.g. `~/ros2_ws3`:

```bash
ros2 daemon stop                             # clear cached discovery
source ~/ros2_ws3/install/setup.bash
ros2 topic hz /joint_states                  # ~100 Hz = DDS connection to the Jetson works
echo "jetson: $(ssh jetson-orin date +%s)  laptop: $(date +%s)"   # should differ by 0-1 s
ros2 launch open_manipulator_x_moveit_config moveit_rviz.launch.py
```

MotionPlanning panel → Planning tab:
1. Planning Group `gripper` → Goal State `open`, Plan & Execute → `close`, Plan & Execute
   (safe first move: the base does not rotate)
2. Planning Group `arm` → Goal State `home` → check the preview → Plan & Execute
3. Planning Group `arm` → Goal State `init` (folded) → Plan & Execute

**4. Stop — warning**

Stopping `arm.launch.py` (Ctrl+C, closing the terminal, a dropped SSH/VS Code connection)
**disables the torque and the arm drops**. First move the arm to `init` (folded) or
support it by hand. Run the launch file inside `tmux` to survive connection drops
(see "Keep processes running with tmux" in the james_docker README).

### Launch options

| Option | Effect |
|---|---|
| `moveit:=false` | Hardware and controllers only, no `move_group` |
| `use_fake_hardware:=true` | Simulated arm, no hardware needed (testing on the laptop) |
| `port_name:=/dev/ttyUSB0` | Other serial port (default `/dev/openmanipulator`, set by `run.sh`) |

### Test without hardware (laptop)

```bash
source ~/ros2_ws3/install/setup.bash
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 ros2 launch james_bringup arm.launch.py use_fake_hardware:=true
```

The separate domain and localhost-only setting keep the simulated arm from mixing with
the real robot on the network. Use the same two variables for RViz in that test.

### Troubleshooting

**MoveIt: "Joint 'joint2' from the starting state is outside bounds", nothing moves**

The arm is outside the URDF joint limits (e.g. `joint2` > 1.5 rad after it dropped).
MoveIt cannot plan from an invalid start state. Either:

- Support the arm, stop `arm.launch.py`, move the arm upright by hand and start again, or
- Move it back into range with the controller directly (it does not check URDF limits):

```bash
ros2 topic echo /joint_states --once       # note joint1..joint4 (match by name)
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4],
    points: [{positions: [J1, 1.3, J3, J4], time_from_start: {sec: 4}}]}}"
```

Replace `J1`, `J3`, `J4` with the current values; only `joint2` moves.

**Harmless messages**

- `FastSyncRead Rx Fail ... -3001` once at startup — a single read timeout, the driver only
  fails if errors persist for 500 ms
- `Could not enable FIFO RT scheduling` — the container has no realtime permission;
  `/joint_states` still runs at a stable 100 Hz
- `error_timeout_ms parameter not found` — parameter name mismatch between ROBOTIS packages
- Octomap `No 3D sensor plugin(s) defined` — no camera is connected to MoveIt (yet)

---

## Development workflow

Edit on the laptop, sync to the Jetson and build inside the container. How the sync and
the `~/git` mount work is described in Scenario H of the
[james_docker](https://github.com/timassman/james_docker) README.

```bash
# On the laptop (from james_docker):
./sync_to_jetson.sh james_robot

# Inside the container:
cd ~/git/james_robot
colcon build --symlink-install --packages-select james_bringup
source install/local_setup.bash
ros2 pkg prefix james_bringup     # expect /home/james/git/james_robot/install/james_bringup
```

The `source` applies to that terminal only; without it, `ros2 launch` uses the older
`james_bringup` built into the image.

## Known issues

- `james.launch.py` still includes `rtabmap_kinect_xbox_360.launch.py`, which does not
  exist (old Kinect setup). Start the Roomba directly with `create_bringup` instead.
