# robot_teach_replay

Teach & replay tool for gamepad-driven trajectories on HARP2, with a
`nav2_collision_monitor` safety overlay that **auto-pauses the replay** when
an obstacle gets too close to the robot.

## Overview

This package builds on top of the **safety_layer** shipped by `robot_bringup`,
which inserts a `nav2_collision_monitor` between every command source and the
controller. The teach/replay launches simply reuse that layer.

```
TEACH:
  gamepad -> teleop_node -> /cmd_vel ──> [safety_layer] ──> /omnidirectional_controller/cmd_vel_safety_unstamped -> controller
                                |
                                '-> ros2 bag record  (/cmd_vel)

REPLAY:
  ros2 bag play (--start-paused) -> /cmd_vel ──> [safety_layer] ──> /omnidirectional_controller/cmd_vel_safety_unstamped -> controller
                                                                              |
                                                                              '-> /collision_monitor_state -> safety_supervisor -> pause/resume bag
```

`[safety_layer]` is the collision_monitor (default) or a transparent relay if
`safety_enabled:=False`. See `robot_bringup/launch/safety_layer.launch.py`.

The bag stores only `/cmd_vel` (teleop intent). The `/scan`, `/odom` and `/tf`
streams stay **live** during replay so the safety filter reacts to the actual
environment, not the recorded one.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup robot_controller robot_teach_replay --symlink-install
source install/setup.bash
```

The first time you build, also include `robot_bringup` and `robot_controller`
since both were modified to support the safety layer.

## Record a trajectory (teach)

On the robot:

```bash
ros2 launch robot_teach_replay teach.launch.py launch_on_robot:=True
```

Off the robot (sim or robot already running its own micro-ROS agent):

```bash
ros2 launch robot_teach_replay teach.launch.py
```

Then drive the robot with the gamepad. Stop the launch with `Ctrl+C` when done.

The bag is written to:

```
~/ros2_ws/recorded_trajectories/teach_<YYYYMMDD_HHMMSS>/
```

You can override the output path:

```bash
ros2 launch robot_teach_replay teach.launch.py bag_path:=/tmp/my_run
```

## Replay a trajectory

```bash
ros2 launch robot_teach_replay replay.launch.py \
    bag:=$HOME/ros2_ws/recorded_trajectories/teach_20260513_193500 \
    launch_on_robot:=True
```

The bag starts **paused** and is resumed by `safety_supervisor` once it is
connected and the front of the robot is clear. While replaying:

- If an obstacle enters the `ApproachCircle`, `collision_monitor` reduces
  `cmd_vel` AND the supervisor pauses the bag, so the playback head freezes
  at the current command. No trajectory time is lost while waiting.
- If an obstacle enters the `StopCircle`, `cmd_vel` is forced to zero
  (hard stop) and the bag stays paused.
- When the obstacle clears, the bag resumes from where it stopped.

## Disable the safety filter (transparent mode)

Both launches accept `safety_enabled:=False` to bypass collision_monitor:

```bash
ros2 launch robot_teach_replay teach.launch.py launch_on_robot:=True safety_enabled:=False
ros2 launch robot_teach_replay replay.launch.py bag:=<path> launch_on_robot:=True safety_enabled:=False
```

In transparent mode, `/cmd_vel` is relayed unchanged to
`/omnidirectional_controller/cmd_vel_safety_unstamped`. **No obstacle
detection** in this mode — only use it for calibration or when you are sure
the area is clear.

## Safety zones (parameters)

Defined in `robot_bringup/params/collision_monitor.yaml` (shared by all
launches that include the safety layer):

| Zone | `radius` | Diameter | Action | Behavior |
|---|---|---|---|---|
| `StopCircle` | 0.20 m | 40 cm | `stop` | Forces `cmd_vel = 0` if any scan point falls inside. Last-line of defense. |
| `ApproachCircle` | 0.40 m | 80 cm | `approach` | Projects the current `cmd_vel` `time_before_collision` seconds into the future. If the projection collides with an obstacle, `cmd_vel` is scaled down so the contact would happen exactly at that time horizon, not before. |

The robot is ~35 cm in diameter (radius 17.5 cm), so the stop zone only
extends ~2.5 cm past the physical footprint, and the approach zone extends
~22.5 cm past it.

Tune these values by editing the YAML. With `--symlink-install`, no rebuild
is needed; just relaunch.

### Why this combo emulates a dynamic oval zone

`StopCircle` is geometrically a circle. `ApproachCircle` is also a circle in
shape, but the `approach` action looks ahead along the current `cmd_vel`
direction. The effective braking region therefore stretches in the direction
of motion: moving forward triggers braking on obstacles ahead but not on the
ones at the side, etc. Direction tracking is automatic for any holonomic
heading.

## RViz visualization

Add these displays:

- `Polygon` on `/collision_monitor/stop_circle` (red, recommended)
- `Polygon` on `/collision_monitor/approach_circle` (yellow/orange)
- `LaserScan` on `/scan`

You will see two static circles around the robot in RViz. The dynamic aspect
(direction-of-motion-aware braking) happens internally and is not visible
directly — listen to `/collision_monitor_state` to observe action transitions:

```bash
ros2 topic echo /collision_monitor_state
```

## Troubleshooting

- **The bag never starts playing.** Check `safety_supervisor` logs. It must
  print `rosbag2 player services connected` shortly after start. If not,
  the `/rosbag2_player/pause` and `/resume` services are missing — the bag
  player did not come up. Look at the `ros2 bag play` process output.
- **The robot keeps stopping for no reason.** The lidar may be seeing parts
  of the robot itself or close walls. Verify `/scan` in RViz and tune the
  zone radii in the YAML.
- **The bag plays but the robot does not move.** Check that
  `collision_monitor` is activated (lifecycle state `active`). The
  `lifecycle_manager_collision_monitor` in the launch handles that
  automatically; check its logs.
- **You see commands on `/cmd_vel` but nothing on
  `/omnidirectional_controller/cmd_vel_safety_unstamped`.** Either
  `collision_monitor` is not active, or you ran with `safety_enabled:=False`
  but the `topic_tools/relay` failed to start. Check the launch output.

## Files

- [`launch/teach.launch.py`](launch/teach.launch.py) — includes `gamepad_control.launch.py` (which already brings the safety layer) and adds `ros2 bag record` on `/cmd_vel`.
- [`launch/replay.launch.py`](launch/replay.launch.py) — robot description, controller, safety_layer, `ros2 bag play --start-paused`, safety_supervisor.
- [`robot_teach_replay/safety_supervisor.py`](robot_teach_replay/safety_supervisor.py) — bridges `/collision_monitor_state` to the rosbag2 player pause/resume services.

The actual safety zones config lives in `robot_bringup/params/collision_monitor.yaml`.
