# Go2 gait switch demo

This package performs a safety-limited Go2 demonstration:

1. Switch to the verified normal gait through `StaticWalk()`, rotate with
   `vx=0`, `vy=0`, `wz=0.5 rad/s`, then stop.
2. Switch to stair gait through legacy `SwitchGait(3)` (API 1011), apply the
   same angular command, then stop. If the robot firmware rejects this legacy
   API, the sequence stops before sending the angular command.
3. Repeat the two phases once and issue three final `StopMove()` calls.

Real movement is disabled by default. The node does not call `StandUp()`; place the robot on a
clear, level surface and use the remote controller to make it stand steadily before arming the
test.

```bash
cd /home/wei/github_code/topo_graph_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select go2_gait_switch_demo
source install/setup.bash

# Dry run: no SDK connection and no motion.
ros2 launch go2_gait_switch_demo go2_gait_switch_demo.launch.py

# Real robot; replace eth0 if necessary.
ros2 launch go2_gait_switch_demo go2_gait_switch_demo.launch.py \
  network_interface:=eth0 enable_motion:=true
```

Emergency stop from another terminal:

```bash
ros2 service call /go2_gait_switch_demo/emergency_stop std_srvs/srv/Trigger '{}'
```

`Ctrl+C` also invokes `StopMove()` during node destruction. Keep the physical remote emergency
stop ready because no user-space process can guarantee a stop after a hard power loss, `kill -9`,
network failure, or computer crash.
