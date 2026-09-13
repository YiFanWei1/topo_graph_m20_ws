# M20 实机速度控制

该包提供一条与 Route3D 规控隔离的实机手动速度链路。用户只向下面的话题持续发布：

```text
/m20/manual/cmd_vel    geometry_msgs/msg/Twist
```

控制链：

```text
/m20/manual/cmd_vel
  -> m20_velocity_control（ready、RL 17、急停、超时、限速）
  -> /m20/manual/cmd_vel_safe
  -> basic_server_bridge
  -> M20 Type=2, Command=21
```

## 安全行为

- launch 默认 `enable_motion=false`，只有实机测试时才显式改为 `true`。
- 自动准备只执行普通使用模式确认、必要时 Stand、RL 状态 17，不发送步态切换命令。
- 未 ready、状态不是新鲜的 RL 17、急停或输入超过 0.30 秒未更新时持续输出零速度。
- 默认限速 `|vx|<=0.8 m/s`、`vy=0`、`|wz|<=0.5 rad/s`。
- 如确实需要横移，修改配置中的 `allow_lateral_motion: true` 后，`|vy|<=0.3 m/s`。
- 专用 bridge 只订阅 `/m20/manual/cmd_vel_safe`，不会接收 Route3D 的
  `/cmd_vel_smoothed`。

同一时间只能启动一套 M20 控制栈。启动本包前必须退出 Route3D 实机 launch 和其他
`basic_server_bridge`，但不要关闭遥控器急停能力。首次测试应架空机器人或置于空旷区域，
并使用很低的速度。

## 启动实机

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/10_start_m20_manual_velocity.sh true
```

该脚本会在启动前检查是否已有 Route3D adapter 或 bridge，避免两套控制链同时连接机器人。
等效的 ROS 2 launch 命令是：

```bash
source install/setup.bash
ros2 launch m20_velocity_control m20_manual_control.launch.py enable_motion:=true
```

只做离线话题门控测试、不连接 M20 时使用：

```bash
ros2 launch m20_velocity_control m20_manual_control.launch.py \
  start_bridge:=false enable_motion:=false
```

确认准备完成：

```bash
ros2 topic echo /m20/control/preparation_status
ros2 topic echo /m20/manual/status
```

开始发布速度前必须先看到准备状态中的 `ready=true`，以及手动状态中的
`motion_state=17`。此时因为还没有速度输入，`block_reason=command_stale` 是正常的；开始持续
发布后才应变为 `blocked=false`。

## 发布速度

低速直行，必须连续发布：

```bash
ros2 topic pub -r 10 /m20/manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

低速原地转向：

```bash
ros2 topic pub -r 10 /m20/manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}"
```

发布终端按 `Ctrl+C` 后，0.30 秒内自动归零。也可以明确发布一次零速度：

```bash
ros2 topic pub --once /m20/manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

软件急停与恢复：

```bash
ros2 topic pub --once --qos-durability transient_local \
  /m20/manual/emergency_stop std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once --qos-durability transient_local \
  /m20/manual/emergency_stop std_msgs/msg/Bool "{data: false}"
```

软件急停不能替代遥控器或硬件急停。
