# M20 basic_server bridge

该包把 Route3D 的标准 ROS 2 接口桥接到 M20 `basic_server` 协议。

控制输入：

- `/cmd_vel_smoothed` -> `Type=2, Command=21`
- `/m20/control/motion_state` -> `Type=2, Command=22`
- `/m20/control/usage_mode` -> `Type=1101, Command=5`

状态输出：

- `/m20/state/motion_state`
- `/m20/state/gait`（只读状态，用于速度归一化）
- `/m20/state/motion_info`
- `/m20/state/odometry`（仅诊断）
- `/m20/state/control_result`

bridge 不订阅运动模式切换控制话题，也不会发送 `Type=2, Command=23`。Route3D 的导航
里程计继续使用 `/lio_odom`。

启动：

```bash
ros2 launch basic_server_bridge m20_control.launch.py start_prepare:=true
```

`m20_ready_cmd_vel` 只执行普通使用模式确认、必要的 Stand 和 RL 17 准备，并发布
`/m20/control/ready` 与 `/m20/control/preparation_status`。协议结果或状态未确认时会持续重试。

bridge 对直接输入还会执行 `0.8/0.3/0.5` 的硬限幅、命令超时归零和网络失联归零；
正常导航应让 `route3d_m20_adapter` 先执行更严格的阶段限幅与安全门控。
