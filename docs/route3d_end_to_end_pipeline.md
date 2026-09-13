# Route3D M20 端到端控制链

## 数据流

```text
topoGraph_data.json
  -> route3d_dijkstra_planner
  -> route3d_route_slicer
  -> PID 或 Efficient 3D
  -> route3d_m20_adapter（仲裁、门控、限幅、超时归零）
  -> /cmd_vel_smoothed
  -> basic_server_bridge
  -> M20
```

M20 adapter 是控制器速度的唯一出口。PID 和 Efficient 可以同时常驻，但只有
`/route3d_controller/active_source` 指定的一路能进入 adapter。bridge 不提供运动模式切换
入口，只读取当前状态用于协议速度归一化。

导航定位来自 `/lio_odom` 或 `/lio_odom_hf`；bridge 生成的 `/m20/state/odometry` 只作诊断。

## 准备与 ready 门控

实机启用后，`m20_ready_cmd_vel` 按以下状态机运行：

```text
等待状态 -> 普通使用模式确认 -> Stand（必要时）-> RL 17 -> ready
```

协议拒绝、状态未到达或状态回退都会保持 `ready=false` 并重试。adapter 还独立检查 RL
状态反馈的新鲜度、命令时效、控制源和 `enable_motion`。任一条件失败即持续输出零速度。

## 路线执行

`obstacleMode` 决定控制器与普通点云停障行为。路线任务不含运动模式命令，也没有等待确认
状态。`isSlope` 只保留为地形标注。

最终目标永远拆出姿态对准语义；中间点只有 `alignFinalYaw=true` 才停车对准。若路径段由
Efficient 执行，终点需要对准时会追加单点 PID 任务。PID 先完成位置跟踪并停车，再进入独立
低速 `vx/vy/wz` 对准阶段。

## M20 三维停障

只有模式 0 的 PID 路线启用普通三维点云扫掠。检测流程为：

1. 按点云时间戳插值完整三维 `/lio_odom` 姿态。
2. 过滤 M20 实体范围内自点。
3. 将 `base_link` 点云端点变换到世界坐标；不重复应用雷达外参。
4. 按路线航向和坡度连续放置 `0.92 x 0.53 x 0.57 m` 检测体。
5. 排除路径平面、具有二维邻域支撑的台阶表面及其上方 0.10 m。
6. 发布碰撞点、最近命中距离、MarkerArray 和 JSONL。

模式 1 使用 Efficient 主动绕障；模式 2 和 3 不触发普通点云停障；模式 4 交接外部栅格。
急停、碰撞等级和数据超时不受这些模式影响。

## 启动

```bash
ros2 launch route3d_m20_adapter m20_pid_route.launch.xml \
  graph_file:=/home/wei/github_code/topo_graph_m20_ws/data/79792/topoGraph_data.json \
  enable_motion:=false
```

实机联调前重点观察：

```bash
ros2 topic echo /m20/control/preparation_status
ros2 topic echo /route3d_m20_adapter/status
ros2 topic echo /route3d_m20_adapter/selected_command
ros2 topic echo /route3d_pid_controller/nearest_hit_distance
```

## 离线 bag 高度

现有 `xili`、`regu`、`full_nav_replay_with_plan` 由 0.40 m 高度的机器人录制。它们只用于
复现环境与定位轨迹：构图减去 0.40 m 得到地面，模拟 M20 时再加 0.57 m，因此回放检测
中心相对原录制位姿上移 0.17 m。
