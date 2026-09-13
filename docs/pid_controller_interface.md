# M20 PID 控制器接口

输入：

- `/route3d_route_slicer/route`：切片后的 RouteTask 列表
- `/lio_odom_hf`（可配置）：导航里程计
- `/cloud_registered_body`：`base_link` 下的点云
- 外部暂停、取消、急停与碰撞状态

输出：

- `/cmd_vel_pid`：PID 候选速度
- `/route3d_pid_controller/alignment_active`：独立姿态对准阶段
- `/route3d_controller/active_source`：PID/Efficient/外部栅格选择
- `/route3d_pid_controller/status`：执行与安全状态 JSON
- `/route3d_pid_controller/swept_volume`：检测体 MarkerArray
- `/route3d_pid_controller/swept_collision_points`：碰撞点云
- `/route3d_pid_controller/nearest_hit_distance`：最近命中距离，未命中为 -1

普通跟踪禁止横移；姿态对准阶段允许低速横移。最终路线目标强制对准 yaw，中间点遵从
`alignFinalYaw`。

模式 0 的三维停障按时间戳同步点云与六自由度里程计。M20 实体尺寸为
`0.82 x 0.43 x 0.57 m`，检测体为 `0.92 x 0.53 x 0.57 m`，路径参考高度为
`ground Z + heightOffsetM + 0.57 m`。参数位于
`src/route3d_pid_controller/config/m20_pid.yaml`。

PID 不直接连接 M20；所有速度必须经过 `route3d_m20_adapter` 的 ready、RL 状态、命令时效
与限幅门控。
