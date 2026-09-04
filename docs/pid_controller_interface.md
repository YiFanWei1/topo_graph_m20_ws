# Route3D 通用 PID 控制器与 Go2 接口

## 1. 定位与安全边界

`route3d_pid_controller` 是一个纯 C++17 的路径跟踪功能包，直接消费
`route3d_route_slicer/RouteTaskArray`，输出标准 ROS 2 `geometry_msgs/Twist`：

```text
RouteTaskArray -> 路线执行器
                    | 平地：PID -> /cmd_vel_pid
                    | 坡段：effi -> /cmd_vel_effi
                    v
              route3d_go2_adapter（单路仲裁）
    |
    | Unitree SportClient::Move(vx, vy, vyaw)
    v
Go2
```

PID 算法没有链接 Unitree SDK。机器人型号相关内容只存在于最后一级适配器，因此：

- Go2 使用本工作空间的 `route3d_go2_adapter/go2_route_adapter_node`，由它统一持有速度与
  步态 SDK 控制权；
- B2、轮式机器人或仿真只需实现另一个 `Twist -> 底盘命令` 适配器；
- 图搜索、切片、途经点逻辑和 PID 核心无需修改；
- 所有机型都受 ROS 侧超时、暂停、取消和停障状态机保护。

默认 `execution.auto_start=true`。收到新路线后自动执行；适配器仍会等待执行器明确发布
`/route3d_controller/active_source`，因此两路控制器常驻也不会同时向底盘输出。

## 2. 参考代码的 PID 实际做法

参考工程 `graph_pid_ws_refactor` 的控制链分成两层：

1. `Controller` 状态机选择前视点、检查到点、检查碰撞并决定
   `ROTATION/FOLLOWING/ADJUSTMENT/FINISHED`；
2. `PoseCtrl` 把地图系目标变换到机体系，对 `x/y/yaw` 三个误差分别做位置式 PID，最后
   调用机器人 SDK 的 `Move(vx, vy, vyaw)`。

其 `Pid::pid_control_position()` 包含：

- 比例、积分、微分三项；
- 误差超过 `1.3` 时关闭并清空积分；
- 积分上下限；
- 速度输出上下限；
- 固定控制周期，Go2 配置为 50 Hz。

参考 Go2 配置中的 `Planner/forwardDis` 是 `1.0 m`。控制器寻找距离当前位置约该距离的
轨迹点；接近最终目标时改用最后一点。上游还会对稀疏点插值和平滑。

本项目没有机械复制其中的全局变量、编译宏和 SDK 耦合，而是保留控制思想并修正接口：

- PID 使用实际 `dt`，不是假定每周期永远准时；
- 微分项增加低通滤波；
- 停障时清空积分，避免解除障碍后速度突跳；
- 增加速度和角速度加速度限制；
- 轨迹投影进度单调推进，降低回环附近跳段风险；
- 默认前视距离改为更保守的 `0.60 m`；
- 前视点受强制途经门限制，不会越过尚未通过的拐点。

所有增益和限幅都在 `config/go2_pid.yaml` 中，后续实机应根据日志逐步标定，不能直接把
参考工程最高 `1.8 m/s`、`1.8 rad/s` 的 PID 输出上限当作初次测试速度。

## 3. 不切弯的途经门

切片消息中的点分为：

- 普通中间点：只是折线路径采样点；
- `must_pass_through=true`：必须进入 `pass_radius_m` 才能释放后续路线；
- 任务末点：使用任务的 `endpoint_tolerance_m`；
- 整条路线末点：还可通过 `align_goal_yaw` 要求最终 yaw 对齐。

控制循环先把机器人投影到当前允许的折线路段，再沿弧长选择前视点。如果前方存在尚未
通过的强制点：

```text
前视位置 = min(当前进度 + lookahead，强制点弧长)
```

因此即使前视距离大于机器人到拐点的距离，目标也只会落在拐点，不会直接跑到转角后的
线段。当机器人进入 `passRadiusM` 后才释放下一段。临近强制点还会使用
`tracking.corner_speed_mps` 限速，但不强制停车。

这比“看到目标点是拐点就临时改到点距离”更清晰，因为：

- `isCorner` 只描述几何；
- `mustPassThrough/passRadiusM` 描述中间路线约束；
- `acc` 描述任何普通点或特殊点作为最终目标时的到达精度。

## 4. 参考工程的停障是怎样触发的

参考工程的 PID 本身不检测障碍。每次 `Controller::stateMachine()` 在执行 PID 前调用两类
碰撞检查：

1. `check_collision()` 读取高程地图的底盘碰撞等级；
2. `check_collision(length, spacing)` 抽取当前前视点之后的轨迹，调用
   `calculateTrajectoryCollisionLevel()` 检查轨迹是否与地图障碍相交。

旧字段 `obs` 的实际语义是：

| 参考 `obs` | Schema V2 `obstacleMode` | 行为 |
| ---: | ---: | --- |
| `0` | `0` | 停障等待：近障或底盘碰撞立即停止，障碍清除后继续原路径 |
| `1` | `1` | 绕障：近障停车并异步重规划，远障可后台重规划 |
| `3` | `3` | 忽略普通障碍；严重碰撞仍有额外保护 |
| 栅格任务 | `4` | 保留为显式外部控制器任务；当前自动坡段链不使用 DWA |

参考代码在命中时调用 `poseCtrl(true)`，最终进入 `StopMove()`；但没有把状态机永久切成
失败状态。下一控制周期继续检查，当碰撞结果消失后又进入 `FOLLOWING`。底盘碰撞还通过
20/25 个控制周期计数保持一段时间，防止障碍边界抖动导致走走停停。

所以，`obstacleMode=0` 确实是“停障模式”，并不是局部绕障模式。

## 5. 当前高程图轨迹碰撞实现

### 5.1 高程图

当前实机输入 `/cloud_registered_body` 的点坐标已经位于 `base_link`。每帧点云按参考
`ElevationMap` 的方式重建机器人周围 `4.0m × 4.0m` 的局部高程图：

```text
栅格分辨率：0.10m
默认地面高度：-0.40m
参与障碍建图的高度：[-0.20, 0.40]m
机身剔除框：x=(-0.50,0.30)m, y=(-0.25,0.25)m
```

随后对每个栅格的 3×3 邻域计算样本标准差。参考代码条件
`stdev * 20 > 2.5` 等价于高度标准差超过 `0.125m`；满足条件的格子进入 `standard`
障碍层。实现不再使用“框内至少 5 个原始点”的规则。

注意：节点当前不对点云做 TF。若更换点云话题，必须保证 XYZ 位于机器人机体系。

### 5.2 沿轨迹按分辨率扫描

控制器把剩余地图系路径转换到当前机体系，并在每个抽样姿态上旋转、平移 Go2 警戒足迹。
默认足迹与参考 Go2 配置一致：

- `x=[0.05,0.60]m`，5 个采样；
- `y=[-0.25,0.24]m`，两侧各内收 `0.13m` 后取 5 个采样；
- `obstacleMode=0`：沿轨迹每 `0.40m` 采样，最多 15 个姿态，碰撞立即停车；
- `obstacleMode=1`：近场使用 25 个姿态/`0.25m`，碰撞时停车并请求重规划；远场使用
  60 个姿态/`0.20m`，仅请求后台重规划；
- 当前机器人足迹碰撞会保持 20 个控制周期，与参考代码的 `coll_times=20` 一致。

任一足迹采样落在 `standard` 障碍格上即判定碰撞。停障时发布零 `Twist`、清空 PID 状态，
并把活动控制器切为 `none`；清除后从当前位姿恢复原任务。点云超过 `0.50s`、里程计超过
`0.30s`、点云解析失败、外部急停或 `/collision_level>=100` 同样停车。

`obstacleMode=1` 的近场或远场命中还会发布：

```text
/route3d_pid_controller/replan_required = true
```

模式 1 的重规划信号仍预留给外部规划器。自动识别出的坡段使用 `obstacleMode=3`，由
`efficient_3d_local_planner` 跟踪；平地保持 PID 的模式 0 停障。

## 6. 控制状态机

```text
IDLE
  -> READY                  收到 RouteTaskArray
  -> TRACKING               默认自动开始，或手动调用 ~/start
  -> PAUSED                 人工 pause
  -> WAITING_TRANSITION     步态切换或要求停车的任务边界
  -> HANDOVER_REQUIRED      遇到尚未接入的显式外部控制器
  -> FINISHED               最终目标验收成功
  -> CANCELLED              人工取消
  -> ERROR                  非法任务或计算异常
```

坡点任务带 `requires_gait_switch_at_start=true`。路线执行器会在边界停车并进入
`WAITING_TRANSITION`；`route3d_go2_adapter` 会先调用 `StopMove()`，再执行
`StaticWalk()/SwitchGait(3)`，确认稳定后通过带路线/任务编号的服务自动选择 PID 或 effi。
人工 `~/continue` 仅保留给业务停车任务和调试。

跟踪进入终点前 `0.30 m` 后切换为独立 `ADJUSTMENT` 阶段。该阶段继续使用机体系
`vx/vy/wz`，Go2 默认限速为 `0.20/0.20/0.90`，最终路线目标按 `0.10 m/0.15 rad`
验收。正常跟踪的平面线速度（`hypot(vx,vy)`）最高为 `0.80 m/s`。

## 7. ROS 接口

### 7.1 输入

| 名称 | 类型 | 默认值/说明 |
| --- | --- | --- |
| `/route3d_route_slicer/tasks` | `RouteTaskArray` | 切片器输出，transient-local |
| `/lio_odom_hf` | `nav_msgs/Odometry` | 机器人定位 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 机体系障碍点云 |
| `/route3d_pid_controller/external_safety_stop` | `std_msgs/Bool` | 外部安全停机信号 |
| `/collision_level` | `std_msgs/Int32` | 参考高程图碰撞等级；达到 100 时任何障碍模式都停车 |

### 7.2 输出

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel_pid` | `geometry_msgs/Twist` | 机体系 `vx/vy/wz`，交给机型适配器 |
| `/route3d_controller/efficient_path` | `nav_msgs/Path` | 当前坡段下发给 effi 的路径 |
| `/route3d_controller/active_source` | `std_msgs/String` | `pid`、`efficient_3d_local_planner` 或 `none` |
| `/route3d_pid_controller/status` | `std_msgs/String` | JSON 状态、任务、进度、停障原因 |
| `/route3d_pid_controller/diagnostics` | `DiagnosticArray` | 标准诊断状态 |
| `/route3d_pid_controller/active_path` | `nav_msgs/Path` | 当前 PID 任务片 |
| `/route3d_pid_controller/lookahead` | `PointStamped` | 当前前视目标 |
| `/route3d_pid_controller/obstacle_stop` | `std_msgs/Bool` | 综合安全停止状态 |
| `/route3d_pid_controller/replan_required` | `std_msgs/Bool` | 模式 1 的重规划请求 |

### 7.3 服务

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/route3d_pid_controller/start` | `std_srvs/Trigger` | 手动模式下执行已缓存路线；默认自动启动无需调用 |
| `/route3d_pid_controller/continue` | `std_srvs/Trigger` | 确认步态等边界动作完成 |
| `/route3d_pid_controller/pause` | `std_srvs/Trigger` | 停车并保留任务 |
| `/route3d_pid_controller/resume` | `std_srvs/Trigger` | 从当前位姿恢复 |
| `/route3d_pid_controller/cancel` | `std_srvs/Trigger` | 停车并取消当前执行 |

## 8. 编译和无实机检查

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-up-to route3d_pid_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon test --packages-select route3d_pid_controller
colcon test-result --test-result-base build/route3d_pid_controller --verbose
```

启动 Dijkstra、切片和 PID，但不启动 Go2 适配器：

```bash
ros2 launch route3d_pid_controller pid_route_demo.launch.xml \
  graph_file:=/home/wei/github_code/topo_graph_ws_/data/regu_schema_v2/topoGraph_data.json
```

发送路径请求：

```bash
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [1, 109]}"
```

默认会在点云和里程计就绪后自动执行，可直接观察：

```bash
ros2 topic echo /route3d_pid_controller/status \
  --qos-durability transient_local

ros2 topic echo /cmd_vel_pid
```

此时没有启动 `route3d_go2_adapter`，所以只能看到速度命令，Go2 不会运动。可先用 bag
检查前视点、停障和速度方向。

测试外部停障：

```bash
ros2 topic pub --once /route3d_pid_controller/external_safety_stop \
  std_msgs/msg/Bool '{data: true}'

ros2 topic pub --once /route3d_pid_controller/external_safety_stop \
  std_msgs/msg/Bool '{data: false}'
```

## 9. Go2 实机接入

确认两路命令、障碍框和急停都正确后，启动统一适配器：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch route3d_go2_adapter go2_pid_route.launch.xml \
  graph_file:=/absolute/path/to/topoGraph_data.json \
  network_interface:=enp2s0 enable_motion:=true
```

PID 和 effi 同时常驻。适配器只转发 `/route3d_controller/active_source` 选中的新命令，切换
时清空两路缓存并调用 `StopMove()`；平面速度再次限制到 `0.80 m/s`，若 0.10 s 内没有
新命令就发送零速度。紧急停止整条路线：

```bash
ros2 service call /route3d_pid_controller/cancel std_srvs/srv/Trigger '{}'
```

初次实机测试建议保持遥控器急停可用，先把 `limits.maximum_vx_mps` 降至 `0.15`，确认
机体系正方向、侧向方向、yaw 正负号以及点云坐标后再逐级升速。

## 10. 扩展其他机型

适配器只需满足四个约束：

1. 订阅 `geometry_msgs/Twist`；
2. 把 `linear.x/y` 和 `angular.z` 转成底盘命令；
3. 输入超时自动发零速度；
4. 节点退出时执行硬件停止接口。

差速轮式底盘可在独立 YAML 中设置 `limits.maximum_vy_mps=0.0`。不支持横移或倒退的机型
应由适配器再次拒绝非法命令，而不是修改通用 PID 算法。
