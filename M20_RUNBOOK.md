# M20 Route3D 实机与 Bag 回放操作手册

本文档对应工作空间：

```text
/home/wei/github_code/topo_graph_m20_ws
```

推荐始终按“检查输入 → 自动打点并保存 → 离线禁运动检查 → 实机启用 → 发送目标”的顺序操作。
Route3D 导航使用 `/lio_odom`、`/lio_odom_hf` 和 `/cloud_registered_body`；
`/m20/state/odometry` 仅用于诊断，不能替代 LIO 定位。

## 1. 每个终端的公共准备

新开终端后先执行：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

首次部署或修改代码后编译：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

同一次测试的所有终端必须使用相同的 `ROS_DOMAIN_ID`。现场 Web 配置使用 42 时，可在每个终端先执行：

```bash
export ROS_DOMAIN_ID=42
```

## 2. 实机运行前提

### 2.1 启动雷达和 LIO

先按 M20 现场部署方式启动雷达驱动、建图或定位程序。它们不由 Route3D 总启动入口重复启动。
Route3D 至少需要以下输入：

| 话题 | 作用 | 期望坐标系 |
|---|---|---|
| `/lio_odom` | 点云同步、自动打点和控制 | `camera_init -> base_link` |
| `/lio_odom_hf` | 高频定位和网页显示 | `camera_init -> base_link` |
| `/cloud_registered_body` | 实时雷达点云 | `base_link` |

检查输入：

```bash
ros2 topic hz /lio_odom
ros2 topic hz /lio_odom_hf
ros2 topic hz /cloud_registered_body
ros2 topic echo /lio_odom --once
ros2 topic echo /cloud_registered_body --once
```

`/cloud_registered_body.header.frame_id` 应为 `base_link`。M20 外参由 Route3D 发布为：

```text
body -> base_link: xyz=[0.0, 0.0, 0.0], rpy=[0, 0, 0]
```

当前定位约定 `body` 与 `base_link` 重合。点云已经在 `base_link` 中，不能再重复平移。
雷达安装位置 `[0.32028, 0.0, -0.013]` 只用于 Efficient mapper 的射线起点。

### 2.2 检查 M20 basic_server 网络

只有实机控制需要 `basic_server_bridge`。当前默认地址在
`src/basic_server_bridge/config/basic_server_bridge.yaml` 中：

```text
10.21.31.103:30001/TCP
```

启动实机运动前至少确认：

```bash
ping -c 3 10.21.31.103
nc -vz 10.21.31.103 30001
```

如果出现 `Connection refused`，说明 M20 服务端没有监听或地址/端口不对；此时不要启用运动。

## 3. 实机自动打点并保存

自动打点阶段用遥控器人工驾驶 M20，不要同时启动自主规控。

终端 A：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/1_start_auto_waypoint.sh
```

操作顺序：

1. 等待 RViz、在线骨架和数据输入正常。
2. 在启动终端按 `1`，开始同步记录里程计和点云。
3. 用遥控器沿希望导航的路线行走；转弯、坡道和闭环按现场路线正常通过。
4. 回到安全位置后，在同一终端按 `2`。
5. 程序停止订阅、生成最终拓扑，然后提示输入名称，例如 `m20_floor1_001`。
6. 输入名称并回车。已有目录不会被覆盖。
7. 保存完成后按 `q` 退出。

正式控制图保存到：

```text
data/m20_floor1_001/topoGraph_data.json
```

同目录还会保存 `pose.json`、`pose_realtime.json`、`key_frames/`、构图中间文件和
`processing_summary.json`。新记录图默认使用：

- M20 高度 `0.57 m`；
- 新边 `obstacleMode=1`，即整条边默认由 Efficient 3D 主动绕障；
- 新顶点 `alignFinalYaw=false`；最终目标仍无条件对准该顶点的 `rpy.yaw`。

保存后检查 JSON 和点边数量：

```bash
GRAPH=/home/wei/github_code/topo_graph_m20_ws/data/m20_floor1_001/topoGraph_data.json
python3 -m json.tool "$GRAPH" >/dev/null
jq '{version, vertices:(.vertices|length), edges:(.edges|length)}' "$GRAPH"
```

## 4. 加载拓扑并做禁运动检查

当前总入口会一次性完成“加载拓扑 + Dijkstra + 路径切片 + PID + Efficient + M20 adapter”。
第一次加载新图必须保持 `enable_motion=false`：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/2_start_planning_control.sh \
  ./data/m20_floor1_001/topoGraph_data.json false true
```

此时 bridge 会连接实机，但不会启动 M20 准备状态机；adapter 持续输出零速度。可先发送路线验证
拓扑连通性、路径切片和控制器选择：

```bash
./sh/4_send_goal.sh 1 20
```

另开终端检查：

```bash
ros2 topic echo /route3d_dijkstra/status
ros2 topic echo /route3d_route_slicer/status
ros2 topic echo /route3d_pid_controller/status
ros2 topic echo /route3d_controller/active_source
ros2 topic echo /route3d_m20_adapter/status
ros2 topic echo /cmd_vel_smoothed
```

禁运动时 `/route3d_m20_adapter/status` 应包含：

```json
{"blocked":true,"block_reason":"motion_disabled"}
```

`/cmd_vel_smoothed` 必须持续为零。PID/Efficient 的原始计算命令可分别观察：

```bash
ros2 topic echo /cmd_vel_pid
ros2 topic echo /cmd_vel_effi
```

如果只想查看或编辑拓扑，不启动规控，可启动 Web 控制台：

```bash
ros2 launch route3d_web_console route3d_web_console.launch.py
```

浏览器访问 `http://机器人IP:8080`，选择 `m20_floor1_001`。planner 运行期间 Web 会禁止写回拓扑；
修改拓扑前先停止 planner。

## 5. 启动实机控制

先停止上一套禁运动进程，不能同时运行两套规控节点：

```bash
./sh/3_stop_all_ros.sh
```

确认遥控急停可用、机器人周围安全、定位正确后，显式启用运动：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/2_start_planning_control.sh \
  ./data/m20_floor1_001/topoGraph_data.json true true
```

启用后准备节点执行：普通使用模式确认 → 必要时 Stand → RL 状态 17。另开终端观察：

```bash
ros2 topic echo /m20/control/preparation_status
ros2 topic echo /m20/control/ready
ros2 topic echo /m20/state/motion_state
ros2 topic echo /route3d_m20_adapter/status
```

只有同时满足以下条件才允许发出非零 `/cmd_vel_smoothed`：

- `enable_motion=true`；
- `/m20/control/ready=true`；
- `/m20/state/motion_state` 为 17 且反馈新鲜；
- PID 或 Efficient 控制源有效；
- 控制命令没有超时。

M20 不发布步态切换命令，也不等待 gait ACK。

## 6. 发送目标点

### 6.1 明确指定起点和终点

机器人已在起点附近时使用：

```bash
./sh/4_send_goal.sh 1 20
```

起点和终点必须是拓扑 JSON 中存在的不同顶点。机器人与声明起点不一致时不要使用这种方式。

### 6.2 只指定终点（现场推荐）

让 Dijkstra 根据当前 `/lio_odom` 自动匹配最近拓扑点：

```bash
./sh/6_send_goal_only.sh 20
```

默认要求机器人距可匹配起点不超过 `1.0 m`。如果提示找不到起点，应先检查定位、地图坐标和
机器人是否确实位于拓扑路线附近，不要盲目放大匹配半径。

### 6.3 暂停、恢复、取消和停止

```bash
ros2 service call /route3d_pid_controller/pause std_srvs/srv/Trigger '{}'
ros2 service call /route3d_pid_controller/resume std_srvs/srv/Trigger '{}'
ros2 service call /route3d_pid_controller/cancel std_srvs/srv/Trigger '{}'
./sh/3_stop_all_ros.sh
```

最终目标无论是否坡点、无论 `alignFinalYaw` 的值，都要对准顶点 `rpy.yaw`。中间顶点仅在
`alignFinalYaw=true` 时停车对准。

## 7. M20 原生 Bag：录制建议

后续由 M20 实机录制的 Bag 是 M20 原生数据。推荐至少录制导航输入和诊断话题：

```bash
mkdir -p /home/wei/bag
ros2 bag record -s mcap -o /home/wei/bag/m20_route_001 --topics \
  /lio_odom \
  /lio_odom_hf \
  /cloud_registered_body \
  /tf \
  /tf_static \
  /m20/state/motion_state \
  /m20/state/control_result \
  /m20/control/ready \
  /m20/control/preparation_status \
  /route3d_controller/active_source \
  /route3d_pid_controller/status \
  /route3d_m20_adapter/status \
  /cmd_vel_pid \
  /cmd_vel_effi \
  /cmd_vel_smoothed
```

用 `Ctrl+C` 正常结束录制，然后检查：

```bash
ros2 bag info /home/wei/bag/m20_route_001
```

M20 原生 Bag 的规则非常明确：

```text
recording_body_height_m = 0.57
M20 原生 Bag 不加 +0.17 m，不修改 odom Z，不重复变换点云。
```

## 8. M20 原生 Bag：重新自动打点

这是用录制数据复现“实机自动打点”的流程。

终端 A：启动自动打点，使用默认 M20 配置 `body_height=0.57`：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/1_start_auto_waypoint.sh
```

在终端 A 按 `1` 后，终端 B 开始回放传感器输入：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/9_play_sensor_bag.sh /home/wei/bag/m20_route_001 1.0 false
```

Bag 播放结束后回到终端 A，按 `2`，输入新的结果目录名，例如 `m20_route_001_rebuilt`，等待生成：

```text
data/m20_route_001_rebuilt/topoGraph_data.json
```

不要在 Bag 已经播放一段后才按 `1`，否则开头轨迹会丢失。需要重来时应退出本轮自动打点，重新启动，
再从 Bag 起点播放。

## 9. M20 原生 Bag：加载拓扑、运行规控和发送目标

Bag 回放绝不能连接 `basic_server`，也不能启用运动。

终端 A：启动专用仿真规控栈：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/8_start_bag_planning_control.sh \
  ./data/m20_route_001_rebuilt/topoGraph_data.json true 0.57
```

这个入口固定使用：

```text
enable_motion=false
start_m20_bridge=false
publish_odometry_tf=true
use_sim_time=true
```

Bag 中录制的旧 `/tf` 不参与回放。规控栈从 `/lio_odom_hf` 生成唯一的动态
`camera_init -> base_link`。这是 Bag 专用重建；实机定位发布 `camera_init -> body`，再连接
静态 `body -> base_link`，不会让 `body` 出现两个父坐标系。

因此不会出现 `10.21.31.103:30001 Connection refused`，也不会把命令发送到实机。

终端 B：从头回放 Bag：

```bash
./sh/9_play_sensor_bag.sh /home/wei/bag/m20_route_001 1.0 false
```

终端 C：等 `/lio_odom` 开始发布且机器人位于拓扑附近后发送目标：

```bash
./sh/6_send_goal_only.sh 20
```

也可以明确发送：

```bash
./sh/4_send_goal.sh 1 20
```

Bag 验证时重点观察：

```bash
ros2 topic echo /route3d_dijkstra/status
ros2 topic echo /route3d_controller/active_source
ros2 topic echo /cmd_vel_pid
ros2 topic echo /cmd_vel_effi
ros2 topic echo /route3d_pid_controller/nearest_hit_distance
ros2 topic echo /route3d_m20_adapter/status
```

RViz 中的停障/绕障显示按边模式区分：

- `M20 3D Stop Swept Volume (mode 0 PID)`：模式 0 的三维路径扫掠检测体；无碰撞为
  亮绿色粗线框方盒，命中时变为红色粗线框。
- `M20 Swept Collision Points (mode 0 PID)`：模式 0 扫掠体内真正参与停车判定的红色碰撞点。
- `Efficient 3D Search Corridor (mode 1)`：模式 1 的蓝色三维 A* 搜索走廊。
- `Efficient 3D Optimized Local Path (mode 1)`：模式 1 实际交给局部跟踪器的洋红色绕障路径。
- `Efficient 3D Hard Obstacles (mode 1)`：模式 1 局部体素图中的橙色硬障碍。

自动生成的正式边默认是 `obstacleMode=1`，因此回放正式图时不会发布模式 0 的停车扫掠盒；
此时应观察 Efficient 的搜索走廊、局部路径和硬障碍。只有将测试边明确设成
`obstacleMode=0` 且 `controllerMode=pid` 后，才会显示三维停车扫掠盒。不要为了看到扫掠盒而
修改正式拓扑，应该复制一份拓扑作为模式 0 测试图。

工作空间已经提供一份 `regu` 对比图，不会修改正式数据。它完整保留 109 个点和 108 条边，
仅把两端都在 1–40 范围内的 39 条边改为模式 0/PID；40→41 及其余 69 条边保持模式 1：

```bash
./sh/8_start_bag_planning_control.sh \
  ./data/m20_validation/regu_mode0_1_40/topoGraph_data.json true 0.40
./sh/9_play_sensor_bag.sh /home/wei/bag/regu 1.0 false
./sh/4_send_goal.sh 1 40
```

在这套测试中应看到亮绿色粗线框三维扫掠盒；如果命中判定障碍，线框变红并显示红色碰撞点。
若发送 `./sh/4_send_goal.sh 1 109`，还可对比 40 号点前的 PID 停障和 40 号点后的
Efficient 绕障。详细说明和实测结果分别见
`data/m20_validation/regu_mode0_1_40/README.md` 和
`validation/bags/regu_mode0_1_40_runtime.json`。

如果需要观察整条 regu 路线全部使用三维停障，加载全边模式 0/PID 的独立版本：

```bash
./sh/8_start_bag_planning_control.sh \
  ./data/m20_validation/regu_mode0_all/topoGraph_data.json true 0.40
./sh/9_play_sensor_bag.sh /home/wei/bag/regu 1.0 false
./sh/4_send_goal.sh 1 109
```

它仍完整保留原图 109 个点和 108 条边；详细说明见
`data/m20_validation/regu_mode0_all/README.md`。

adapter 因 `enable_motion=false` 会把 `/route3d_m20_adapter/selected_command` 和
`/cmd_vel_smoothed` 保持为零；控制器是否在计算应看 `/cmd_vel_pid`、`/cmd_vel_effi` 和控制器状态。

Bag 回放是开环数据回放，不是机器人动力学闭环仿真。录制的 `/lio_odom` 轨迹不会因本次计算的
`cmd_vel` 改变。因此它适合验证：

- 自动打点和保存结果；
- Dijkstra 路线是否连通；
- 路径切片和 PID/Efficient 选择；
- 点云停障、绕障输入、最终姿态对准状态；
- 所有禁运动和超时门控。

它不能证明 M20 会按本次输出速度实际走出同样轨迹；闭环跟踪效果最终仍需低速实机测试。

## 10. 现有 Go2 Bag 与未来 M20 Bag 的区别

现有三套历史 Bag：

```text
/home/wei/bag/regu
/home/wei/bag/xili/test_0911
/home/wei/bag/full_nav_replay_with_plan
```

由约 `0.40 m` 高度的 Go2 录制，仅作为历史验证数据。其已重建拓扑位于：

```text
data/m20_validation/regu/topoGraph_data.json
data/m20_validation/xili/topoGraph_data.json
data/m20_validation/full_nav_replay_with_plan/topoGraph_data.json
```

回放历史 Go2 图时，Dijkstra 起点匹配高度应传 `0.40`：

```bash
./sh/8_start_bag_planning_control.sh \
  ./data/m20_validation/regu/topoGraph_data.json true 0.40
./sh/9_play_sensor_bag.sh /home/wei/bag/regu 1.0 false
```

历史离线三维扫掠验证会把 M20 检测中心相对 Go2 轨迹上移 `0.17 m`。这条补偿只属于旧 Go2
Bag，不能沿用到 M20 原生 Bag。

| 数据来源 | 自动构图 `body_height` | 回放起点匹配高度 | 额外 Z 补偿 |
|---|---:|---:|---:|
| 历史 Go2 Bag | `0.40` | `0.40` | M20 离线扫掠 `+0.17` |
| M20 原生 Bag | `0.57` | `0.57` | `0.00` |

## 11. M20 实机独立手动速度控制

这条链路用于脱离 Route3D 直接验证 M20 速度控制。启动前先退出所有 Route3D 实机 launch
和其他 `basic_server_bridge`，然后执行：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
./sh/10_start_m20_manual_velocity.sh true
```

等待 `/m20/control/preparation_status` 显示 `ready=true`、`/m20/manual/status` 显示
`motion_state=17` 后，持续向专用话题发布低速指令：

```bash
ros2 topic pub -r 10 /m20/manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止发布后 0.30 秒内自动归零。详细限速、软件急停和离线测试方法见
`src/m20_velocity_control/README.md`。

## 12. 常见问题

### Bag 回放时 bridge 一直报连接失败

启动方式错了。停止当前节点后使用：

```bash
./sh/3_stop_all_ros.sh
./sh/8_start_bag_planning_control.sh /绝对路径/topoGraph_data.json true 0.57
```

确认启动日志中为 `start_m20_bridge=false`。

### 已发送目标但 `/cmd_vel_smoothed` 始终为零

- Bag 模式下这是正确且强制的安全行为；看 `/cmd_vel_pid` 和 `/cmd_vel_effi`。
- 实机模式检查 adapter 的 `block_reason`、ready、motion state 17 和控制源。

### 发送仅终点请求后找不到起点

检查 `/lio_odom` 是否新鲜、定位是否在对应地图坐标系、机器人是否在拓扑点 `1.0 m` 范围内，
并确认所加载拓扑与当前地图/Bag 是同一批数据。

### 自动打点只有很少几帧

检查 `/lio_odom` 与 `/cloud_registered_body` 的时间戳差。现场 Web 模式配置了 `0.15 s` 同步窗口，
终端默认 M20 配置当前为 `0.05 s`；如果现场 LIO 固定早于点云约 `0.10 s`，应在专用 YAML 中把
`route3d_live.recorder.sync_slop` 调到 `0.15` 后作为参数传给 `1_start_auto_waypoint.sh`。

### 如何立即安全停止

优先使用遥控急停，然后执行：

```bash
./sh/3_stop_all_ros.sh
```

脚本会先请求取消巡航和当前 PID 任务，再停止当前用户的 ROS 2 进程。
