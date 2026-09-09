# topo_graph_ws

这个工作空间已经把实时路线记录、智能拓扑生成、Dijkstra 搜索、语义切片、PID 跟踪和
Go2 速度适配串起来。新控制链以 Schema V2 `topoGraph_data.json` 为权威地图；
`topoSingle_data.json` 只保留给旧线性控制链兼容使用。

## 数据流程

```text
实时 /lio_odom + /cloud_registered_body
        |
        +--> pose.json + key_frames/*.pcd
        |
        +--> route3d_topology_core 离线重放
                    |
                    +--> topoGraph_data.json（Schema V2 智能拓扑权威结果）
                              |
                              +--> C++ Dijkstra 最短路径
                                      |
                                      +--> C++ 语义切片 RouteTaskArray
                                              |
                                              +--> 路线执行器 /active_source
                                                    |             |
                                                    |             +--> 坡段 effi /cmd_vel_effi
                                                    +--> 平地 PID /cmd_vel_pid
                                                                  |
                                                        Go2 单路仲裁适配器

        +--> 旧线性流程 --> topoSingle_data.json（兼容旧控制链）
```

实时阶段同时写 `live/topoSingle_live.json` 和 `live/topoGraph_live.json`。前者保持旧线性
语义；后者按真实图结构工作，能抑制沿相邻旧边的原路退回、从旧边中部离开时拆边，
还能通过连续几何匹配和同步关键帧 PCD 验证建立闭环、抑制同向重复绕圈，并保留稳定的
节点 ID。新的 Dijkstra、切片器和 PID 控制链读取 `topoGraph_data.json`；旧的
`recorded_waypoint_route`/线性控制器仍读取 `topoSingle_data.json`，两条链不要混用文件。

目标文件中的每个点为 `id x y z_ground NORMAL|CORNER`。`z_ground` 是地面高度；转换器匹配原始 Nav2 图中的机身位姿来恢复四元数和 RPY，因此只在“地面坐标”和“机身坐标”之间使用一次 `body_height`，默认值为 `0.40 m`。

智能拓扑状态机、原路退回、PCD 闭环、拆边、在线/离线产物以及当前已知限制的完整说明见
[`docs/topology_generation_logic.md`](docs/topology_generation_logic.md)。

产品生成的四个 Topology JSON 已统一为 Schema V2。新增控制属性、默认值、类型、单位，
以及与 `graph_pid_ws_refactor` 字段的逐项映射见
[`docs/topology_schema_v2_interface.md`](docs/topology_schema_v2_interface.md)。三种图格式的
总体差异见 [`docs/topology_format_comparison.md`](docs/topology_format_comparison.md)。

基于 Schema V2 `topoGraph_data.json` 的 Dijkstra 最短路径包、ROS 话题、RViz 演示和
复杂测试图使用方法见 [`docs/dijkstra_planner.md`](docs/dijkstra_planner.md)。

Dijkstra 路径之后的纯 C++ 语义切片、坡点步态意图、拐点强制途经约束、参考工程业务
边兼容规则和后续控制器执行接口见
[`docs/route_slicer_interface.md`](docs/route_slicer_interface.md)。

通用 C++ PID 跟踪器、Go2 速度适配、停障触发与恢复、拐点防切弯和实机安全启动步骤见
[`docs/pid_controller_interface.md`](docs/pid_controller_interface.md)。

## 编译

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`nav2_util` 的上游测试依赖 `test_msgs` 没有包含在当前源码集合中，所以默认关闭测试；运行组件和本流程包均正常编译。

## 从重新打点到 Go2 控制（当前推荐流程）

### 1. 编译并加载环境

首次使用或源码更新后执行：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

每个新终端都需要重新执行上面的两条 `source`。实机运行使用系统时间，即
`use_sim_time:=false`。

### 2. 重新自动打点并保存智能图

先确保定位和点云正在发布：

```bash
timeout 5 ros2 topic hz /lio_odom
timeout 5 ros2 topic hz /cloud_registered_body
```

启动产品打点程序：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_product_demo live_route_product.launch.py
```

键盘流程：

1. 按 `1` 开始记录，然后遥控机器人完整走过需要建图的路线。
2. 允许原路退回和重复绕圈；实时窗口中绿色边是去重后的拓扑，橙色粗边是闭环。
3. 回到启动终端按 `2` 停止并处理，输入一个新的保存名称，例如 `building_a_v1`。
4. 等待最终处理完成并打印保存目录，再按 `q` 退出。

最终控制使用：

```text
/home/wei/github_code/topo_graph_ws_/data/building_a_v1/topoGraph_data.json
```

自动打点当前生成的以下四个文件都已经是
`schema.name=route3d_topology, schema.version=2`：

| 文件 | 用途 | 是否用于新控制链 |
| --- | --- | --- |
| `live/topoGraph_live.json` | 实时智能图预览 | 否，仅观察 |
| `topoGraph_data.json` | 停止后用 `pose.json` 离线复核得到的权威智能图 | **是** |
| `live/topoSingle_live.json` | 实时线性兼容预览 | 否 |
| `topoSingle_data.json` | 旧线性控制器兼容文件 | 仅旧链路 |

生成器会保留 `isCorner`、`isSlope`、`isJunction`、稳定节点 ID、拆边和闭环边，并补齐
控制字段。自动生成时业务字段采用安全的通用默认值：

```text
controllerMode=auto       -> 平地解析为 pid，坡段解析为 efficient_3d_local_planner
obstacleMode=0            -> 遇障停车等待，清除后恢复
locomotionMode=0          -> 普通步态
travelMode=bidirectional  -> 双向边
linearSpeedMps=1.0        -> 边期望上限；适配器最终把平面速度限制在 0.80m/s
```

`locomotionMode=0` 是边的默认值；如果路径经过 `meta.isSlope=true` 的点，切片器仍会根据
坡段边界生成 `switch_gait_3`/`static_walk` 步态切换任务。

如果某条边必须单向、需要局部规划器、专用障碍框或特殊速度，应在建图完成后修改该边的
Schema V2 属性。不要通过改 `isCorner` 来表达控制业务。

### 3. 保存后检查

先检查文件合法性和概要：

```bash
GRAPH=/home/wei/github_code/topo_graph_ws_/data/building_a_v1/topoGraph_data.json
python3 -m json.tool "$GRAPH" >/dev/null
jq '{schema, frame_id, vertex_count:(.vertices|length), edge_count:(.edges|length), generation}' "$GRAPH"
```

同时检查：

- RViz 中闭环、分支和节点编号是否符合实际路线；
- `processing_summary.json` 中离线结果以及在线/离线一致性；
- 起点和终点编号；可使用 `[起点ID, 终点ID]` 原接口，或仅发送终点并从当前定位匹配
  1 m 内最近拓扑点作为起点；
- 坡点、拐点和交汇点标记是否合理。

### 4. 用 regu bag 检查完整双控制器流程（不连接底盘）

终端 A 启动完整控制链。`enable_motion:=false` 时不会打开 Unitree SDK；PID 与 effi 都常驻，
但适配器只放行 `/route3d_controller/active_source` 指定的一路：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch route3d_go2_adapter go2_pid_route.launch.xml \
  graph_file:=/home/wei/github_code/topo_graph_ws_/data/regu_schema_v2/topoGraph_data.json \
  enable_motion:=false use_sim_time:=true launch_rviz:=true \
  gait_settle_time_s:=0.10
```

终端 B 从 bag 的有效路段开始加速回放：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play /home/wei/bag/regu --clock --start-offset 75 --rate 5.0
```

终端 C 请求从节点 `49` 到节点 `109` 的路径。PID 默认自动启动，不需要调用 `start`：

```bash
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [49, 109]}"
```

检查执行状态、当前获准控制器和适配器最终选中的速度：

```bash
ros2 topic echo --once /route3d_dijkstra/status --qos-durability transient_local
ros2 topic echo --once /route3d_route_slicer/tasks_json --qos-durability transient_local
ros2 topic echo /route3d_pid_controller/status
ros2 topic echo /route3d_controller/active_source --qos-durability transient_local
ros2 topic echo /route3d_go2_adapter/selected_command
```

预期控制器序列包含“坡段 effi → 平地 PID → 坡段 effi → 平地 PID → none”。平地高程图在
剩余轨迹足迹上检测到高差障碍时会显示 `PID safety stop`，坡段的普通楼梯点云不会触发；外部急停、
碰撞等级和里程计超时在两种控制器下始终有效。

需要立即停止时：

```bash
ros2 service call /route3d_pid_controller/cancel std_srvs/srv/Trigger '{}'
```

### 5. 接入 Go2 实机

首先停止其他会直接向 Go2 发送运动命令的程序。不要另外启动旧的
`up_and_down_demo.launch.py`，因为下面的统一 launch 已经同时启动 PID 和 effi。建议先把
`src/route3d_pid_controller/config/go2_pid.yaml` 中
`limits.maximum_vx_mps` 临时降到 `0.15`。

确认地图和规划结果正确后，使用统一 Go2 适配器启动完整控制链。它是唯一直接调用
`SportClient` 的进程，同时处理速度和坡段步态切换：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch route3d_go2_adapter go2_pid_route.launch.xml \
  graph_file:=/absolute/path/to/topoGraph_data.json \
  network_interface:=enp2s0 enable_motion:=true
```

保持遥控器急停可用。请求路径后会自动执行，不需要另行调用 PID 的 `start`：

```bash
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [起点ID, 终点ID]}"
```

默认边的 `obstacleMode=0`。PID 从点云生成分辨率 `0.10m` 的局部高程图，计算 3×3 邻域
高度标准差，并沿剩余路径以 `0.40m` 间距扫描 15 个位置的 Go2 足迹。标准差超过
`0.125m` 时停车，障碍从轨迹足迹消失后自动恢复。里程计超过 `0.30s` 或点云超过
`0.50s` 未更新也会停车。

### 6. 自动步态、控制器切换与停障

进入坡段时，切片器自动生成 `switch_gait_3`，离开坡段时自动生成 `static_walk`。路线执行器
在边界先把输出切为 `none`；`route3d_go2_adapter` 执行 `StopMove()`、步态调用和稳定性检查，
再按路线编号与任务编号确认。成功后自动选择下一控制器，失败或超时则保持停车，不需要
人工调用 `continue`。

- 自动识别出的坡段会把默认 `obstacleMode=0` 解析成 `3`，忽略楼梯点云造成的普通停障；
  平地仍保持模式 0 的停障等待。`/collision_level >= 100`、外部急停和里程计超时在坡段也
  始终有效。
- 平地由 PID 输出 `/cmd_vel_pid`；坡段由 `efficient_3d_local_planner` 输出 `/cmd_vel_effi`。
  两者始终同时运行，适配器根据 `/route3d_controller/active_source` 独占放行一路，切换时
  清空旧命令并等待新命令。当前链路不使用 DWA。
- 适配器最终将合成平面速度 `hypot(vx, vy)` 限制为 `0.80m/s`。常规路径跟踪主要使用
  `vx/wz`，PID 的终点精确调整保留小范围 `vy`，与参考实现一致。

## 实时记录路线

先启动机器人或回放节点，再启动记录器。输出目录必须是新的目录，程序收到 Ctrl-C 后会把 `pose.json.partial` 完整收尾为 `pose.json`。

```bash
ros2 launch route3d_data_recorder record_route.launch.py \
  output_directory:=/home/wei/github_code/topo_graph_ws_/data/my_route \
  odom_topic:=/lio_odom \
  cloud_topic:=/cloud_registered_body \
  sync_slop_s:=0.05
```

默认每个成功匹配的点云记录一帧，里程计和点云按 header 时间戳在 `sync_slop_s` 内近似同步。记录结果为：

```text
my_route/
├── pose.json
├── key_frames/0.pcd, 1.pcd, ...
└── manifest.json
```

记录阶段不扣机身高度：`pose.json` 的 z 是机身/传感器位姿高度，PCD 点保留在点云消息自己的 frame 中。

记录结束后构图：

```bash
ros2 run nav2_route3d route_graph_builder_3d \
  --map-path /home/wei/github_code/topo_graph_ws_/data/my_route \
  --poses-file pose.json \
  --pcd-dir key_frames \
  --output-json route3d_graph.json
```

## 单独运行离线处理

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run route3d_bag_tools route_corner_target_generator \
  --graph data/regu/route3d_graph.json \
  --output data/regu/route_targets_with_corners.txt \
  --output-topo data/regu/topoSingle_data.json \
  --target-spacing 1.0 \
  --body-height 0.40
```

这一步生成旧线性控制器使用的 `topoSingle_data.json`。新 Dijkstra/PID 控制链应使用产品
流程离线复核生成的 `topoGraph_data.json`。如果需要验证旧线性结果，再额外生成简化的
Nav2 子图：

```bash
ros2 run topo_graph_tools nav2_to_topo_single \
  --graph data/regu/route3d_graph.json \
  --targets data/regu/route_targets_with_corners.txt \
  --output-nav2 data/regu/selected_route3d_graph.json \
  --body-height 0.40
```

输出的 topo 节点在 `meta.isCorner` 中标记拐点；`meta.sourceNodeId` 表示它匹配到的原始 Nav2 图节点。边默认按路线顺序建立，`weight` 为相邻目标点之间的三维距离。

## 一键生成并可视化

先关闭 RViz 验证完整数据链：

```bash
ros2 launch topo_graph_tools topo_conversion_visualization.launch.py \
  launch_rviz:=false
```

需要图形界面时：

```bash
ros2 launch topo_graph_tools topo_conversion_visualization.launch.py \
  launch_rviz:=true
```

默认使用 `data/regu/route3d_graph.json`。也可以替换数据集：

```bash
ros2 launch topo_graph_tools topo_conversion_visualization.launch.py \
  graph_filepath:=/home/wei/github_code/topo_graph_ws_/data/0825_1/route3d_graph.json \
  target_file:=/home/wei/github_code/topo_graph_ws_/data/0825_1/route_targets_with_corners.txt \
  topo_output:=/home/wei/github_code/topo_graph_ws_/data/0825_1/topoSingle_data.json \
  reduced_graph_output:=/home/wei/github_code/topo_graph_ws_/data/0825_1/selected_route3d_graph.json
```

RViz 中显示的是转换后的 `selected_route3d_graph.json`，并叠加目标点路径；这样可以同时确认点的数量、拐点位置、边连接关系以及高度语义。拐点提取阈值可直接传给目标点生成器，当前一键启动文件使用默认阈值。

## 产品演示：键盘分步打通完整流程

### 实机实时模式（推荐验收入口）

推荐通过产品 launch 启动。它会读取同一个 YAML，并按配置选择是否发布
`map -> camera_init` 和 `body -> base_link` 静态 TF：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_product_demo live_route_product.launch.py
```

默认参数文件：

```text
src/route3d_product_demo/config/live_route_product.yaml
```

使用另一份实机配置：

```bash
ros2 launch route3d_product_demo live_route_product.launch.py \
  config_file:=/absolute/path/to/live_route_product.yaml
```

YAML 中 `static_tf.*.publish` 控制静态 TF。默认发布两条单位变换：

```yaml
static_tf:
  map_to_camera_init:
    publish: true
    parent_frame: map
    child_frame: camera_init
    translation: [0.0, 0.0, 0.0]
    rotation_rpy: [0.0, 0.0, 0.0]
  body_to_base_link:
    publish: true
    parent_frame: body
    child_frame: base_link
    translation: [-0.15, 0.0, -0.21]
    rotation_rpy: [0.0, 0.0, 0.0]
```

如果定位模块后来开始发布同名父子 TF，必须把对应 `publish` 改为 `false`，避免 TF
树出现双发布者。平移或安装姿态不是零时，应填写实测外参，不能继续使用单位变换。

仍可绕过 launch 直接运行键盘控制器：

```bash
ros2 run route3d_product_demo route3d_live_keyboard \
  --config /home/wei/github_code/topo_graph_ws_/src/route3d_product_demo/config/live_route_product.yaml
```

实时模式只有两个主要按键：

```text
1  开始同步记录 pose/PCD、在线构建骨架并实时更新 RViz
2  停止订阅、完成写盘、离线复核，输入名称后保存全部结果
3  停止记录器、在线骨架和 RViz 后退出，不执行最终处理，临时数据保留在 /tmp
q  空闲时退出；记录中应使用 2 或 3
```

RViz 中灰色线是完整实时原始轨迹（包括退回过程），绿色线是去重后的拓扑边，青色线
是当前正在退回或重走的旧边，橙色粗边是确认生成的闭环边。蓝色点是普通拓扑点，
黄色点是坡点，红色点是确认拐点，
橙色点是尚未结束的候选拐点，紫色点表示检测到定位跳变。
在线数据来自记录器成功写盘后发布的 `/route3d/synchronized_pose`，因此显示过的每一帧
都能在 `pose_realtime.json` 和 `key_frames/*.pcd` 中溯源。

按 `2` 后最终目录为：

```text
data/<名称>/
├── pose.json
├── pose_realtime.json
├── key_frames/*.pcd
├── manifest.json
├── events.jsonl
├── processing_summary.json
├── live/topoSingle_live.json
├── live/topoGraph_live.json
├── route3d_graph.json
├── route_targets_with_corners.txt
├── selected_route3d_graph.json
├── topoSingle_data.json
└── topoGraph_data.json
```

`live/topoSingle_live.json` 保留旧线性的实时预览结果；旧线性控制器使用停止后复核生成的
`topoSingle_data.json`。新 Dijkstra、切片和 PID 控制链使用 `topoGraph_data.json`，它是
智能拓扑的权威结果，允许 `[1,6]` 等非连续 ID 连边；`selected_route3d_graph.json` 从它
转换而来，会完整保留这些边供 RViz 最终检查。`processing_summary.json` 记录实时/离线
的节点数、边数、退回次数、闭环次数、
PCD 验证次数、抑制距离、拆边数和结构哈希一致性。构图器会从
`manifest.json.odom_frame` 继承坐标系；默认 RViz
Fixed Frame 是 `map`，通过可选的 `map -> camera_init` 静态 TF 显示定位轨迹。如果关闭
这条静态 TF，应把 RViz Fixed Frame 改为定位实际发布的 odom frame。

第二期闭环检测不会因单个空间近邻点立即连边。候选旧边必须与当前活动点的图距离至少
`3m`，XY/Z/方向分别满足 `0.40m/0.20m/45°`，再连续匹配至少 `0.80m`，并比较当前与
历史同步关键帧的局部体素重合率；默认重合率必须达到 `0.45`。确认后才创建
`source: loop_closure` 的边，后续重复绕圈只移动图游标、不新增节点。PCD 缺失时默认退化
为连续几何判定，可把 `loop_closure.pcd_validation.required` 设为 `true` 改成强制验证。
当前阶段仍不创建“虽然没有走过但由 PCD 推断可通行”的任意捷径边。已确认的闭环边可由
当前 C++ Dijkstra 搜索并交给新 PID 链执行；旧线性控制器仍不支持环图搜索。

#### 用测试 bag 查看实时效果

终端 A 先启动产品 launch，并在界面出现后按 `1`：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_product_demo live_route_product.launch.py
```

终端 B 再播放任一测试 bag。默认速率最容易看清青色退回边：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play /home/wei/bag/topo_1
# 或：ros2 bag play /home/wei/bag/topo_2
```

播放过程中无需额外启动 Nav2。bag 内的 `/lio_odom` 和
`/cloud_registered_body` 会由记录器同步，并发布 `/route3d/synchronized_pose` 给在线
拓扑节点。若想加速自动验收可以增加 `--rate 5.0`，但青色高亮会停留得更短。播放结束
后回到终端 A 按 `2`，输入一个未使用的目录名完成在线/离线一致性复核；最后按 `q` 退出。

不启动 RViz 的 bag 自动化测试入口：

```bash
ros2 run route3d_product_demo route3d_live_keyboard --no-rviz
```

### 原离线三键模式

先在一个终端启动 ROS 2 环境和演示程序：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run route3d_product_demo route3d_product_demo
```

程序只在终端读取单键：

```text
1  开始记录 /lio_odom 和 /cloud_registered_body，并写入临时目录
2  停止记录，完成 pose.json、key_frames/*.pcd、manifest.json
3  处理最近一次记录，输入名称后保存到 data/<名称>，并启动 RViz
q  退出并清理子进程
```

第三步会依次执行三维路线建图、等间隔采样与拐点提取、TopoSingle 转换。这个旧流程只
生成旧线性控制器的兼容输入：

```text
/home/wei/github_code/topo_graph_ws_/data/<名称>/topoSingle_data.json
```

它不会生成新 Dijkstra/PID 链所需的智能 `topoGraph_data.json`。新建图请使用上面的
`live_route_product.launch.py` 两键流程。

录 bag 测试时，终端 A 运行上面的演示程序；终端 B 播放已有 bag：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play /home/wei/bag/regu --clock
```

bag 开始播放后在终端 A 按 `1`，播放完成后按 `2`、`3`，输入例如 `regu_demo`。也可以在播放前按 `1`，记录器会等待对应话题。

可通过参数覆盖默认配置：

```bash
ros2 run route3d_product_demo route3d_product_demo \
  --target-spacing 1.0 --body-height 0.40 --sync-slop 0.05
```

注意：记录阶段不会减去机身高度；`pose.json` 的 z 是机身/传感器位姿高度，`--body-height` 只在路线处理、目标点和可视化阶段使用。

## 主要源码

- `src/route3d_bag_tools/scripts/route_corner_target_generator`：按间隔取点并加入局部原地转向拐点。
- `src/route3d_data_recorder`：实时同步记录里程计和点云，生成 `map_data` 目录。
- `src/topo_graph_tools/scripts/nav2_to_topo_single`：将带 `NORMAL/CORNER` 的目标文件转换为 topoSingle，并可生成用于可视化的 Nav2 子图。
- `src/topo_graph_tools/launch/topo_conversion_visualization.launch.py`：串行执行生成、转换和 RViz 可视化。
- `src/route3d_product_demo`：用 `1/2/3/q` 键盘按键控制记录、处理、保存和可视化。
- `src/route3d_online_skeleton`：从已落盘同步位姿增量生成在线拓扑和 RViz Marker。
- `src/route3d_topology_core`：在线与离线共用的退回判定、稳定 ID、拆边和图感知坡点核心。
- `src/route3d_dijkstra_planner`：纯 C++ 读取 Schema V2 智能图，执行 Dijkstra 并发布路径和 RViz Marker。
- `src/route3d_route_slicer`：纯 C++ 将最短路径按控制/业务语义切片，输出坡点步态与逐点通过约束。
- `src/route3d_pid_controller`：纯 C++ 跟踪 PID、终点低速精调、强制途经门和停障状态机，输出标准 `Twist`。
- `src/route3d_go2_adapter`：独占 Go2 SDK，限速转发 `vx/vy/wz`，并自动闭环执行坡段步态切换。
- `src/path_tracking_benchmark`：已纳入当前工作空间，用于后续固定路线跟踪评测。
- `data/regu/`：已复制的样例 Nav2 图、目标点和转换结果。

## 规划器的两种全局路径模式

`path_source:=topology` 使用产品生成的 TopoSingle JSON，通过点编号选择目标，并保留
`meta.isCorner` 的严格拐点到达逻辑：











































colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

自动记点
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_product_demo live_route_product.launch.py
```bash
ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=topology \
  route_file:=/home/wei/github_code/topo_graph_ws_/data/510/topoSingle_data.json \
  use_sim_time:=false

ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=topology \
  route_file:=/home/langyi/workspace/wyf/topo_graph_ws/data/510/topoSingle_data.json \
  use_sim_time:=false


```

发布目标编号：

```bash
ros2 topic pub --once \
  /recorded_waypoint_route/goal_id \
  std_msgs/msg/Int32 \
  "{data: 231}"
```

`path_source:=plan` 不启动 JSON 加载器和路线序列器，直接使用 PCTPlanner 发布的完整
地面 `nav_msgs/msg/Path`。局部规划器会从机器人在完整路径上的投影位置截取前视段：

```bash
ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=plan \
  global_plan_topic:=/plan \
  use_sim_time:=false
```

两种模式的输入都按地面高度处理，统一增加 `body_height`（默认 0.40 m）后再进行局部
三维规划。PCT 模式没有目标编号和 `isCorner` 状态机，路径几何拐角仍会作为 Guided A*
参考线。

### `use_sim_time` 的选择

`use_sim_time` 只决定 ROS 节点从哪里取得当前时间，与使用拓扑 JSON 还是 `/plan` 无关：

- 实机实时话题：使用 `use_sim_time:=false`，节点读取计算机系统时间。
- rosbag 或仿真：使用 `use_sim_time:=true`，并确保有 `/clock`；rosbag 必须加 `--clock`。
- 只加载 JSON 看静态路线、不播放 bag：使用 `false`。
- 拓扑 JSON 配合 bag 的点云和里程计测试：使用 `true`，同时用 `--clock` 播包。

如果设置为 `true` 却没有任何节点发布 `/clock`，ROS 时间会停在 0，带时间戳的 TF、
消息过滤和超时判断可能无法正常工作。规划、地图、控制和 RViz 应保持相同的
`use_sim_time` 设置。

PCT `/plan` 的 bag 测试：

```bash
ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=plan \
  global_plan_topic:=/plan \
  use_sim_time:=true
```

另一个终端播放 bag：

```bash
ros2 bag play /home/wei/bag/full_nav_replay_with_plan \
  --clock \
  --exclude-topics /tf_static
```



langyi@langyi:~/workspace/wyf/topo_graph_ws$ ros2 launch route3d_go2_adapter go2_pid_route.launch.xml   graph_file:=/home/langyi/workspace/wyf/topo_graph_ws/data/ceshi_1/topoGraph_data.json   network_interface:=enp2s0 enable_motion:=true

langyi@langyi:~/workspace/wyf/topo_graph_ws$ ros2 launch route3d_loop_patrol loop_patrol.launch.py
