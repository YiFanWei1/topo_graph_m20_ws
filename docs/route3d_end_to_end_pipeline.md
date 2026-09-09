# Route3D 从自动打点到 Go2 实机控制的完整逻辑

> 本文按当前工作空间 `/home/wei/github_code/topo_graph_ws_` 的源码编写，描述的是当前实际实现，
> 而不是未来设计。重点回答：自动点和特殊点怎样产生、最终点边文件怎样影响路径搜索、平地与
> 坡段怎样切换控制器和步态、速度最终怎样到达 Go2，以及网页目前接入到了哪一步。

## 1. 先看结论：系统不是一个节点，而是两条前后衔接的流水线

整套系统分为两个生命周期。

1. **拓扑生产期**：机器人由人工带着走，系统同步记录定位和点云，自动生成普通点、拐点、
   坡点、交汇点、分支和闭环，最后保存 `topoGraph_data.json`。
2. **导航执行期**：Dijkstra 在已经保存的拓扑图上搜索最短路，切片器把路径转换为带控制语义
   的任务，PID 或三维局部规划器输出速度，Go2 适配器选择唯一速度源并调用 Unitree SDK。

```text
┌────────────────────────────── 拓扑生产期 ──────────────────────────────┐
│ 雷达 + GLIO                                                          │
│   │ /lio_odom + /cloud_registered_body                              │
│   ▼                                                                  │
│ route3d_data_recorder                                                │
│   │ 同步位姿 + key frame PCD                                         │
│   ▼                                                                  │
│ route3d_online_skeleton + route3d_topology_core                      │
│   │ 普通点 / 拐点 / 坡点 / 回退抑制 / 分支 / 闭环                     │
│   ▼                                                                  │
│ 按 2 停止并离线复核 → topoGraph_data.json                            │
└──────────────────────────────────────────────────────────────────────┘
                                  │
                                  │ 人工检查、网页编辑、显式保存
                                  ▼
┌────────────────────────────── 导航执行期 ──────────────────────────────┐
│ route3d_dijkstra_planner                                             │
│   ▼ 最短路径的点 ID + 边 ID                                          │
│ route3d_route_slicer                                                 │
│   ▼ 平地 PID 任务 / 坡段 Effi 任务 / 步态转换                         │
│ route3d_pid_controller ──────┬──── /cmd_vel_pid                      │
│                              ├──── Effi path → /cmd_vel_effi         │
│                              └──── active_source                     │
│                                      ▼                               │
│ route3d_go2_adapter → Unitree SportClient::Move(vx, vy, wz) → Go2   │
└──────────────────────────────────────────────────────────────────────┘
```

这带来三个重要结论：

- 自动打点输出的不是一串供机器人立刻追踪的临时坐标，而是一张需要保存、检查并重新加载的图。
- `meta.isSlope` 标在**点**上；切片器据此判断相邻的**边**是不是坡段，再自动选择步态和控制器。
- PID 和 Effi 两套控制器可以同时常驻，但任何时刻只有 `/route3d_controller/active_source`
  指定的一个速度源能进入 Go2 SDK。

## 2. 功能包职责总表

| 功能包 | 所属阶段 | 当前职责 | 关键输入 | 关键输出 |
| --- | --- | --- | --- | --- |
| `route3d_product_demo` | 生产期编排 | 提供按键工作流，启动记录器、在线骨架和 RViz；停止后组织最终生成和保存 | 键盘 `1/2/3`、YAML | 各子进程、最终数据目录 |
| `route3d_data_recorder` | 生产期采集 | 近似同步里程计和机身点云；先写盘，再发布对应位姿 | `/lio_odom`、`/cloud_registered_body` | `pose.json`、`manifest.json`、`key_frames/*.pcd`、`/route3d/synchronized_pose` |
| `route3d_online_skeleton` | 生产期在线预览 | 消费已落盘位姿，在线维护线性骨架和智能拓扑，发布 RViz 图形 | `/route3d/synchronized_pose`、关键帧目录 | 两份 live JSON、事件日志、Path、Marker |
| `route3d_topology_core` | 生产期核心算法 | 增量点边、回退抑制、拆边、分支、闭环、PCD 闭环验证、Schema V2 默认值 | 位姿序列、关键帧 PCD | 智能拓扑内存模型/JSON |
| `route_slope_annotator` | 生产期语义 | 沿路线拟合高度变化，标记坡点和坡段缓冲区 | 带 XYZ 的点序列/图 | `meta.isSlope`、`slopeAnnotation` |
| `nav2_route3d` | 最终生成兼容步骤 | 从记录数据构造原始 Route3D 图 | `pose.json`、关键帧 | `route3d_graph.json/.geojson` |
| `route3d_bag_tools` | 最终生成兼容步骤 | 生成旧线性目标与 `topoSingle_data.json` | 原始图 | 目标文本、线性拓扑 |
| `topo_graph_tools` | 最终可视化 | 将智能拓扑映射成保留任意端点关系的可视化子图 | 原始图、智能图 | `selected_route3d_graph.json` |
| `route3d_web_console` | 地图/运维 | 选择 PCD，启动雷达/定位/控制，设置初始位姿，显示点云与位姿，编辑保存拓扑 | 浏览器 WebSocket、ROS | ROS 命令、进程状态、保存后的拓扑 |
| `route3d_dijkstra_planner` | 执行期全局搜索 | 严格加载 Schema V2，按边权和方向搜索最短路 | `[start_id, goal_id]` | Path、点 ID、边 ID、状态 JSON、Marker |
| `route3d_route_slicer` | 执行期语义编排 | 把最短路按控制器、步态、速度、障碍策略等属性切成任务 | Dijkstra 状态、同一拓扑文件 | `RouteTaskArray`、任务 JSON、Marker |
| `route3d_pid_controller` | 执行期调度/PID | 跟踪任务、平地 PID、停障、安全状态、发布 Effi 路径、控制器选择和步态请求 | 任务、里程计、机身点云、安全量 | `/cmd_vel_pid`、Effi 路径、`active_source`、步态请求 |
| `efficient_3d_local_planner` | 执行期坡段局部规划 | 三维体素地图、走廊 A*、B 样条和平滑跟踪 | 点云、里程计、坡段任务路径 | `/cmd_vel_effi` |
| `obstacle_occlusion_extension` | Effi 可选扩展 | 对遮挡空间做扩展；当前默认关闭 | 局部体素图、里程计 | 扩展点云/栅格 |
| `route3d_go2_adapter` | 实机输出 | 步态切换、速度源仲裁、限幅、超时归零、Unitree SDK 调用 | 两路速度、`active_source`、步态请求 | SDK `Move`、选中速度、状态、步态确认 |
| `go2_gait_switch_demo` | 独立测试 | 脱离导航链单独验证平地/楼梯步态 SDK 调用 | 测试参数 | Unitree 步态和旋转命令 |

其中 `circle_path_benchmark`、`path_tracking_benchmark`、`recorded_waypoint_route` 主要是测试、
基准或旧接口兼容，不是本文主生产链上的必经包。

## 3. 坐标系、话题和数据前置条件

### 3.1 当前使用的坐标关系

默认配置假定：

- 定位/拓扑全局坐标系：`camera_init`。
- 机身点云坐标系：`base_link`；现有 GLIO 话题名虽然带 `body`，当前配置按机器人局部坐标使用。
- 为兼容 RViz 和下游节点，可发布静态 TF：`map -> camera_init`。
- 可发布静态 TF：`body -> base_link`，平移为 `[-0.15, 0.0, -0.21]`。

如果现场已有节点发布同一父子 TF，必须关闭重复静态 TF，否则 TF 树会出现多发布者冲突。

### 3.2 为什么采集和控制用了不同里程计话题

- 自动打点配置使用 `/lio_odom`。
- 控制链使用高频 `/lio_odom_hf`。

前者与关键帧点云做同步并用于落盘；后者用于 50 Hz 控制闭环。部署时必须确认两个话题都
存在、时间戳连续且坐标一致。若现场定位只提供一个里程计话题，应在配置中统一修改，不能仅靠
名字猜测它们等价。

### 3.3 控制前必须满足

- 图的 `frame_id` 与里程计全局坐标系一致。
- `/cloud_registered_body` 的 XYZ 是机器人局部坐标，供 PID 高程停障使用。
- `topoGraph_data.json` 已保存且通过 Schema V2 校验。
- Dijkstra、切片器和网页必须使用同一份图文件。
- 实机时网卡名称正确，当前目标值为 `enp2s0`。
- `ROS_DOMAIN_ID` 和 DDS 配置在各终端一致；网页默认设为 `42`。

## 4. 阶段 A：启动自动打点工作流

入口是：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch route3d_product_demo live_route_product.launch.py
```

实际部署到机器人电脑后，应把第一行换成远端工作空间。入口配置为
[`live_route_product.yaml`](../src/route3d_product_demo/config/live_route_product.yaml)。

键盘行为：

| 按键 | 行为 |
| --- | --- |
| `1` | 创建 `/tmp/route3d_live_*` 会话目录，启动 RViz、在线骨架和记录器 |
| `2` | 停止采集，检查数据完整性，执行离线最终生成，询问 `data/<name>` 保存目录 |
| `3` | 正常退出但不执行最终生成；临时数据仍保留，便于排查 |

按 `1` 后，编排器先启动在线骨架，等待约 1.5 秒让 DDS 完成发现，再启动记录器，避免最早的
已提交帧没有在线订阅者。子进程都在独立进程组中；结束时先发 `SIGINT`，超时后再升级为
`SIGTERM/SIGKILL`。

## 5. 阶段 B：里程计与点云同步记录

核心源码：

- [`route3d_data_recorder_node.cpp`](../src/route3d_data_recorder/src/route3d_data_recorder_node.cpp)
- [`record_route.launch.py`](../src/route3d_data_recorder/launch/record_route.launch.py)

### 5.1 同步规则

记录器分别订阅：

```text
/lio_odom                  nav_msgs/msg/Odometry
/cloud_registered_body    sensor_msgs/msg/PointCloud2
```

默认队列深度为 `100`，最大时间差 `sync_slop_s=0.05 s`。它不是简单地各取最新消息，而是将
时间最接近的一对里程计和点云作为同一关键帧。

### 5.2 一帧提交的顺序

每个同步帧依次执行：

1. 校验预期 frame、有限坐标和四元数。
2. 归一化四元数。
3. 从 PointCloud2 中提取有限的 `x/y/z`。
4. 将点云写到临时文件，再原子改名为 `key_frames/<index>.pcd`。
5. 将位姿追加到 `pose.json.partial`。
6. 只有上述持久化成功后，才发布 `/route3d/synchronized_pose`。

因此在线骨架看到的每个位姿原则上都有对应的关键帧 PCD。这个顺序对闭环验证很重要：如果先
发布位姿再写盘，闭环候选可能引用一个尚不存在的 PCD。

### 5.3 会话文件

```text
/tmp/route3d_live_xxxxxx/
├── pose.json.partial        # 记录过程中写入
├── pose.json                # 正常停止时原子完成
├── manifest.json            # 话题、帧和同步统计
└── key_frames/
    ├── 0.pcd
    ├── 1.pcd
    └── ...
```

PCD 使用二进制格式并严格写 `x y z`。若输出目录已经有 `pose.json`、`manifest.json` 或关键帧，
记录器会拒绝覆盖，以免误删既有采集结果。

## 6. 阶段 C：普通点与在线骨架生成

核心源码：

- [`online_route_skeleton_node`](../src/route3d_online_skeleton/scripts/online_route_skeleton_node)
- [`topology.py`](../src/route3d_topology_core/route3d_topology_core/topology.py)

### 6.1 输入与实时输出

```text
输入：/route3d/synchronized_pose
输出：/route3d/live_raw_path
      /route3d/live_skeleton
      live/topoSingle_live.json
      live/topoGraph_live.json
      events.jsonl
```

`live_raw_path` 是完整灰色运动轨迹；`live_skeleton` 是点、边、候选拐点、坡点、闭环边和跳变点
组成的 MarkerArray。

### 6.2 普通点怎样产生

当前产品配置：

| 参数 | 值 | 含义 |
| --- | ---: | --- |
| `target_spacing` | `1.0 m` | 沿累计三维路程每隔约 1 m 插值一个拓扑点 |
| `dedup_distance` | `0.05 m` | 小于该位移视为静止抖动，不累计里程 |
| `relocation_distance` | `0.50 m` | 相邻定位超过该距离视为定位跳变 |
| `body_height` | `0.40 m` | JSON 地面 Z = 定位机身 Z - 0.40 m |

节点不是按消息频率生成，也不是从 PCD 中做语义分割得到。它们沿机器人真实走过的位姿折线按
距离生成，所以机器人走得快慢不会改变理想点间距。

停止采集时，最后一段即使不足 1 m，只要大于去重阈值，也会保留真实终点。

### 6.3 定位跳变

若相邻输入位姿的三维距离超过 `0.50 m`：

- 当前拐点窗口被结束；
- 创建新的 component；
- 在新位置建立起始点；
- 绝不连接跳变前后两点；
- 在 `events.jsonl` 记录 `pose_jump`，RViz 中显示对应标记。

这能避免定位重定位或漂移突变产生一条横穿空间的假边，但也意味着错误跳变会让图变成多个不
连通分量，Dijkstra 无法跨 component 规划。

## 7. 阶段 D：特殊点识别

当前自动识别的特殊点是：拐点、坡点，以及因拆边形成的交汇点。业务点类型（门、充电、换图
等）不是由几何算法自动猜测，通常需要人工或网页编辑 `meta.type/typeId`。

### 7.1 拐点识别

系统维护一个局部空间窗口：

- XY 半径不超过 `0.50 m`；
- 窗口内 Z 波动不超过 `0.10 m`；
- 将连续 yaw 展开，避免 `-π/π` 跳变；
- 累计转角超过 `45°`；
- 持续时间至少 `0.50 s`。

满足条件后，选择窗口时间中点附近的真实位姿作为代表拐点，并写：

```json
"meta": {
  "isCorner": true,
  "turnDeg": 87.3
}
```

`0.35 m` 内的候选会合并，两个独立拐点沿路程默认至少相隔 `1.0 m`。Schema 默认再令拐点：

```json
"mustPassThrough": true,
"passRadiusM": 0.20
```

所以拐点对控制的主要影响是“必须穿过较小的门”，防止跟踪器为了前视而切弯；它本身通常不
直接形成任务切片边界。

### 7.2 坡点识别

核心源码为 [`slope_analysis.py`](../src/route_slope_annotator/route_slope_annotator/slope_analysis.py)。

算法将路线按累计 XY 距离参数化，在局部窗口中拟合：

```text
z = grade × s + intercept
```

当前产品阈值：

| 参数 | 值 |
| --- | ---: |
| 拟合半径 | `2.0 m` |
| 坡度阈值 | `abs(grade) >= 0.12` |
| 核心坡段最短长度 | `2.0 m` |
| 核心坡段最小高差 | `0.20 m` |
| 同方向候选最大合并间隙 | `1.0 m` |
| 坡前坡后缓冲 | `1.0 m` |

有效坡段及前后缓冲区中的点写入 `meta.isSlope=true`。图模式下会先找最大非分支链分别计算，
同一个 junction 从多条链获得的坡标记采用 OR 合并。

注意：源码类的 `buffer_distance` 缺省值是 3 m，但当前产品 YAML 显式配置成 1 m，实际运行以
YAML 为准。

### 7.3 交汇点

当机器人沿旧边返回后从边中间离开，系统会：

1. 确认离开旧走廊；
2. 在离开投影位置创建或吸附一个点；
3. 删除原边并拆为两条边；
4. 给该点设置 `meta.isJunction=true`；
5. 从 junction 向新区域继续建立 `discovery` 分支。

已有顶点 ID 不重排。旧边被替换后边 ID 可以出现空洞，这是正常结构，不能用数组下标代替 ID。

## 8. 阶段 E：回退抑制、旧路重入与闭环

`IncrementalTopologyBuilder` 的主要状态为：

```text
DISCOVERING
   │ 发现相邻旧边/远端旧边候选
   ▼
RETRACE_PENDING ──连续确认──> RETRACING
   │ 候选失效/PCD拒绝             │ 偏离走廊
   └──────────> DISCOVERING        ▼
                               EXIT_PENDING
                                  │ 确认离开
                                  └──> DISCOVERING（建新分支）
```

### 8.1 原路退回

原路退回只匹配当前活动位置相邻的旧边，不吸附任意空间近邻，避免把并排但不同的走廊错误合并。

当前阈值：

- XY 走廊容差 `0.30 m`，退出容差 `0.45 m`；
- Z 容差 `0.20 m`；
- 运动方向误差 `35°`；
- 进入和离开都需累计确认 `0.30 m`；
- 顶点吸附距离 `0.30 m`；
- 低于 `0.10 m` 的单次运动不用于可靠方向判断。

确认处于旧边后，系统移动图上的活动游标而不重复创建点边，并累计
`generation.suppressed_distance_m`。

### 8.2 全局旧路重入与闭环

闭环允许匹配图距离足够远的旧边。几何候选必须同时满足：

- XY 走廊距离不超过 `0.40 m`；
- Z 误差不超过 `0.20 m`；
- 方向误差不超过 `45°`；
- 连续匹配距离达到 `0.80 m`；
- 与当前位置的图距离至少 `3.0 m`；
- 与历史点的时间间隔至少 `5.0 s`。

确认后可能在当前路径和旧边上分别拆边，再添加 `source=loop_closure` 的连接边。

### 8.3 PCD 闭环复核

核心源码为 [`pcd_validation.py`](../src/route3d_topology_core/route3d_topology_core/pcd_validation.py)。

验证流程：

1. 找当前候选和历史候选各自对应的同步关键帧。
2. 用各自位姿将局部点云变换到世界坐标。
3. 过滤距离到 `[0.5, 12.0] m`。
4. 以 `0.30 m` 体素降采样，每帧至少需要 100 个有效体素。
5. 在 XY `±0.60 m`、Z `±0.30 m` 内搜索小平移。
6. 计算 `交集体素数 / min(两帧体素数)`。
7. 重合率至少 `0.45` 才接受闭环。

当前 `required=false`：关键帧偶发缺失时允许仅凭连续几何条件退化确认；但如果 PCD 存在且明确
算出重合率不足，候选仍会被拒绝。

## 9. 按 2 后为什么还要再生成一次

在线结果用于实时反馈，最终结果由完整数据离线重放生成。按 `2` 后顺序是：

1. 停记录器，等待 `pose.json` 和 PCD 原子落盘。
2. 停在线骨架。
3. 检查至少有两帧 PCD，且 live 两份 JSON 存在。
4. 复制 `pose.json` 为 `pose_realtime.json`。
5. 运行 `nav2_route3d/route_graph_builder_3d`。
6. 运行 `route3d_bag_tools/route_corner_target_generator`。
7. 运行 `route3d_topology_core/build_topology_graph`，用相同参数完整重放智能状态机。
8. 运行 `topo_graph_tools/nav2_to_topo_single` 生成可视化子图。
9. 对兼容线性图重新做坡度标注。
10. 生成统计和结构 SHA256，移动到用户指定的 `data/<name>`。

最终目录的主要文件：

| 文件 | 定位 | 是否用于当前控制链 |
| --- | --- | --- |
| `topoGraph_data.json` | 智能图最终权威文件 | **是** |
| `topoSingle_data.json` | 旧线性控制器兼容文件 | 否，除非显式使用旧流程 |
| `live/topoGraph_live.json` | 在线智能图快照 | 调试对照 |
| `live/topoSingle_live.json` | 在线线性预览 | 调试/兼容 |
| `route3d_graph.json` | 原始 Route3D 图 | 中间产物 |
| `selected_route3d_graph.json` | 最终 RViz 子图 | 可视化 |
| `processing_summary.json` | live/final 计数、闭环、拆边、一致性摘要 | 验收 |
| `events.jsonl` | 跳变、候选、闭环等事件 | 故障分析 |
| `pose.json`、`key_frames/` | 原始同步数据 | 可重放和复核 |

控制链应读取 `topoGraph_data.json`。在线和最终图不一致时，以最终离线重放为权威，同时利用
`processing_summary.json` 分析差异。

## 10. Schema V2：点和边怎样影响后续控制

完整字段说明另见 [`topology_schema_v2_interface.md`](topology_schema_v2_interface.md)。这里重点
说明从文件到控制的因果关系。

### 10.1 顶点控制字段

| 字段 | 当前含义 | 下游作用 |
| --- | --- | --- |
| `pos` | 地面 XYZ | Path 和跟踪目标 |
| `rpy[2]` | yaw | 最终姿态对齐、路径姿态 |
| `acc` | 作为任务最终点时的容差 | 切片任务 `endpoint_tolerance_m` |
| `alignFinalYaw` | 最终点是否对齐 yaw | 只在路线终点/业务停车点启用 |
| `mustPassThrough` | 是否为硬途经门 | 防止前视控制切过中间点 |
| `passRadiusM` | 中间途经门半径 | 拐点默认 0.20 m，普通点默认 0.45 m |
| `meta.isCorner` | 转角或平地交汇点；与坡点互斥 | 默认令其必须穿过；还触发接近拐点减速 |
| `meta.isSlope` | 坡点，优先于拐点语义 | 令与该点相邻的路径边成为坡段，并强制 `isCorner=false` |
| `meta.isJunction` | 分支/拆边点；平地时同时标成拐点 | 拓扑语义和可视化，不直接切控制器 |
| `meta.type/typeId` | 业务类型 | 点对组合可形成门、充电、换图等业务任务 |

### 10.2 边控制字段

| 字段 | 当前含义 | 下游作用 |
| --- | --- | --- |
| `v` | 两个端点 ID | 图连接关系 |
| `weight` | Dijkstra 代价 | 最短路不是按几何距离重算，而直接使用该值 |
| `meta.travelMode` | 双向/首到次/次到首 | 决定有向邻接；逆向请求可能不可达 |
| `meta.controllerMode` | `auto` 或显式控制器 | `auto` 时平地 PID、坡段 Effi |
| `meta.locomotionMode` | 基础运动模式 | 0 为普通，坡段可被自动提升为 2 |
| `meta.linearSpeedMps` | 边速度上限/期望 | PID 再受全局 0.8 m/s 上限限制 |
| `meta.angularSpeedRadps` | 边角速度属性 | 进入任务并参与切片 |
| `meta.obstacleMode` | 障碍策略 | 决定停车、重规划、忽略普通坡面等行为 |
| `meta.obstacleBoxM` | 自定义局部警戒框 | 非零有效框覆盖默认 Go2 警戒足迹 |
| `meta.gridMapName` | 栅格地图引用 | 非空时 `auto` 可解析为网格局部控制器 |
| `rotationAllowed` | 是否允许转向 | 属性变化会形成切片边界 |

### 10.3 “步态是点属性还是边属性”的准确答案

当前自动逻辑是两层：

1. 自动坡度分析把 `isSlope` 写在点上。
2. 切片器遍历实际路径边，只要该边任一端点 `isSlope=true`，就认为
   `contains_slope=true`。

随后，若边仍是默认配置：

```text
平地边：controller=pid
        locomotionMode=0
        gait_command=static_walk
        obstacleMode=0

坡段边：controller=efficient_3d_local_planner
        locomotionMode=2
        gait_command=switch_gait_3
        obstacleMode=3（仅当原值为 0 时自动改）
```

如果边显式写了非 `auto` 的 `controllerMode`、非默认 `locomotionMode` 或非默认
`obstacleMode`，显式设置优先。也就是说，**坡点是检测结果，边属性是最终控制策略，切片器负责
把两者合成任务**。

## 11. 网页拓扑编辑位于流程的什么位置

`route3d_web_console` 当前适合放在“最终图生成后、执行导航前”。它把以下内容转发给浏览器：

- 静态定位 PCD 的多级降采样数据；
- `/cloud_registered` 实时点云；
- `/lio_odom_hf` 实时位置和轨迹；
- 拓扑点边与路径/状态；
- 雷达、定位、规划控制子进程日志。

网页可选择绑定的 PCD、定位模板和拓扑 JSON，设置 `/initialpose`，编辑点边属性并显式保存。
保存采用校验、临时文件、原子替换和历史备份；规划链运行时禁止保存，因为 Dijkstra 只在节点
启动时加载图。

### 11.1 当前网页已实现

- 地图目录与 PCD/定位配置/拓扑绑定；
- 雷达、定位和 `go2_pid_route.launch.xml` 子进程启停；
- 静态 PCD、实时点云、机器人位姿和轨迹 WebSocket 可视化；
- 点击拖动发布带 yaw 的 `/initialpose`；
- 拓扑加载、编辑、保存；
- 起终点规划，PID 暂停、恢复、取消；
- 单浏览器控制租约、3 秒心跳丢失自动 cancel；
- `enable_motion=false` 默认 dry-run，开启实机运动需要再次确认。

### 11.2 当前网页尚未接入

- “开始自动打点”按钮；
- `/route3d/live_raw_path` 和 `/route3d/live_skeleton` 的在线构图展示；
- “停止并生成最终拓扑”及保存目录命名；
- 将新生成 PCD/拓扑自动登记到 `map_catalog.json`。

因此当前客户若要自动打点，仍需用终端启动 `route3d_product_demo`；网页负责已有 PCD 定位、
图编辑和导航控制。不能把网页中“新增一个点”理解为自动构图算法已经集成到网页。

网页的实现和部署说明见 [`route3d_web_console/README.md`](../src/route3d_web_console/README.md)。

## 12. 阶段 F：Dijkstra 最短路径

核心源码：

- [`dijkstra_planner_node.cpp`](../src/route3d_dijkstra_planner/src/dijkstra_planner_node.cpp)
- [`topology_graph.cpp`](../src/route3d_dijkstra_planner/src/topology_graph.cpp)

Dijkstra 节点启动时严格加载一次 Schema V2。请求格式：

```bash
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [1, 4]}"
```

处理过程：

1. 校验数组正好包含起点和终点 ID。
2. 根据每条边 `travelMode` 建立有向邻接表。
3. 使用边的 `weight` 作为代价运行优先队列 Dijkstra。
4. 回溯出完整顶点 ID 和边 ID。
5. 发布 latched/transient-local 结果，供晚启动的切片器和 RViz 读取。

主要输出：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/route3d_dijkstra/path` | `nav_msgs/msg/Path` | 最短路几何路径 |
| `/route3d_dijkstra/path_vertex_ids` | `Int32MultiArray` | 顶点 ID 序列 |
| `/route3d_dijkstra/path_edge_ids` | `Int32MultiArray` | 边 ID 序列 |
| `/route3d_dijkstra/markers` | `MarkerArray` | 全图和选中路径 |
| `/route3d_dijkstra/status` | `String` JSON | 请求、代价、耗时和路径 |

拓扑保存后必须重启规划链。否则浏览器看到的是新图，Dijkstra 内存中仍是旧图。

## 13. 阶段 G：路径语义切片

核心源码：

- [`route_slicer.cpp`](../src/route3d_route_slicer/src/route_slicer.cpp)
- [`RouteTask.msg`](../src/route3d_route_slicer/msg/RouteTask.msg)
- [`route_slicer.yaml`](../src/route3d_route_slicer/config/route_slicer.yaml)

Dijkstra 只回答“走哪些点和边”，切片器回答“每一段如何走”。相邻边出现以下任一差异就结束
当前任务并创建下一任务：

- 配置/解析后的控制器不同；
- locomotion mode 或步态命令不同；
- 线速度、角速度、高度偏置不同；
- 障碍模式或警戒框不同；
- grid map 不同；
- 转向许可不同；
- 立即重复同一条边；
- 遇到独立障碍任务或业务点组合。

任务中保留：waypoint、边 ID、控制器、步态、速度、障碍策略、是否坡段、是否反向、终点容差、
是否对齐 yaw、切片原因等完整信息。

### 13.1 停车边界

- 第一任务一定显式请求目标步态，因为系统不能假设机器人启动前处于哪种模式。
- 后续 locomotion mode 改变时请求步态切换。
- 控制器、步态或业务任务发生硬切换前，前一任务 `requires_stop_at_end=true`。
- 路线最终任务一定停车并执行精确终点验收。

这就是为什么日志中会先看到 `WAITING_TRANSITION`，收到 Go2 步态确认后才会看到
`TRACKING`。这是设计内的安全握手，不是 PID 没有默认开启。

## 14. 阶段 H：PID 跟踪、任务调度和安全停障

核心源码：

- [`pid_controller_node.cpp`](../src/route3d_pid_controller/src/pid_controller_node.cpp)
- [`controller_core.cpp`](../src/route3d_pid_controller/src/controller_core.cpp)
- [`elevation_collision_checker.cpp`](../src/route3d_pid_controller/src/elevation_collision_checker.cpp)
- [`go2_pid.yaml`](../src/route3d_pid_controller/config/go2_pid.yaml)

### 14.1 控制器状态

```text
IDLE → READY → WAITING_TRANSITION → TRACKING → FINISHED
                    │                  │
                    │                  ├─ PAUSED
                    │                  ├─ safety stop（状态内零速，条件清除后恢复）
                    │                  └─ 下一任务/下一次步态切换
                    ├─ HANDOVER_REQUIRED（未知控制器）
                    ├─ CANCELLED
                    └─ ERROR
```

`execution.auto_start=true`：收到新的 `RouteTaskArray` 后自动激活第一个任务。但如果第一任务要求
步态切换，自动启动的第一步是进入 `WAITING_TRANSITION`，并非立即发速度。

### 14.2 平地 PID 跟踪

跟踪器对 `x/y/yaw` 分别使用 PID：

- 前视距离 `0.60 m`；
- 最大 `vx=0.80 m/s`；
- 常规最大 `vy=0.25 m/s`；
- 最大 `wz=0.80 rad/s`；
- 最大线加速度 `0.50 m/s²`；
- 最大角加速度 `1.20 rad/s²`；
- 拐点前 `0.70 m` 开始约束，拐点速度 `0.20 m/s`；
- yaw 误差超过 `1.05 rad` 时停止平移，优先转向。

边的 `linearSpeedMps` 只能降低任务速度，不能突破全局 `0.8 m/s`。

进入最终点 `0.30 m` 内转入精调阶段，保留 `vx/vy/wz` 三自由度低速修正：

- 精调最大 `vx=0.20 m/s`；
- 精调最大 `vy=0.20 m/s`；
- 精调最大 `wz=0.90 rad/s`；
- 最终位置容差 `0.10 m`；
- 最终 yaw 容差 `0.15 rad`。

因此“普通路段没有横移”与“最终到点可横移精调”并不矛盾。Effi follower 当前则配置为
`max_vy=0`。

### 14.3 平地障碍检测

PID 从 `/cloud_registered_body` 建立机器人局部高程图：

1. 使用 `4.0 m × 4.0 m` 区域，分辨率 `0.10 m`。
2. 忽略机身自身区域 `x=[-0.50,0.30]`、`y=[-0.25,0.25]`。
3. 只接受局部 Z 在 `[-0.20,0.40] m` 范围的点。
4. 每格保留代表高度。
5. 只用真实观测格计算每个 3×3 邻域的高度标准差。
6. 标准差大于 `0.125 m` 的格子标为 rough/障碍。
7. 沿剩余轨迹采样 Go2 警戒足迹，足迹落入 rough 格即碰撞。

未观测格不会拿默认地面高度参与标准差，否则稀疏点云边缘会在平地上制造假高度突变。

默认警戒足迹：

```text
x: 0.05 .. 0.60 m
y: -0.25 .. 0.24 m
5 × 5 采样，侧边内缩 0.13 m
```

默认路径检查最多 15 个姿态、间隔 `0.40 m`。当前障碍策略：

| `obstacleMode` | 行为 |
| ---: | --- |
| `0` | 本体或剩余轨迹检测到高程碰撞就停车，清除后自动恢复 |
| `1` | 近处碰撞停车并请求重规划；远处碰撞只发重规划请求 |
| `3` | 忽略普通高程障碍，供楼梯/坡面通过 |
| `4` | 交给带 grid map 的其他控制策略；PID 不作为该任务控制器 |

即使是模式 3，以下硬安全条件仍会停车：

- 外部安全停止为真；
- `/collision_level >= 100`；
- 里程计超过 `0.30 s` 未更新。

需要高程检测的任务还要求点云不超过 `0.50 s`、点云可解析且高程图 ready。安全停车时发布
零速、清 PID 状态并把 `active_source` 设为 `none`；条件清除后再恢复原任务控制器。

RViz 调试话题 `/route3d_pid_controller/elevation_debug` 使用 best-effort QoS，包含高程格、rough
格、检查轨迹和警戒范围。RViz 若使用 reliable 会提示 QoS 不兼容且看不到数据，应将该 Display
的 Reliability 改成 Best Effort。

## 15. 阶段 I：坡段 Effi 三维局部规划

`go2_pid_route.launch.xml` 默认让以下节点常驻：

```text
/cloud_registered_body + /lio_odom_hf
          │
          ▼
local_voxel_mapper_node
          │ 约 0.1 m 分辨率、8×8×5 m 滚动三维体素图
          ▼
corridor_astar_planner_node
          │ 受拓扑任务约束的局部 3D A* + B 样条
          ▼
/local_planner/local_path
          ▼
local_path_follower_node
          ▼
/cmd_vel_effi
```

PID 调度节点在坡任务激活时做三件事：

1. 给 PID 自己发布零速；
2. 将当前任务的 Path 发布到 `/route3d_controller/efficient_path`；
3. 将 `/route3d_controller/active_source` 设为 `efficient_3d_local_planner`。

launch 将 Effi 原来的输入路径重映射到该任务路径；follower 原本的 `/cmd_vel_smoothed` 被重映射
为 `/cmd_vel_effi`。

局部 A* 优先在 hard 和 soft 障碍之外搜索；失败时允许以代价穿越 soft 层。默认局部前视约
`2.8 m`，B 样条失败还有窄通道/楼梯重试。当前 follower 上限 `vx=0.8 m/s`、`vy=0`。

`obstacle_occlusion_extension` 进程虽随 launch 启动，但当前配置 `enabled=false`，不会执行硬注入。

## 16. 阶段 J：步态切换握手

步态不是靠发送一次零速度自动切出来的，而是显式 SDK 调用：

| 任务步态命令 | Go2 调用 |
| --- | --- |
| `static_walk` | `SportClient::StaticWalk()` |
| `switch_gait_3` | legacy API 1011，参数 `{"data":3}` |

握手时序：

```text
route_slicer
   │ task.requires_gait_switch_at_start=true
   ▼
pid_controller: active_source=none, 发布零速, WAITING_TRANSITION
   │ /route3d_pid_controller/gait_transition
   ▼
go2_adapter: STOPPING
   │ StopMove()；失败时回退 Move(0,0,0)
   ▼
调用 StaticWalk() 或 SwitchGait(3)
   ▼
SETTLING：等待至少 2 s + 新鲜稳定 SportModeState 样本
   ▼
发布 /route3d_pid_controller/gait_acknowledged
   ▼
pid_controller 激活任务并选择 pid 或 efficient_3d_local_planner
```

当前实机固件上 `SportModeState.gait_type` 在 `SwitchGait(3)` 后仍可能保持 0，因此适配器不再把
“反馈 gait_type 必须等于 3”作为成功条件。当前判定是：SDK 调用返回成功、等待稳定时间、收到
至少配置数量的新鲜稳定 SportModeState 样本。

`StopMove()` 的作用是切换前安全停止，不是选择楼梯步态。它在该固件可能返回 `-1`，此时适配器
用 `Move(0,0,0)` 回退；真正选择楼梯步态的仍是 legacy `SwitchGait(3)`。

## 17. 阶段 K：速度源仲裁与实机输出

Go2 适配器同时订阅：

```text
/cmd_vel_pid
/cmd_vel_effi
/route3d_controller/active_source
```

合法来源只有：

```text
none
pid
efficient_3d_local_planner
```

选择逻辑：

| `active_source` | 实际转发 |
| --- | --- |
| `none` | 持续零速度 |
| `pid` | 只转发最新 `/cmd_vel_pid` |
| `efficient_3d_local_planner` | 只转发最新 `/cmd_vel_effi` |

切换来源时会清除两路旧缓存，防止新控制器刚激活就发出旧速度。命令超过 `0.10 s` 未更新也自动
归零。最终限幅为：`vx=0.8 m/s`、`vy=0.4 m/s`、`wz=1.2 rad/s`，并限制平面合速度。

选中速度同步发布到 `/route3d_go2_adapter/selected_command` 便于网页和命令行观察。只有
`enable_motion=true` 时才打开 Unitree 通道并执行：

```cpp
sport_client_->Move(vx, vy, wz);
```

`enable_motion=false` 时完整规划、切片、步态握手和速度仲裁仍能 dry-run，但机器人不会动。

## 18. 一次“平地 → 坡段 → 平地”的完整时序

假设路径的点坡度标记为：

```text
1(normal) -- 2(slope) -- 3(slope) -- 4(normal) -- 5(normal)
```

由于“任一端点为坡点就算坡边”，边 1-2、2-3、3-4 都会进入坡任务，4-5 才回到普通任务。

1. Dijkstra 输出 `1→2→3→4→5`。
2. 切片器形成坡任务 `1→4` 和 PID 任务 `4→5`。
3. 第一任务要求 `switch_gait_3`，PID 进入 `WAITING_TRANSITION`。
4. Go2 停稳，调用 `SwitchGait(3)`，发布确认。
5. PID 调度器发布坡任务 Path，选择 Effi。
6. Effi 输出 `/cmd_vel_effi`，适配器转发给 Go2。
7. 到点 4，先停止并把来源设为 `none`。
8. 请求 `static_walk`，适配器调用 `StaticWalk()` 并确认。
9. PID 激活 4→5，输出 `/cmd_vel_pid`。
10. 最终进入精调，位置/yaw 均满足后状态变为 `FINISHED` 并归零。

这个规则意味着坡度标记带缓冲是有意的：机器人在真正坡面前就切到楼梯步态，在完全离开坡面后
才切回普通步态。

## 19. 导航控制启动方式

### 19.1 先 dry-run

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch route3d_go2_adapter go2_pid_route.launch.xml \
  graph_file:=/home/wei/github_code/topo_graph_ws_/data/building_a_v1/topoGraph_data.json \
  network_interface:=enp2s0 \
  enable_motion:=false \
  use_sim_time:=false \
  launch_rviz:=true
```

另一个已 source 相同环境的终端发请求：

```bash
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [1, 4]}"
```

### 19.2 实机

确认定位稳定、图与 PCD 对应、遥控急停就绪后，仅将：

```text
enable_motion:=true
```

实机启动日志必须出现：

```text
REAL GO2 OUTPUT ENABLED on enp2s0
```

如果出现 `DRY RUN`，即使 `/cmd_vel_pid` 或 `/cmd_vel_effi` 有速度，机器人也不会动。

## 20. 运行时话题与服务速查

### 20.1 自动打点

| 名称 | 类型 | 方向/用途 |
| --- | --- | --- |
| `/lio_odom` | `nav_msgs/Odometry` | 记录器输入 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 记录器同步输入 |
| `/route3d/synchronized_pose` | `geometry_msgs/PoseStamped` | 已落盘同步位姿 |
| `/route3d/live_raw_path` | `nav_msgs/Path` | 完整轨迹 |
| `/route3d/live_skeleton` | `visualization_msgs/MarkerArray` | 在线拓扑和特殊点 |

### 20.2 搜索与切片

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/route3d_dijkstra/plan_request` | `std_msgs/Int32MultiArray` | `[start, goal]` |
| `/route3d_dijkstra/path` | `nav_msgs/Path` | 最短路径 |
| `/route3d_dijkstra/status` | `std_msgs/String` | 搜索状态 JSON |
| `/route3d_route_slicer/tasks` | `RouteTaskArray` | 控制任务主接口 |
| `/route3d_route_slicer/tasks_json` | `std_msgs/String` | 可读任务 JSON |
| `/route3d_route_slicer/status` | `std_msgs/String` | 切片状态 |

### 20.3 控制与实机

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/lio_odom_hf` | `nav_msgs/Odometry` | 控制反馈 |
| `/cloud_registered_body` | `PointCloud2` | PID 高程停障/Effi 地图 |
| `/cmd_vel_pid` | `geometry_msgs/Twist` | PID 候选速度 |
| `/cmd_vel_effi` | `geometry_msgs/Twist` | Effi 候选速度 |
| `/route3d_controller/efficient_path` | `nav_msgs/Path` | 当前坡任务给 Effi 的走廊路径 |
| `/route3d_controller/active_source` | `std_msgs/String` | 唯一速度源选择 |
| `/route3d_go2_adapter/selected_command` | `geometry_msgs/Twist` | 适配器实际选中速度 |
| `/route3d_pid_controller/gait_transition` | `GaitTransition` | 步态请求 |
| `/route3d_pid_controller/gait_acknowledged` | `GaitTransition` | 步态完成确认 |
| `/route3d_pid_controller/status` | `std_msgs/String` | PID/调度状态 JSON |
| `/route3d_go2_adapter/status` | `std_msgs/String` | SDK、步态和速度源状态 JSON |
| `/route3d_pid_controller/elevation_debug` | `MarkerArray` | 高程图与检测范围 |
| `/route3d_pid_controller/pause` | `std_srvs/Trigger` | 暂停并归零 |
| `/route3d_pid_controller/resume` | `std_srvs/Trigger` | 恢复当前任务 |
| `/route3d_pid_controller/cancel` | `std_srvs/Trigger` | 取消整条路线并清 Effi Path |
| `/route3d_go2_adapter/emergency_stop` | `std_srvs/Trigger` | 锁存急停，需重启适配器清除 |

## 21. 故障定位：先看哪一层

### 21.1 发了规划请求，但没有任务

依次检查：

```bash
ros2 topic echo --once /route3d_dijkstra/status
ros2 topic echo --once /route3d_route_slicer/status
ros2 topic echo --once /route3d_route_slicer/tasks_json
```

常见原因是点 ID 不存在、方向不允许、图不连通、Schema V2 无效，或规划节点仍加载旧图。

### 21.2 PID 有速度，但机器人不动

不能只看 `/cmd_vel_pid`。继续检查：

```bash
ros2 topic echo /route3d_controller/active_source
ros2 topic echo /route3d_go2_adapter/selected_command
ros2 topic echo --once /route3d_go2_adapter/status
```

若 `active_source=none`，通常正在步态切换、安全停车、暂停或已取消。若 selected command 有速度
但机器人不动，检查 `enable_motion`、网卡、SDK 返回码和是否有另一个程序争用 Unitree 通道。

### 21.3 Effi 被选中但不走

检查：

```bash
ros2 topic echo --once /route3d_controller/efficient_path
ros2 topic hz /cmd_vel_effi
ros2 topic echo /cmd_vel_effi
ros2 node list | grep -E 'voxel|astar|follower'
```

只有 active source 切换成功还不够；Effi 必须收到非空任务 Path、有效里程计和局部体素地图，
follower 才会持续输出新鲜速度。适配器 0.10 s 超时会把间断输出变成零速。

### 21.4 平地走走停停

查看：

```bash
ros2 topic echo /route3d_pid_controller/status
ros2 topic echo /route3d_pid_controller/obstacle_stop
ros2 topic hz /cloud_registered_body
```

重点区分 `elevation-map trajectory collision`、`obstacle cloud timeout`、
`elevation map unavailable` 和 `odometry timeout`。前者再用 RViz 的 elevation debug 判断是否是
真实 rough 格或点云坐标/高度范围不对。

### 21.5 一直等待步态

同时看 PID 和适配器状态。正常序列应为：

```text
PID WAITING_TRANSITION
Adapter STOPPING → SETTLING → PUBLISHING_ACKNOWLEDGEMENT → IDLE
PID TRACKING
```

如果适配器进入 ERROR，检查 SDK 调用、`rt/sportmodestate` 是否新鲜、DDS/RMW 是否正确，以及
是否有另一个 gait demo 或控制适配器同时占用机器人。

## 22. 安全约束和当前已知边界

- 自动拓扑只表示机器人实际走过的可通行路线，不会从大地图 PCD 自动推断未走过的捷径。
- 自动识别几何拐点、坡点和 junction，不自动理解门、充电位、换图点等业务语义。
- PCD 闭环验证是局部体素重合检查，不是 ICP 回环优化，也不会修正整条历史轨迹。
- 定位跳变会主动断图；错误跳变需先解决定位或人工修图。
- Dijkstra 启动时一次性加载图，不支持保存后的热重载。
- 坡边默认 `obstacleMode=3` 是为了不把坡面/楼梯本身当障碍，不代表取消硬急停。
- 适配器是最终唯一 SDK 所有者。不要同时运行旧 `robot_control_adapter`、步态 demo 和正式适配器。
- 网页控制租约是误操作保护，不是互联网级身份认证；当前只适用于可信局域网。
- 当前网页还没有把自动打点编排接进来，客户的一体化流程仍差这一段 UI/后台接口。
- 所有实机首次测试都应先 `enable_motion=false` 验证图、任务、步态和选中速度，再开启真实运动。

## 23. 关键源码索引

### 23.1 自动打点与最终生成

- [`route3d_live_keyboard`](../src/route3d_product_demo/scripts/route3d_live_keyboard)：按键编排和最终产物。
- [`live_route_product.launch.py`](../src/route3d_product_demo/launch/live_route_product.launch.py)：入口与 TF。
- [`live_route_product.yaml`](../src/route3d_product_demo/config/live_route_product.yaml)：当前自动识别参数。
- [`route3d_data_recorder_node.cpp`](../src/route3d_data_recorder/src/route3d_data_recorder_node.cpp)：同步和原子记录。
- [`online_route_skeleton_node`](../src/route3d_online_skeleton/scripts/online_route_skeleton_node)：在线拐点、坡点与可视化。
- [`topology.py`](../src/route3d_topology_core/route3d_topology_core/topology.py)：智能图状态机。
- [`pcd_validation.py`](../src/route3d_topology_core/route3d_topology_core/pcd_validation.py)：闭环 PCD 复核。
- [`schema.py`](../src/route3d_topology_core/route3d_topology_core/schema.py)：Schema V2 默认值。

### 23.2 规划、控制和实机

- [`dijkstra_planner_node.cpp`](../src/route3d_dijkstra_planner/src/dijkstra_planner_node.cpp)：规划请求和结果发布。
- [`route_slicer.cpp`](../src/route3d_route_slicer/src/route_slicer.cpp)：控制语义解析和任务切片。
- [`route_slicer.yaml`](../src/route3d_route_slicer/config/route_slicer.yaml)：坡段控制器和步态策略。
- [`pid_controller_node.cpp`](../src/route3d_pid_controller/src/pid_controller_node.cpp)：任务状态机、安全和控制器调度。
- [`controller_core.cpp`](../src/route3d_pid_controller/src/controller_core.cpp)：PID 路径跟踪和最终精调。
- [`elevation_collision_checker.cpp`](../src/route3d_pid_controller/src/elevation_collision_checker.cpp)：高程图停障。
- [`go2_pid.yaml`](../src/route3d_pid_controller/config/go2_pid.yaml)：PID、限速和停障参数。
- [`go2_route_adapter_node.cpp`](../src/route3d_go2_adapter/src/go2_route_adapter_node.cpp)：步态、仲裁和 SDK。
- [`go2_route_adapter.yaml`](../src/route3d_go2_adapter/config/go2_route_adapter.yaml)：适配器参数。
- [`go2_pid_route.launch.xml`](../src/route3d_go2_adapter/launch/go2_pid_route.launch.xml)：完整执行链入口。

### 23.3 网页

- [`web_console_node.cpp`](../src/route3d_web_console/src/web_console_node.cpp)：ROS/WebSocket/进程管理入口。
- [`web_console.yaml`](../src/route3d_web_console/config/web_console.yaml)：现场路径、话题和命令配置。
- [`map_catalog.json`](../src/route3d_web_console/config/map_catalog.json)：PCD、定位模板和拓扑绑定。
- [`protocol.md`](../src/route3d_web_console/docs/protocol.md)：WebSocket JSON 和二进制点云协议。

## 24. 验收整套链路时建议记录的证据

一次完整验收至少保存：

1. 自动打点的 `processing_summary.json` 和 `events.jsonl`。
2. 最终 `topoGraph_data.json`，并记录其 SHA256。
3. 规划请求和 `/route3d_dijkstra/status`。
4. `/route3d_route_slicer/tasks_json`，确认平地/坡段切片是否符合预期。
5. PID、Go2 adapter 的状态 JSON。
6. `/route3d_controller/active_source` 与 `/route3d_go2_adapter/selected_command`。
7. 平地障碍实验的 elevation debug 截图。
8. 坡前、坡中、坡后实际步态视频或状态记录。

这组证据能把问题明确定位到“图生成、图属性、最短路、切片、控制器、停障、速度仲裁、SDK”中的
某一层，避免只看到“机器人没动”就反复修改多个模块。
