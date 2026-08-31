# nav2_route3d

`nav2_route3d` 是一个面向 ROS 2 Jazzy + Nav2 的 3D route server 包。它的目标是替换 Nav2 默认的 2D `nav2_route` route server，同时保持 Nav2 action、service 和 topic 接口兼容，并在内部 graph、planner、scorer、tracker、path converter、route operation 等模块中完整保留 `x/y/z` 三维信息。

这个包主要解决室内外混合、坡道、楼梯/台阶状区域、多层空间等 2D route graph 难以表达的问题。

## 功能概览

- 3D route graph：节点 pose 包含 `x/y/z + quaternion`。
- 3D A* 规划：启发式距离和边代价使用三维距离。
- 3D 路径输出：`nav_msgs/Path` 中保留 `pose.position.z`。
- Nav2 兼容接口：复用 `nav2_msgs/action/ComputeRoute`、`ComputeAndTrackRoute` 和 `SetRouteGraph`。
- Route operations：支持楼梯/台阶、室内外切换、陡坡等场景事件。
- 离线 graph builder：从 odom pose 序列和 PCD 点云构建 3D route graph。
- shortcut 剪枝：基于累计路程的 anchor 节点生成 shortcut，避免折返处产生密集冗余边。
- JSON / GeoJSON graph 文件读写，方便调试和可视化。

## 包结构

```text
nav2_route3d/
  include/nav2_route3d/
    types.hpp
    graph_3d.hpp
    file_loader_3d.hpp
    graph_builder_3d.hpp
    edge_scorer_3d.hpp
    route_planner_3d.hpp
    route_tracker_3d.hpp
    goal_intent_extractor_3d.hpp
    path_converter_3d.hpp
    route_operations_3d.hpp
    route_server_3d.hpp
  src/
    route_server_3d.cpp
    route_graph_builder_3d_main.cpp
    graph_builder_3d.cpp
    graph_3d.cpp
    file_loader_3d.cpp
    pcd_io.cpp
    kdtree_3d.cpp
    gridmap_2d.cpp
    edge_scorer_3d.cpp
    route_planner_3d.cpp
    route_tracker_3d.cpp
    goal_intent_extractor_3d.cpp
    path_converter_3d.cpp
    route_operations_3d.cpp
  config/
    route_server_3d.yaml
  launch/
    route_server_3d.launch.py
  scripts/
    graph_editor.py
```

## Nav2 兼容接口

默认节点名建议保持为：

```text
/route_server
```

Action：

```text
compute_route
compute_and_track_route
```

Service：

```text
route_server/set_route_graph
```

Topic：

```text
plan
route_events
```

接口类型：

- `nav2_msgs/action/ComputeRoute`
- `nav2_msgs/action/ComputeAndTrackRoute`
- `nav2_msgs/srv/SetRouteGraph`
- `nav2_msgs/msg/Route`
- `nav_msgs/msg/Path`
- `std_msgs/msg/String`

因此 Nav2 BT 中的 `ComputeRoute` / `ComputeAndTrackRoute` action plugin 可以继续指向 `/route_server`。

## 构建

在 workspace 根目录执行：

```bash
colcon build --packages-select nav2_route3d
source install/setup.bash
```

运行测试：

```bash
colcon test --packages-select nav2_route3d --ctest-args --output-on-failure
```

## Graph Builder 输入

离线建图工具读取一个地图目录：

```text
map_path/
  poses.json
  pcd/
    0.pcd
    1.pcd
    2.pcd
```

`poses.json` 支持数组格式：

```json
[
  [timestamp, tx, ty, tz, qx, qy, qz, qw]
]
```

也支持对象格式：

```json
{
  "poses": [
    [timestamp, tx, ty, tz, qx, qy, qz, qw]
  ]
}
```

PCD 文件支持常见 ASCII 和 binary PCD，读取 `x/y/z` 字段。PCD 文件名按 frame index 查找，例如 `0.pcd`、`1.pcd`。

## 生成 3D Route Graph

示例：

```bash
ros2 run nav2_route3d route_graph_builder_3d \
  --map-path /home/langyi/workspace/map/79_long_louti_lio/map/slam_data \
  --poses-file pose.json \
  --pcd-dir key_frames \
  --radius 9.0 \
  --time-threshold 10.0 \
  --z-threshold 0.5 \
  --sensor-height 0.5 \
  --current-up-threshold 0.8 \
  --neighbor-pcd-window 1 \
  --grid-resolution 0.15 \
  --anchor-segment-length 1.0 \
  --min-shortcut-distance 0.8 \
  --shortcut-angular-bin-deg 20.0 \
  --max-candidates-per-angular-bin 2 \
  --max-raytrace-candidates 24 \
  --max-shortcut-edges 5 \
  --workers 16
```

输出文件：

```text
map_path/route3d_graph.json
map_path/route3d_graph.geojson
```

### Graph Builder 流程

1. 读取 odom poses。
2. 每个 pose 生成一个 graph node。
3. 顺序连接 `i -> i+1`，形成完整 teach-repeat 链路。
4. 按 odom 顺序累计 3D 路程，每 `anchor_segment_length_m` 米切分一个 segment。
5. 选取每个 segment 累计路程中点附近的 pose 作为 shortcut anchor。
6. 只用 anchor pose 构建 KD-tree。
7. 只遍历 anchor pose，并在 anchor KD-tree 中做半径搜索。
8. 过滤时间过近、Z 差过大、过近 shortcut 和角度桶内冗余候选。
9. 仅加载候选 anchor 附近 PCD，构建 2D occupancy grid。
10. 使用机器人通行高度带内的点云作为障碍，对 anchor 连线做 raytrace 可见性检查。
11. 可见且未超过 shortcut 数量上限时添加 shortcut 边。

顺序边不受 anchor 影响，仍然覆盖所有 odom pose。anchor 只影响额外 shortcut 边的生成。

### 常用 Graph Builder 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--radius` | `5.0` | shortcut 候选搜索半径，单位 m |
| `--time-threshold` | `10.0` | 时间差小于该值的候选会被过滤，避免局部连续帧互连 |
| `--z-threshold` | `0.2` | Z 差超过该值的候选会被过滤 |
| `--sensor-height` | `0.7` | 从 anchor 高度向下定义通行高度带下界（map 坐标 z） |
| `--current-up-threshold` | `1.2` | 从 anchor 高度向上定义通行高度带上界；0.4 m 左右小机器人可试 `0.05` + `0.45` |
| `--neighbor-pcd-window` | `3` | 加载每个候选 anchor 前后多少帧 PCD |
| `--grid-resolution` | `0.2` | occupancy grid 分辨率 |
| `--anchor-segment-length` | `5.0` | 每段累计路程长度；每段取一个 anchor 生成 shortcut |
| `--min-shortcut-distance` | `1.0` | 小于该距离的 shortcut 候选会被过滤 |
| `--shortcut-angular-bin-deg` | `20.0` | 按 XY 方位角分桶 |
| `--max-candidates-per-angular-bin` | `2` | 每个角度桶保留的候选数量 |
| `--max-raytrace-candidates` | `24` | 每个 anchor 最多进入 raytrace 的候选数量 |
| `--max-shortcut-edges` | `8` | 每个 anchor 最多添加的 shortcut 边数量 |
| `--workers` | CPU 核心数 | raytrace 并行 worker 数 |

如果 `--anchor-segment-length <= 0`，所有 pose 都会作为 anchor，行为接近旧版全量 shortcut 搜索。

## 启动 Route Server

```bash
ros2 launch nav2_route3d route_server_3d.launch.py \
  graph_filepath:=/home/langyi/workspace/map/79_louti_lio/map/slam_data/route3d_graph.json
```

Lifecycle：

```bash
ros2 lifecycle set /route_server configure
ros2 lifecycle set /route_server activate
```

默认参数文件：

```text
config/route_server_3d.yaml
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `route_frame` | `map` | route graph 所在坐标系 |
| `base_frame` | `base_link` | 机器人 base frame |
| `graph_filepath` | 空 | graph JSON 路径 |
| `path_density` | `0.5` | route 转 Path 时的插值密度，单位 m |
| `max_planning_time` | `2.0` | 最大规划时间，单位 s |
| `radius_to_achieve_node` | `1.0` | tracker 判断到达节点的半径 |
| `tracker_update_rate` | `20.0` | tracker 更新频率 |

## Graph 文件格式

JSON graph 顶层包含：

```json
{
  "format": "nav2_route3d",
  "version": 1,
  "frame_id": "map",
  "metadata": {},
  "nodes": [],
  "edges": []
}
```

节点包含：

```json
{
  "id": 1,
  "pose": [timestamp, tx, ty, tz, qx, qy, qz, qw],
  "metadata": {},
  "real_neighbors": [],
  "fake_neighbors": []
}
```

边包含：

```json
{
  "id": 1,
  "start_id": 1,
  "end_id": 2,
  "cost": 1.0,
  "bidirectional": true,
  "metadata": {},
  "operations": []
}
```

`metadata` 使用 `nlohmann::json`，可以自由扩展。

## Route Operations

默认支持三类 route operation：

- `stair_like_transition`
- `indoor_outdoor_transition`
- `steep_slope`

当 route 中的边 metadata 命中对应 `edge_type` 时，route server 会在 `route_events` topic 发布 JSON 字符串。

边 metadata 示例：

```json
{
  "edge_type": "stair_like_transition",
  "enter_profile": "stair_like_transition",
  "exit_profile": "factory_indoor",
  "speed_limit_mps": 0.25,
  "risk": 1.5
}
```

`route_events` 示例：

```json
[
  {
    "edge_id": 12,
    "operation": "stair_like_transition",
    "scenario_profile": "stair_like_transition",
    "speed_limit_mps": 0.25,
    "message": "Entering stair-like transition"
  }
]
```

下游可以由 behavior tree、scenario policy engine 或控制器订阅该 topic，用于切换导航 profile、限速或触发诊断。

## 调参建议

折返或平行轨迹处 shortcut 过密时，优先调整：

- 增大 `--anchor-segment-length`，例如 `5.0` 到 `8.0`。
- 减小 `--max-shortcut-edges`，例如 `8` 到 `2~4`。
- 减小 `--max-candidates-per-angular-bin`，例如 `2` 到 `1`。
- 增大 `--min-shortcut-distance`，使其略大于 odom 采样间距。
- 适当减小 `--radius`，避免跨越过远轨迹段。

如果漏连明显：

- 减小 `--anchor-segment-length`。
- 增大 `--radius`。
- 放宽 `--z-threshold`。
- 提高 `--max-raytrace-candidates` 或 `--max-shortcut-edges`。

如果 shortcut 穿墙仍连上：

- 先确认错误边的 `metadata.visibility_reason` 是否为 `raytrace_clear`；若是，说明 raytrace 认为可见，需要重建 graph 并检查 PCD 是否覆盖墙所在 teach 路径。
- 减小 `--grid-resolution`，例如 `0.15` 到 `0.2`。
- 收窄 `--sensor-height` / `--current-up-threshold`，使墙体点更容易进入障碍高度带。
- 增大 `--neighbor-pcd-window`，例如 `10` 到 `15`。
- 减小 `--max-shortcut-edges` 和 `--radius`，减少漏检后的错误 shortcut 数量。
- 少量仍错误的边，可在 `graph_editor.py` 中手动删除。

## Graph 编辑与可视化

`scripts/graph_editor.py` 提供图编辑能力，可用于检查 JSON / GeoJSON graph、修正节点和边 metadata、标注特殊 route operation 场景。

运行前请确认 Python 依赖已安装：

```bash
sudo apt install python3-numpy python3-pyqt6
```

## 当前定位

`nav2_route3d` 当前适合作为 Nav2 route server 的 3D 替代实现，用于离线构建 route graph，并在运行时通过 Nav2 标准 action 进行 route planning。它不是局部避障器，也不替代 controller 或 costmap；它输出的是带 3D 信息和场景 metadata 的全局 route / path。
