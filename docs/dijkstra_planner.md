# Route3D Schema V2 Dijkstra 最短路径规划

## 1. 功能与输入格式

`route3d_dijkstra_planner` 是纯 C++17 实现的独立 ROS 2 功能包，直接读取自动骨架流程生成的
`topoGraph_data.json`。加载器要求：

```json
"schema": {
  "name": "route3d_topology",
  "version": 2
}
```

它不会加载旧 `topoSingle` 或 `nav2_route3d` 格式。搜索时：

- 顶点来自 `vertices`，不要求 ID 连续。
- 邻接关系来自 `edges.*.v`，不按数字 ID 猜测连接关系。
- 搜索代价使用非负的 `edges.*.weight`。
- 行驶方向优先且严格使用 Schema V2 的 `edges.*.meta.travelMode`。
- 规划结果保留每条边的 `controllerMode`，为后续 PID/局部规划器切换提供输入。
- `isCorner`、`isSlope`、`isJunction` 等属性不会改变当前基础 Dijkstra 代价。

`travelMode` 的取值为：

| 值 | 可遍历方向 |
| --- | --- |
| `bidirectional` | `v[0] ↔ v[1]` |
| `first_to_second` | 仅 `v[0] → v[1]` |
| `second_to_first` | 仅 `v[1] → v[0]` |

## 2. ROS 接口

### 订阅

| 话题 | 消息类型 | 数据 |
| --- | --- | --- |
| `/route3d_dijkstra/plan_request` | `std_msgs/msg/Int32MultiArray` | 必须恰好为 `[起点ID, 终点ID]` |

### 发布

| 话题 | 消息类型 | 含义 |
| --- | --- | --- |
| `/route3d_dijkstra/path` | `nav_msgs/msg/Path` | 按最短路径节点生成的位姿序列 |
| `/route3d_dijkstra/path_vertex_ids` | `std_msgs/msg/Int32MultiArray` | 有序路径节点 ID |
| `/route3d_dijkstra/path_edge_ids` | `std_msgs/msg/Int32MultiArray` | 有序路径边 ID |
| `/route3d_dijkstra/markers` | `visualization_msgs/msg/MarkerArray` | 完整图、标签和当前最短路径 |
| `/route3d_dijkstra/status` | `std_msgs/msg/String` | JSON 状态、代价、耗时和控制器提示 |

除请求话题外，输出均使用可靠、`transient_local` QoS，新启动的 RViz 或调试订阅者也能
立即收到最后一次规划结果。

成功状态示例：

```json
{
  "success": true,
  "implementation": "cpp",
  "algorithm": "dijkstra",
  "schema": {"name": "route3d_topology", "version": 2},
  "start_id": 3,
  "goal_id": 13,
  "vertex_ids": [3, 13],
  "edge_ids": [38],
  "controller_modes": ["local_planner"],
  "total_cost": 3.2,
  "expanded_vertices": 5,
  "relaxed_edges": 9,
  "planning_time_ns": 12726,
  "planning_time_ms": 0.012726
}
```

`planning_time_ns/ms` 使用单调高精度时钟测量纯搜索过程，不包含 JSON 加载、ROS 消息
序列化和 RViz 渲染时间。

## 3. 编译

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select route3d_dijkstra_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 4. 复杂测试图演示

包内测试图为 `config/complex_topology_v2.json`，包含：

- 22 个 Schema V2 顶点、40 条边；
- 网格回环、对角捷径和不同 `weight`；
- `pid`、`local_planner`、`auto` 三种 `controllerMode`；
- 一条只能 `3 → 13` 行驶的单向边；
- 一个与主图不连通的 `21—22` 分量。

启动规划节点和 RViz：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_dijkstra_planner dijkstra_demo.launch.xml
```

另一个终端发送规划请求：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wei/github_code/topo_graph_ws_/install/setup.bash

ros2 topic pub --once \
  /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray \
  "{data: [1, 20]}"
```

应得到路径：

```text
1 -> 7 -> 13 -> 19 -> 20
cost=9.05
```

推荐继续测试：

```bash
# 另一条跨图路径：5 -> 9 -> 13 -> 17 -> 16
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [5, 16]}"

# 正向使用单向捷径：3 -> 13，cost=3.2
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [3, 13]}"

# 反向不能使用同一单向边：13 -> 8 -> 3，cost=4.0
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [13, 3]}"

# 不连通测试：返回 no path，不发布旧路径
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [1, 22]}"
```

也可以让 launch 每 4 秒自动循环上述成功路径：

```bash
ros2 launch route3d_dijkstra_planner dijkstra_demo.launch.xml \
  auto_demo:=true \
  demo_interval:=4.0
```

RViz 颜色：

| 显示 | 颜色 |
| --- | --- |
| 完整拓扑边 | 灰色细线 |
| 全部顶点 | 蓝色小球和白色 ID |
| 最短路径 | 绿色粗线 |
| 路径节点 | 黄色球 |
| 起点 | 青色大球 |
| 终点 | 红色大球 |

## 5. 使用 `regu_schema_v2` 实际拓扑

启动时覆盖 `graph_file`：

```bash
ros2 launch route3d_dijkstra_planner dijkstra_demo.launch.xml \
  graph_file:=/home/wei/github_code/topo_graph_ws_/data/regu_schema_v2/topoGraph_data.json
```

发送实际节点编号，例如：

```bash
ros2 topic pub --once \
  /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray \
  "{data: [1, 109]}"
```

查看规划状态、节点和边序列：

```bash
ros2 topic echo /route3d_dijkstra/status --qos-durability transient_local
ros2 topic echo /route3d_dijkstra/path_vertex_ids --qos-durability transient_local
ros2 topic echo /route3d_dijkstra/path_edge_ids --qos-durability transient_local
```

当前 `regu_schema_v2/topoGraph_data.json` 是单连通森林，没有闭环捷径，因此部分长距离
查询仍会沿主链经过大量节点。这是输入图的连接关系，不是 Dijkstra 搜索错误；使用包含
闭环的实机拓扑后，搜索会在多条候选路线中选择累计 `weight` 最小的一条。

## 6. 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `graph_file` | 包内复杂测试图 | Schema V2 智能拓扑文件 |
| `strict_schema_v2` | `true` | 要求全部新增控制属性存在且类型正确 |
| `topics.request` | `/route3d_dijkstra/plan_request` | 规划请求话题 |
| `topics.path` | `/route3d_dijkstra/path` | 路径话题 |
| `topics.path_vertex_ids` | `/route3d_dijkstra/path_vertex_ids` | 节点序列话题 |
| `topics.path_edge_ids` | `/route3d_dijkstra/path_edge_ids` | 边序列话题 |
| `topics.markers` | `/route3d_dijkstra/markers` | RViz Marker 话题 |
| `topics.status` | `/route3d_dijkstra/status` | JSON 状态话题 |
| `visualization.z_offset` | `0.12` | Marker 相对图节点的显示高度，单位 m |
| `visualization.show_vertex_labels` | `true` | 是否显示所有顶点 ID |
| `visualization.max_display_vertices` | `20000` | RViz 背景顶点上限；`0` 表示不限 |
| `visualization.max_display_edges` | `50000` | RViz 背景边上限；`0` 表示不限 |
| `visualization.max_display_labels` | `2000` | RViz 顶点文字上限；`0` 表示不限 |

以上限制只对完整图的背景 Marker 做均匀抽样，不会截断当前最短路径、节点 ID 结果或边
ID 结果。小于限制的正常实机图会完整显示。

## 7. 日志

每次成功搜索都会输出：

```text
Dijkstra success: start=1 goal=20 cost=9.050000 vertices=5 edges=4
expanded=20 relaxed=20 planning_time_ms=0.086717 path=1->7->13->19->20
```

无法到达、节点不存在或请求长度错误时会输出 `Dijkstra failed`，同时发布空路径并在
`/route3d_dijkstra/status` 中给出错误原因和失败搜索耗时，避免下游继续使用上一次路径。
