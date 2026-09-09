# Route3D 语义路径切片与控制衔接接口

## 1. 结论与设计目标

`route3d_route_slicer` 是 Dijkstra 与后续控制执行器之间的新纯 C++17 功能包：

```text
Schema V2 topoGraph_data.json
            |
            v
route3d_dijkstra_planner             起点/终点 ID -> 最短点边序列
            |
            | /route3d_dijkstra/status
            v
route3d_route_slicer                 属性切片、业务切片、途经约束、步态意图
            |
            | /route3d_route_slicer/tasks
            v
后续 route executor                 停车 -> 切步态/切控制器 -> 下发路径 -> 验收
            |
            +-- PID 跟踪器
            +-- efficient_3d_local_planner
```

本包只生成可执行任务，不直接调用机器人步态服务，也不发布 `cmd_vel`。这样离线、bag 和
实机可以使用同一套切片结果；真正影响机器人状态的动作集中放在后续 executor 中。

核心取舍是：

- 控制器、步态、速度、障碍策略或业务动作改变时，生成新的**语义任务片**。
- 普通拐点默认不生成新的语义任务片，而是成为片内的**强制途经点**。
- 路径请求的最后一个点才是任务目标。它是不是拐点不影响目标身份。
- `acc` 管最终目标验收，`passRadiusM` 管中间途经点，两套容差不再混用。

因此既能防止控制器切弯，也不会让每个转角都产生一次不必要的停车和控制器重启。

## 2. 为什么参考工程必须切片

参考工程 `graph_pid_ws_refactor` 的 `GraphManager::planAndSlice()` 先得到最短点边序列，
再由 `re_sliceGraphWithTypeAndSequence()` 按边属性和节点业务类型拆成 `Tasks`。下游以一个
`Task` 为单次执行单位：设置当前片参数，执行片前业务，调用控制器，最后执行片后业务。

切片不是按 1 m 或固定点数切割。主要原因是旧控制接口的一次任务只能稳定持有一组：

- 步态、高度、线速度和角速度；
- 障碍检测模式、障碍框和栅格地图；
- 行驶方向与是否可转向；
- PID 或基于栅格的局部规划控制方式；
- 充电、换图、门等片前/片后业务动作。

如果一条长路径中途改变这些属性而不切片，下游只读取片首边参数时，后半段会继续使用
错误参数。这也是本项目保留语义切片的根本理由。

参考代码中所谓“途经点”和“目标点”并不是由 `isCorner` 判断：一个切片的首尾节点构成
本次控制任务，中间节点是控制器路径点；最后一个切片的末点才是整条规划的最终目标。
旧实现的到点精度读取和任务端点耦合较强，无法优雅表达“拐点要贴近通过，但任意普通点
也可能是最终目标”。新接口专门把这两种语义拆开。

## 3. 已保留的参考切片规则

### 3.1 边属性发生变化

相邻边下列任一有效属性变化都会切片，浮点字段使用
`slicing.attribute_epsilon` 比较：

| Schema V2 字段 | 切片原因 |
| --- | --- |
| `meta.controllerMode` 或解析后的控制器 | `controller_mode_changed` |
| `meta.locomotionMode` 或步态命令 | `locomotion_mode_changed` |
| `meta.linearSpeedMps` | `linear_speed_changed` |
| `meta.angularSpeedRadps` | `angular_speed_changed` |
| `meta.heightOffsetM` | `height_offset_changed` |
| `meta.obstacleMode` | `obstacle_mode_changed` |
| `meta.obstacleBoxM` | `obstacle_box_changed` |
| `meta.gridMapName` | `grid_map_changed` |
| `rotationAllowed` | `rotation_permission_changed` |

连续两步使用同一 edge ID 会用 `immediate_edge_retrace` 单独切开，避免把立即折返吞进同一
控制片。每两个相邻任务共享边界顶点，所以下一片不会丢失起点。

### 3.2 障碍特殊边

- `obstacleMode=1`：单边独立成片。
- `obstacleMode=4`：单边独立成片，且必须提供非空 `gridMapName`。
- `controllerMode=auto` 时，坡段优先解析为 `efficient_3d_local_planner`；非坡段若是模式 4
  或绑定了栅格地图则解析为 `local_planner`；其余平地解析为 `pid`。

自动解析名称可通过 YAML 修改，不在核心代码中绑定具体控制器进程名。

### 3.3 节点业务类型组合

以下有序点类型组合沿用参考工程的独立任务处理：

| 当前点类型 -> 下一点类型 | `task_mode` |
| --- | --- |
| `2 -> 1` | `normal_charging` |
| `6 -> 6` | `map_change` |
| 非 `7 -> 7` | `single_charging` |
| `3 -> 3` | `door` |
| `3 -> 7` | `charging_with_door` |
| `7 -> 3` | `retreat_with_door` |
| `3 -> 8` | `open_door` |
| `8 -> 3` | `close_door` |

这些片使用 `COMPLETION_BUSINESS_STOP`，默认要求停车。当前自动生成图中的
`meta.type=0`，所以不会误触发；将来地图编辑器写入业务类型后无需重写切片核心。

参考工程的 `checkNeighborNodes()` 当前只输出检查日志，没有改变切片结果，因此本包没有
复制一个无实际语义的二次切分。若后续明确其业务约束，可在核心层增加独立规则和测试。

## 4. 坡点与步态

### 4.1 当前规则

拓扑图读取完成后会基于原始 JSON 快照做一次点边坡度同步：

- 原始 `meta.isSlope=true` 点的相邻边被标为坡边；
- 原始 `locomotionMode=2` 坡边的两个端点被标为坡点；
- 本轮新标出的点边不再次参与传播。

因此一条边任一原始端点 `meta.isSlope=true` 时会被视为坡段，显式设置
`locomotionMode=2` 的边也会成为坡段。同步只执行一轮，避免一个坡点沿连通图递归扩散到
所有点边。切片器使用这次冻结的边级结果，不会从同步产生的边界点继续向外推导。

当边本身仍是默认 `locomotionMode=0` 时，切片器把坡段的有效模式提升为 `2`：

| 场景 | 有效模式 | `gait_command` |
| --- | ---: | --- |
| 普通路面 | `0` | `static_walk` |
| 坡段 | `2` | `switch_gait_3` |

若地图作者在边上显式填写 `locomotionMode=2`，即使 JSON 中两个端点原本是普通点，该边也
会使用楼梯步态，并在内存中把两个端点补为坡点。其他非零 `locomotionMode` 仍保持显式
配置，不会被坡点的默认值覆盖。可通过 `slope.enable_gait_switch=false` 关闭从坡边到步态 2
的自动提升；显式填写的 `locomotionMode=2` 不受该开关影响。

默认还启用 `slope.ignore_ordinary_obstacles=true`：坡段上原本为 0 的障碍模式会解析为
`obstacleMode=3`，与参考工程楼梯任务一致；平地仍为 0。显式配置的模式 1/3/4 不会被
覆盖，碰撞等级 100 的紧急停车也不受该选项影响。

### 4.2 与已有步态测试包的对应

工作空间中的 `go2_gait_switch_demo` 已验证两种实机调用：

- 普通步态：`StaticWalk()`；
- 坡地步态：旧接口 `SwitchGait(3)`，API ID 为 `1011`。

该 demo 在切换前先停止并留出稳定时间。新切片消息用：

- `requires_gait_switch_at_start` 告诉 executor 本片开始前是否需要切换；
- `gait_command` 告诉 executor 目标步态；
- 上一片的 `requires_stop_at_end` 保证步态或控制器硬切换前先停车。

`route3d_go2_adapter` 已复用这些底层调用和超时处理，切片节点不直接操作机器人。执行
状态序列为：

```text
STOPPING -> SWITCH_GAIT -> SETTLING -> ACKNOWLEDGING -> TRACKING
```

## 5. 拐点、途经点与最终目标

### 5.1 为什么拐点默认不做语义切片

目前节点约 1 m 间隔，拐点只是同一控制策略下的几何约束。若每个拐点都切成完整任务，
容易出现反复停止、重新初始化 PID、轨迹不连续等问题。拐点本身不改变速度/控制器/步态
时，不需要语义切片。

更合适的方式是让控制执行器维护一个“途经门”：

1. 当前锁定第一个尚未通过的 `must_pass_through=true` 点。
2. 前视点最多推进到这个途经点，不能直接跳到转角之后。
3. 机器人进入该点的 `pass_radius_m` 后，才释放下一段路径。
4. 不要求停车，控制器可以连续转弯；如安全性需要，可叠加曲率限速。

这能避免 B 样条或长前视距离从弯道内侧切过去，同时不把拐点误当任务终点。

### 5.2 三个互相独立的概念

| 概念 | 判断来源 | 使用字段 |
| --- | --- | --- |
| 几何拐点 | 构图算法 | `meta.isCorner`、`meta.turnDeg` |
| 中间必须经过 | 点属性 | `mustPassThrough`、`passRadiusM` |
| 整条任务目标 | Dijkstra 请求的终点 ID | `acc`、`alignFinalYaw` |

默认情况下，拐点自动得到 `mustPassThrough=true` 和 `passRadiusM=0.20`；普通点为
`false/0.45`。也可以显式覆盖：

```json
"42": {
  "pos": [12.3, 4.5, 0.0],
  "rpy": [0.0, 0.0, 1.57],
  "acc": 0.08,
  "alignFinalYaw": false,
  "mustPassThrough": true,
  "passRadiusM": 0.12,
  "meta": {
    "isCorner": false,
    "isSlope": false
  }
}
```

这个点虽然是普通点，但：

- 若作为路径中间点，必须进入 0.12 m 半径后才能放行下一段；
- 若它是本次 Dijkstra 终点，则以 `acc=0.08 m` 验收；
- `alignFinalYaw=false` 表示终点不强制对齐 yaw。

因此“普通点也可能需要精确到达”不再需要借用 `isCorner`。当前 `regu_schema_v2` 文件
生成于新增两个途经字段之前，C++ 加载器会从 `isCorner` 推导默认值；之后由产品流程生成
的 Schema V2 JSON 会显式写出这两个字段。

### 5.3 何时仍应在拐点停车

只有下列业务要求才建议把某个拐点升级为任务边界：

- 必须原地旋转后才能继续；
- 窄门、充电、换图等动作要求静止；
- 步态或控制器在这里改变；
- 机器人机构安全规则明确要求零速换向。

前 3 类已经能由边属性或业务类型自动切片。若以后需要“任意普通点强制停车”，建议新增
独立的 `arrivalMode=stop`，不要重载 `isCorner`。本版没有凭空赋予该字段默认行为。

## 6. ROS 2 接口

### 6.1 订阅

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/route3d_dijkstra/status` | `std_msgs/msg/String` | Dijkstra 原子 JSON 结果，同时含有序 vertex/edge ID，避免两个数组话题的时序竞争 |

只处理 `success=true` 的状态。边数必须等于点数减一，且每条边必须连接对应相邻点并满足
`travelMode`，否则拒绝输出任务。起终点相同时允许 1 个点、0 条边，会输出一个
`same_vertex_goal` 终点验收任务，而不是把合法请求误报为切片失败。

### 6.2 发布

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/route3d_route_slicer/tasks` | `route3d_route_slicer/msg/RouteTaskArray` | 控制 executor 应使用的强类型权威接口 |
| `/route3d_route_slicer/tasks_json` | `std_msgs/msg/String` | 同内容 JSON，便于日志与临时调试 |
| `/route3d_route_slicer/status` | `std_msgs/msg/String` | 成功、失败、片数和切片耗时 |
| `/route3d_route_slicer/markers` | `visualization_msgs/msg/MarkerArray` | 各任务彩色路径、标签、橙色拐点、紫色坡点和红色目标点 |

输出使用 reliable + transient-local QoS，后启动的 executor/RViz 仍能收到最近一次结果。

### 6.3 `RouteTaskArray`

| 字段 | 含义 |
| --- | --- |
| `route_sequence` | 每次成功切片递增的序号，executor 用于丢弃旧任务 |
| `route_start_id` / `route_goal_id` | 整条路径的起终点 |
| `total_cost` | 上游 Dijkstra 累计权重 |
| `slicing_time_ms` | C++ 切片与消息准备前的耗时 |
| `source_graph` | 加载的 Schema V2 文件 |
| `tasks` | 按执行顺序排列的任务片 |

### 6.4 `RouteTask`

关键字段如下：

| 字段 | 含义 |
| --- | --- |
| `task_index` | 片序号，从 0 开始 |
| `task_mode` | `normal` 或第 3.3 节业务模式 |
| `configured_controller_mode` | JSON 原始值，例如 `auto` |
| `resolved_controller_mode` | 已解析为 `pid`、`efficient_3d_local_planner` 或显式外部控制器 |
| `gait_command` | `static_walk` 或 `switch_gait_3` |
| `completion_policy` | `0` 过渡片、`1` 路线终点、`2` 业务停车片 |
| `requires_stop_at_end` | 结束后是否必须停车 |
| `requires_gait_switch_at_start` | 开始前是否要执行步态切换 |
| `endpoint_tolerance_m` | 过渡片使用末点通过半径；终点/业务片使用末点 `acc` |
| `align_goal_yaw` | 仅终点/业务片可能为真，中间片不会误对齐 yaw |
| `waypoints` / `edge_ids` | 本片有序点边，始终满足点数=边数+1 |
| `split_reasons` | 可审计的切片原因 |

每个 `RouteWaypoint` 同时携带拐点、坡点、交汇点、RPY、最终到点容差、途经半径和 PCD
引用，下游无需再次打开 JSON 猜测属性。

## 7. 编译与测试

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-up-to route3d_route_slicer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon test --packages-select \
  route3d_dijkstra_planner route3d_route_slicer route3d_topology_core
colcon test-result --verbose
```

`route3d_route_slicer` 的可执行逻辑、核心算法和测试均为 C++；ROS 接口构建时由
`rosidl` 自动生成语言绑定，这不属于 Python 算法实现。

## 8. 复杂图演示

终端 A 启动 Dijkstra、切片器和 RViz：

```bash
cd /home/wei/github_code/topo_graph_ws_
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_route_slicer route_slicer_demo.launch.xml
```

终端 B 发送请求：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wei/github_code/topo_graph_ws_/install/setup.bash

ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [1, 20]}"

ros2 topic echo --once /route3d_route_slicer/tasks \
  --qos-durability transient_local
ros2 topic echo --once /route3d_route_slicer/status \
  --qos-durability transient_local
```

复杂测试图含 PID、局部规划边、坡点、单向边和回环；障碍模式 1/4 由 C++ 单元测试
覆盖。RViz 中每片颜色不同，文字显示
`T编号 控制器 / 步态`。

也可自动循环已有复杂请求：

```bash
ros2 launch route3d_route_slicer route_slicer_demo.launch.xml auto_demo:=true
```

若已经单独启动了 `route3d_dijkstra_planner`，应使用
`start_planner:=false`，避免两个同名规划节点同时响应一份请求：

```bash
ros2 launch route3d_route_slicer route_slicer_demo.launch.xml \
  start_planner:=false
```

## 9. 使用真实 `regu_schema_v2` 图

```bash
ros2 launch route3d_route_slicer route_slicer_demo.launch.xml \
  graph_file:=/home/wei/github_code/topo_graph_ws_/data/regu_schema_v2/topoGraph_data.json
```

另一个终端发送：

```bash
ros2 topic pub --once /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray "{data: [1, 109]}"
```

本次实测结果：Dijkstra 搜索得到 108 个点、107 条边；切片器得到 5 片，依次为普通、
坡地、普通、坡地、普通步态。一次切片耗时约 `0.174 ms`（具体值随主机负载变化）。

## 10. 当前 executor 行为

当前 `route3d_pid_controller` 节点同时承担路线 executor，严格串行执行同一
`route_sequence`：

1. 检查任务序号和共享端点连续性。
2. 若 `requires_gait_switch_at_start=true`，确认机器人已停车，再调用步态接口并等待成功。
3. 根据 `resolved_controller_mode` 发布 `pid` 或 `efficient_3d_local_planner` 控制权。
4. 下发本片全部 waypoint，同时把 `must_pass_through/pass_radius_m` 交给目标推进器。
5. 用 `endpoint_tolerance_m` 和 `align_goal_yaw` 判断本片结束，不能再用 `isCorner` 猜。
6. `requires_stop_at_end=true` 时等速度归零，再进入下一片或业务状态。
7. 收到更大的 `route_sequence` 时，安全取消旧序列，不能混合执行两个规划结果。

PID 和 effi 常驻运行，Go2 适配器只放行 executor 选中的一路。坡段路径发布到
`/route3d_controller/efficient_path`；切换、终点或安全停止时先发布 `none`。
