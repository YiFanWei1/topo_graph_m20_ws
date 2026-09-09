# `ceshi_1/topoGraph_data.json` 点边属性说明

本文对应文件：[`data/ceshi_1/topoGraph_data.json`](../data/ceshi_1/topoGraph_data.json)。

说明以当前工作空间代码（2026-09-09）为准，包含三类信息：

1. 这张图当前实际保存了什么；
2. 每个点、边属性的含义和修改方法；
3. 产品流程自动记点时写入的默认属性。

> 注意：直接修改 `topoGraph_data.json` 只影响这一个已有地图。若希望以后自动记录的图也使用新参数，应修改
> [`src/route3d_product_demo/config/live_route_product.yaml`](../src/route3d_product_demo/config/live_route_product.yaml)。重新录图可能覆盖手工修改，因此编辑前应先备份。

## 1. 当前图概览

| 项目 | 当前值 |
| --- | --- |
| Schema | `route3d_topology` / `2` |
| 坐标系 | `camera_init` |
| 图状态 | `complete` |
| 图结构 | `incremental_forest` |
| 点数 / 边数 | 11 / 10 |
| 连通分量 | 1 |
| 拓扑关系 | `1—2—3—4—5—6—7—8—9—10—11` |
| 行驶方向 | 所有边均为双向 |
| 边权总和 | 约 `9.406 m` |
| 拐点 | 1、7、11 |
| 坡点 | 2、3、4 |

这张图没有分叉和闭环，是一条双向线性路线。所有边的 `weight` 都等于两端点的三维欧氏距离。

本文件中的坡点需要特别注意：

```json
"slopeAnnotation": {
  "method": "manual_gait_switch_test"
}
```

因此点 2、3、4 是为了步态切换测试而人工标记的坡点，**不是当前自动坡度检测器识别出的结果**。

## 2. 顶层属性

| JSON 字段 | 当前值 | 含义 | 如何调整 |
| --- | --- | --- | --- |
| `version` | `25` | 图内容修订号，构图时随点边变化递增；不是格式版本 | 通常不要手改，程序加载主要依据 `schema.version` |
| `type` | `0` | 拓扑类别预留字段 | 普通地图保持 `0`，没有明确业务协议时不要改 |
| `frame_id` | `camera_init` | 所有点坐标所属的 TF 坐标系 | 只有在全部坐标已转换到新坐标系后才能一起修改，不能只改字符串 |
| `status` | `complete` | `recording` 表示记录中，`complete` 表示已完成 | 由记录流程维护，不建议手改 |
| `topology_mode` | `incremental_forest` | 没有闭环时为森林；建立闭环后可为 `incremental_graph` | 构图器自动计算，不要用手改字符串伪造闭环 |
| `schema.name` | `route3d_topology` | 文件格式名称 | 不要改 |
| `schema.version` | `2` | 文件接口版本 | 不要改；与顶层 `version` 含义不同 |
| `sceneMode` | `normal` | 场景模式，预留普通、隧道等场景 | 当前执行链主要使用 `normal`；其他值需先确认下游支持 |
| `vertices` | 11 个对象 | 点表；对象 key 是稳定点 ID | 可以编辑点属性，但不能产生重复 ID |
| `edges` | 10 个对象 | 边表；对象 key 是稳定边 ID | 端点必须引用已存在的点，边 ID 不要求连续 |

### 2.1 `generation` 构图统计

| 字段 | 当前值 | 含义 | 是否应手改 |
| --- | ---: | --- | --- |
| `retrace_count` | 2 | 识别到沿相邻旧边原路退回的次数 | 否，仅为生成报告 |
| `route_reentry_count` | 0 | 重新进入非相邻旧路线的次数 | 否 |
| `loop_closure_count` | 0 | 成功闭环次数 | 否 |
| `loop_validation_available_count` | 0 | 可执行 PCD 闭环验证的次数 | 否 |
| `loop_validation_rejection_count` | 0 | 闭环验证拒绝次数 | 否 |
| `suppressed_distance_m` | `5.3297` | 因识别为旧路退回而未重复建边的里程 | 否 |
| `edge_split_count` | 0 | 为连接旧边而拆边的次数 | 否 |
| `component_count` | 1 | 连通分量数量 | 否 |

### 2.2 `slopeAnnotation` 坡段报告

| 字段 | 当前值 | 含义 |
| --- | --- | --- |
| `method` | `manual_gait_switch_test` | 本次坡点标记方法 |
| `counts.normal` | 8 | 非坡点数 |
| `counts.slope` | 3 | 坡点数 |
| `segments` | 1 段 | 坡段列表 |
| `segments[0].startVertex/endVertex` | `1 / 5` | 测试坡段的前后边界点 |
| `segments[0].vertexIds` | `[2,3,4]` | 实际设置 `isSlope=true` 的点 |
| `segments[0].direction` | `simulated` | 本段是模拟方向，不代表自动判断的上坡或下坡 |
| `segments[0].source` | `manual_gait_switch_test` | 标注来源 |

如果手工修改坡点，应以各点的 `meta.isSlope` 为执行语义的权威值，并同步维护这份报告，避免报告与点属性矛盾。最稳妥的方法是使用坡度标注工具重新生成，而不是只改报告。

## 3. 点属性

点的完整路径为 `vertices.<点ID>`。例如点 2 的关键结构为：

```json
"2": {
  "pos": [-18.4797, 4.0350, -0.3215],
  "rpy": [0.0300, 0.0145, 2.5126],
  "meta": {
    "type": 0,
    "typeId": 0,
    "isCorner": false,
    "isSlope": true,
    "isJunction": false,
    "state": "confirmed",
    "source": "incremental_topology",
    "sourceStamp": 1788867645.459771,
    "component": 0,
    "turnDeg": 0.0,
    "chargingMode": 0
  },
  "pcd": "",
  "acc": 0.5,
  "turnable": true,
  "alignFinalYaw": true,
  "mustPassThrough": false,
  "passRadiusM": 0.45
}
```

### 3.1 点的直接属性

| 字段 | 单位/类型 | 自动默认值 | 含义和调整方法 |
| --- | --- | --- | --- |
| `pos` | `[x,y,z]`，m | 来源位姿；Z 再减 `body_height=0.40 m` | 点的地面坐标。可修正少量定位误差，但移动点后应重算相邻边 `weight`；必须保持在 `frame_id` 坐标系中 |
| `rpy` | `[roll,pitch,yaw]`，rad | 来源姿态 | 点姿态，角度必须用弧度。路径中间航向主要由路线几何决定；终点需要对齐 yaw 时使用第三项 |
| `pcd` | string | `""` | 关联点云文件名。当前生成器不为单点绑定 PCD；没有配套加载流程时保持空字符串 |
| `acc` | m | `0.50` | 点作为最终目标或业务端点时传入切片任务的容差。值越小越精确，但越难到达。**当前整条路线最终验收由控制器全局参数覆盖：PID 默认 `0.10 m`，Efficient 默认 `0.20 m`，所以只改这里不一定改变最终停车精度** |
| `turnable` | bool | `true` | 点级转向许可，当前会传到任务消息，但 PID 尚未直接强制执行。一般保持 `true`；不要依赖改为 `false` 实现禁止转向 |
| `alignFinalYaw` | bool | `true` | 当此点是整条路线终点时，是否要求对齐其 `rpy[2]`。不关心最终朝向可设 `false`；需要固定朝向设 `true` 并正确填写 yaw |
| `mustPassThrough` | bool | 拐点 `true`，普通点 `false` | 中间点是否为强制途经门。设为 `true` 可防止前视控制器从弯内侧切过；它不会自动要求停车 |
| `passRadiusM` | m | 拐点 `0.20`，普通点 `0.45` | 强制途经门半径。减小会更贴近该点，但过小可能抖动或难以放行；增大更顺滑，但更容易切弯。只有 `mustPassThrough=true` 时才是关键约束 |

`acc` 和 `passRadiusM` 不应混用：前者描述任务端点，后者描述路径中间点。若只是希望机器人在拐弯时贴近点，不应修改 `acc`，而应使用 `mustPassThrough=true` 并调整 `passRadiusM`。

### 3.2 `meta` 点属性

| 字段 | 自动默认值 | 含义和调整方法 |
| --- | --- | --- |
| `type` | `0` | 业务点类型。自动记点全部为普通点 `0`。当前切片器识别的有序组合见下文；只有配置了对应业务执行器时才应修改 |
| `typeId` | `0` | 同类业务点编号/预留标识，当前主要透传，几乎没有直接执行行为 |
| `chargingMode` | `0` | 充电或停靠模式，`0` 表示普通；当前主要透传/预留 |
| `isCorner` | 初始 `false`，拐点检测后更新 | 几何拐点。修改它会影响缺省途经门推导，但不会单独强制停车。手工设为 `true` 时建议同时设 `mustPassThrough=true`、`passRadiusM=0.20` |
| `isSlope` | 初始 `false`，坡度分析后更新 | 坡点。加载后会进行一次非递归同步：原始坡点使相邻边成为坡边，原始楼梯边使两个端点成为坡点。手工标坡时通常要标一段连续点，并在坡前后留缓冲 |
| `isJunction` | `false` | 是否为拆边形成的交汇/分支点，由构图器维护；不要仅靠改布尔值创建分支，真正的分支必须有相应边 |
| `state` | `confirmed` | 点已确认。是记录状态元数据，完成图通常保持 `confirmed` |
| `source` | `incremental_topology` | 点的生成来源，主要用于追踪和诊断 |
| `sourceStamp` | 来源 pose 时间戳 | 生成该点的 ROS 时间，单位 s；通常不要手改 |
| `component` | 从 `0` 开始 | 所属连通分量。当前所有点均为 `0`；应由真实边连通关系决定，不应单独手改 |
| `turnDeg` | 普通点 `0.0` | 拐点检测窗口内累计 yaw 扫角，单位是**度**。它是检测报告，不是控制器的目标转角 |

当前业务点 `meta.type` 的有序组合为：

| 当前点 → 下一点 | 任务类型 |
| --- | --- |
| `2 → 1` | `normal_charging` |
| `6 → 6` | `map_change` |
| `type != 7` → `7` | `single_charging` |
| `3 → 3` | `door` |
| `3 → 7` | `charging_with_door` |
| `7 → 3` | `retreat_with_door` |
| `3 → 8` | `open_door` |
| `8 → 3` | `close_door` |

当前图全部为 `type=0`，不会触发这些业务任务。

### 3.3 当前 11 个点

坐标保留三位小数，便于检查；JSON 中应保留原始精度。

| 点 | `pos` (m) | `yaw` (rad) | 拐点 | 坡点 | `alignFinalYaw` | 强制途经 / 半径 |
| ---: | --- | ---: | --- | --- | --- | --- |
| 1 | `[-17.849, 3.667, -0.298]` | `-0.619` | 是 | 否 | 否 | 是 / `0.20 m` |
| 2 | `[-18.480, 4.035, -0.322]` | `2.513` | 否 | 是 | 是 | 否 / `0.45 m` |
| 3 | `[-19.135, 4.779, -0.316]` | `2.393` | 否 | 是 | 是 | 否 / `0.45 m` |
| 4 | `[-19.820, 5.505, -0.309]` | `2.343` | 否 | 是 | 否 | 否 / `0.45 m` |
| 5 | `[-20.349, 6.347, -0.306]` | `2.162` | 否 | 否 | 是 | 否 / `0.45 m` |
| 6 | `[-20.903, 7.176, -0.299]` | `2.195` | 否 | 否 | 是 | 否 / `0.45 m` |
| 7 | `[-21.500, 7.934, -0.287]` | `-1.954` | 是 | 否 | 是 | 是 / `0.20 m` |
| 8 | `[-22.246, 7.969, -0.299]` | `3.010` | 否 | 否 | 是 | 否 / `0.45 m` |
| 9 | `[-23.222, 8.156, -0.300]` | `2.999` | 否 | 否 | 是 | 否 / `0.45 m` |
| 10 | `[-24.196, 8.339, -0.332]` | `2.956` | 否 | 否 | 是 | 否 / `0.45 m` |
| 11 | `[-25.165, 8.572, -0.314]` | `-1.048` | 是 | 否 | 是 | 是 / `0.20 m` |

所有点的其他共同值是：`type=0`、`typeId=0`、`chargingMode=0`、`isJunction=false`、`state=confirmed`、`source=incremental_topology`、`component=0`、`pcd=""`、`acc=0.5`、`turnable=true`。

## 4. 边属性

边的完整路径为 `edges.<边ID>`。例如：

```json
"1": {
  "v": [1, 2],
  "weight": 0.7304436644831561,
  "meta": {
    "dir": 0,
    "source": "discovery",
    "locomotionMode": 0,
    "linearSpeedMps": 1.0,
    "angularSpeedRadps": 0.0,
    "heightOffsetM": 0.0,
    "obstacleMode": 0,
    "travelMode": "bidirectional",
    "headingAngleRad": 0.0,
    "obstacleBoxM": [0.0, 0.0, 0.0, 0.0],
    "gridMapName": "",
    "controllerMode": "auto"
  },
  "rotationAllowed": true
}
```

| 字段 | 单位/类型 | 自动默认值 | 含义和调整方法 |
| --- | --- | --- | --- |
| `v` | `[first,second]` | 构图结果 | 两端点 ID。修改连接关系前必须确认两个点都存在，禁止自环；顺序还决定单向边的方向 |
| `weight` | number，通常为 m | 端点三维欧氏距离 | Dijkstra 路径代价，必须非负。值越小越容易被选中；若只表达几何长度，应在移动端点后按 `sqrt(dx²+dy²+dz²)` 重算 |
| `rotationAllowed` | bool | `true` | 是否允许在该边转向，并参与任务切片；当前 PID 会透传但尚未直接强制执行，不能仅靠设为 `false` 保证机器人绝不旋转 |
| `meta.dir` | integer | `0` | 兼容方向字段：`0` 双向、`1` 从 `v[0]` 到 `v[1]`、`2` 从 `v[1]` 到 `v[0]` |
| `meta.travelMode` | string | 由 `dir` 得到 `bidirectional` | 可读方向字段：`bidirectional`、`first_to_second`、`second_to_first`。修改方向时必须与 `dir` 同步，否则加载器会拒绝矛盾配置 |
| `meta.source` | string | `discovery` | 边来源；还可能是 `edge_split`、`loop_closure`。属于构图追踪信息，通常不手改 |
| `meta.locomotionMode` | integer | `0` | 运动/步态模式。`0` 为普通默认；坡边在值仍为 0 时会解析为有效模式 `2`，对应 `switch_gait_3`。显式设置 `2` 也会在加载时把该边识别为坡边并补齐两个端点的坡点语义；其他非零值保持显式配置 |
| `meta.linearSpeedMps` | m/s | `1.0` | 该边速度上限/期望值。减小可让危险边减速；当前 Go2 PID 平面速度还有全局 `0.80 m/s` 上限，所以设为 1.0 也只能跑到全局上限 |
| `meta.angularSpeedRadps` | rad/s | `0.0` | 角速度属性，变化会触发任务切片；当前 PID 不直接用它做角速度上限，`0` 表示未单独配置 |
| `meta.heightOffsetM` | m | `0.0` | 车身高度补偿属性，变化会触发切片；当前 PID 不直接应用该高度 |
| `meta.obstacleMode` | integer | `0` | 障碍策略，具体模式见下表。坡边原值为 0 时默认解析为有效模式 3 |
| `meta.headingAngleRad` | rad | `0.0` | 预留期望航向；当前加载器解析但没有传入执行控制，不建议用它代替点的 `rpy[2]` |
| `meta.obstacleBoxM` | `[xmin,xmax,ymin,ymax]`，m | `[0,0,0,0]` | 机器人局部坐标系障碍检测框。只有 `xmin<xmax` 且 `ymin<ymax` 才有效；全零表示使用控制器默认足迹 |
| `meta.gridMapName` | string | `""` | 绑定的栅格地图名。`obstacleMode=4` 时必须非空；还需保证对应局部规划器和地图实际存在 |
| `meta.controllerMode` | string | `auto` | 控制器选择。`auto`：平地用 PID、坡边用 Efficient 3D、栅格边用配置的 local planner；也可显式写 `pid` 或 `efficient_3d_local_planner`。未接入的名称会进入等待外部控制器接管状态 |

方向字段必须成对修改：

| 期望方向 | `dir` | `travelMode` |
| --- | ---: | --- |
| 双向 | 0 | `bidirectional` |
| 只允许 `v[0] → v[1]` | 1 | `first_to_second` |
| 只允许 `v[1] → v[0]` | 2 | `second_to_first` |

障碍模式：

| `obstacleMode` | 当前行为 |
| ---: | --- |
| 0 | 普通轨迹碰撞检测；发现障碍后停车等待，障碍清除后继续 |
| 1 | 近场停车并请求重规划，同时支持远场后台重规划；该边会单独切成任务片 |
| 3 | 忽略普通障碍检测，主要用于坡面/楼梯避免误判；外部急停和严重碰撞保护仍有效 |
| 4 | 栅格地图/外部局部规划器任务；必须设置 `gridMapName`，该边会单独切片 |

### 4.1 当前 10 条边

| 边 | 端点 | `weight` (m) |
| ---: | --- | ---: |
| 1 | 1—2 | 0.7304 |
| 2 | 2—3 | 0.9918 |
| 3 | 3—4 | 0.9976 |
| 4 | 4—5 | 0.9951 |
| 5 | 5—6 | 0.9974 |
| 6 | 6—7 | 0.9645 |
| 7 | 7—8 | 0.7471 |
| 8 | 8—9 | 0.9933 |
| 9 | 9—10 | 0.9922 |
| 10 | 10—11 | 0.9967 |

所有边的共同保存值是：`dir=0`、`travelMode=bidirectional`、`source=discovery`、`locomotionMode=0`、`linearSpeedMps=1.0`、`angularSpeedRadps=0.0`、`heightOffsetM=0.0`、`obstacleMode=0`、`headingAngleRad=0.0`、`obstacleBoxM=[0,0,0,0]`、`gridMapName=""`、`controllerMode=auto`、`rotationAllowed=true`。

虽然边 1～4 保存的 `locomotionMode` 和 `obstacleMode` 仍是 0，但加载器会根据原始坡点做一次非递归同步，切片器运行时会得到：

- 边 1～4：坡边，`controllerMode=efficient_3d_local_planner`、有效 `locomotionMode=2`、有效 `obstacleMode=3`；
- 边 5～10：平地边，`controllerMode=pid`、有效 `locomotionMode=0`、有效 `obstacleMode=0`。

这里包括边 1（1—2）和边 4（4—5），是为了在进入坡点前切换、离开最后一个坡点后再恢复，而不是只处理 2—3 和 3—4。同步产生的边界点不会继续把边 5（5—6）扩展成坡边。

## 5. 自动记点默认属性

以下默认值来自 [`schema.py`](../src/route3d_topology_core/route3d_topology_core/schema.py)、[`topology.py`](../src/route3d_topology_core/route3d_topology_core/topology.py) 和产品配置 [`live_route_product.yaml`](../src/route3d_product_demo/config/live_route_product.yaml)。

### 5.1 采点和拐点检测默认参数

| 配置 | 默认值 | 调整影响 |
| --- | ---: | --- |
| `skeleton.target_spacing` | `1.0 m` | 相邻普通拓扑点的目标间距；减小会产生更多点，增大则图更稀疏 |
| `skeleton.dedup_distance` | `0.05 m` | 小于此距离的末端重复点不提交 |
| `skeleton.body_height` | `0.40 m` | 保存点的 Z = 里程计机身 Z − 此值；应按机器人坐标原点到地面的实际高度标定 |
| `corner_xy_radius` | `0.50 m` | 原地/小范围转向检测窗口；过大可能把移动转弯也聚合，过小可能漏检 |
| `corner_yaw_degrees` | `45°` | 判定拐点的最小 yaw 扫角；减小更敏感，增大更保守 |
| `corner_min_duration` | `0.50 s` | 转向需持续的最短时间；减小更容易把瞬时抖动认成拐点 |
| `corner_max_z_range` | `0.10 m` | 拐点窗口允许的最大 Z 变化；避免把坡上姿态变化误判为原地转弯 |
| `corner_merge_distance` | `0.35 m` | 邻近拐点候选的合并距离 |
| `corner_min_separation` | `1.0 m` | 两个独立拐点的最小间隔 |

### 5.2 新点默认值

```text
meta.type=0
meta.typeId=0
meta.chargingMode=0
meta.isCorner=false（检测到拐点后更新）
meta.isSlope=false（坡度分析后更新）
meta.isJunction=false（拆边交汇点除外）
meta.state=confirmed
meta.source=incremental_topology
pcd=""
acc=0.50
turnable=true
alignFinalYaw=true
```

Schema 补全时还会根据最终 `isCorner` 写入：

| 点类型 | `mustPassThrough` | `passRadiusM` |
| --- | --- | ---: |
| 拐点 | `true` | `0.20 m` |
| 普通点 | `false` | `0.45 m` |

`pos`、`rpy`、`sourceStamp`、`component` 和 `turnDeg` 来自实际记录/构图结果，不是固定常数。

### 5.3 新边默认值

```text
weight=两端点三维欧氏距离
meta.dir=0
meta.travelMode=bidirectional
meta.source=discovery（拆边或闭环时会不同）
meta.locomotionMode=0
meta.linearSpeedMps=1.0
meta.angularSpeedRadps=0.0
meta.heightOffsetM=0.0
meta.obstacleMode=0
meta.headingAngleRad=0.0
meta.obstacleBoxM=[0.0,0.0,0.0,0.0]
meta.gridMapName=""
meta.controllerMode=auto
rotationAllowed=true
```

### 5.4 自动坡度检测默认参数

产品录图默认开启坡度分析：

| 配置 | 默认值 | 含义和调整方向 |
| --- | ---: | --- |
| `fit_radius` | `2.0 m` | 点附近用于局部线性拟合的路线半径；增大更平滑，减小更敏感 |
| `grade_threshold` | `0.12` | 坡度阈值 `|Δz/Δxy|`，约等于 12%；减小会识别更缓的坡 |
| `minimum_core_length` | `2.0 m` | 坡段核心最短长度；减小可能增加短噪声坡段 |
| `minimum_height_change` | `0.20 m` | 坡段核心最小高差；减小更敏感 |
| `maximum_core_gap` | `1.0 m` | 可合并为同一坡段的核心间最大间隙 |
| `buffer_distance` | `1.0 m` | 坡段前后额外标记的缓冲距离；增大可更早切换步态，但会扩大坡控制器覆盖范围 |

自动分析生成的 `slopeAnnotation.method` 通常是 `maximal_graph_chains/local_linear_z_over_xy_distance`，与本图当前的人工测试方法不同。

## 6. 常用调整示例

### 6.1 防止某个普通点被切过去

```json
"mustPassThrough": true,
"passRadiusM": 0.20
```

先尝试 `0.20～0.30 m`。若机器人很难放行再适当增大；该设置不会要求机器人停车。

### 6.2 让终点不调整朝向

```json
"alignFinalYaw": false
```

若设为 `true`，同时确认 `rpy[2]` 是正确的弧度值。整条路线的最终位置精度应在控制器配置中的 `completion.pid.position_tolerance_m` 或 `completion.efficient.position_tolerance_m` 调整，而不应只改点的 `acc`。

### 6.3 让危险路段降速

```json
"linearSpeedMps": 0.35
```

应修改该路段涉及的所有连续边；速度属性变化处会自动产生新的任务片，但仅速度变化本身不会强制停车。该值只能降低速度，不能突破全局速度上限。

### 6.4 把边改为单向

假设 `v` 为 `[4,5]`，只允许 4 到 5：

```json
"dir": 1,
"travelMode": "first_to_second"
```

反向则使用 `dir=2` 和 `second_to_first`。不要只改其中一个字段。

### 6.5 手工定义坡段

把坡段核心和必要的前后缓冲点设为：

```json
"isSlope": true
```

一次同步规则是“原始 JSON 中边的任一端点为坡点，则整条边为坡边”，因此连续标记点会自然覆盖进入和离开边。同步补出的边界点不会继续向外传播。实机测试前应检查步态切换位置、停稳延时以及坡面障碍模式是否符合现场安全要求。

### 6.6 显式指定控制器

```json
"controllerMode": "pid"
```

或：

```json
"controllerMode": "efficient_3d_local_planner"
```

通常优先保留 `auto`，让坡点和障碍模式决定控制器。只有确实需要覆盖自动策略并已验证控制器可用时才显式填写。

## 7. 修改后的检查清单

1. 先备份原 JSON。
2. `schema.name` 和 `schema.version` 保持 `route3d_topology / 2`。
3. 每条边的两个端点都必须存在，不能是同一个点。
4. `weight` 必须非负；移动点后重算相邻边距离。
5. `dir` 与 `travelMode` 必须一致。
6. 角度注意单位：`rpy`、`headingAngleRad`、`angularSpeedRadps` 用弧度；`turnDeg` 用度。
7. `obstacleMode=4` 必须配置非空 `gridMapName` 和可用的外部局部规划器。
8. 修改 `isSlope` 后同步检查 `slopeAnnotation`，并检查相邻边的实际坡边范围。
9. 使用下面的命令检查 JSON 语法：

```bash
python3 -m json.tool data/ceshi_1/topoGraph_data.json >/dev/null
```

10. 上车前先在 RViz/bag 环境检查规划点序列、切片结果、有效控制器、步态和障碍模式，再进行低速实机测试。

## 8. 代码依据

- Schema 和点边缺省值：[`schema.py`](../src/route3d_topology_core/route3d_topology_core/schema.py)
- 自动构图、坐标和边权生成：[`topology.py`](../src/route3d_topology_core/route3d_topology_core/topology.py)
- 产品录图参数：[`live_route_product.yaml`](../src/route3d_product_demo/config/live_route_product.yaml)
- Schema V2 总体接口：[`topology_schema_v2_interface.md`](topology_schema_v2_interface.md)
- 路径属性切片和坡点规则：[`route_slicer_interface.md`](route_slicer_interface.md)
- PID 到点和障碍行为：[`pid_controller_interface.md`](pid_controller_interface.md)
