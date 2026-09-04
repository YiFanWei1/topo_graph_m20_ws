# TopoSingle、nav2_route3d 与 graph_pid_ws_refactor 图格式对比

> 本文以旧样例 `data/regu/topoSingle_data_slope.json` 说明历史格式。产品 launch 当前生成的
> Schema V2 已补齐自有命名的控制属性，最新字段定义与映射请以
> [`topology_schema_v2_interface.md`](topology_schema_v2_interface.md) 为准。

## 1. 对比范围

本文对比以下三种文件/加载器：

1. 当前数据文件：
   `data/regu/topoSingle_data_slope.json`
2. 当前工作区的自定义三维路线包：
   `src/nav2_route3d`
3. 参考工程：
   `/media/wei/Lenovo/beifen/ubuntu22/github_code/graph_pid_ws_refactor`

这里的 `nav2_route3d` 指当前工作区内的自定义三维扩展包，不是 Nav2 上游定义的一种
`TopoSingle` 标准。Nav2 上游并没有规定本项目使用的 `vertices + edges` 文件结构。

## 2. 结论摘要

`topoSingle_data_slope.json` 与另外两种格式的关系如下：

| 对比对象 | 结构是否相同 | 能否直接加载 | 主要结论 |
| --- | --- | --- | --- |
| 当前 `nav2_route3d` 原生 JSON | 否 | 否 | 必须转换字段结构和姿态表达 |
| 当前 `nav2_route3d` GeoJSON | 否 | 否 | 必须转换成 FeatureCollection |
| `graph_pid_ws_refactor` | 基本相同 | 语法上基本可以 | 业务属性不完整，部分现有字段会被忽略 |

最关键的判断是：

- 当前文件与参考工程使用的是同一类 `TopoSingle` 点边结构。
- 当前文件不是 `nav2_route3d` 原生图文件。
- 当前文件可以作为参考工程加载器的输入基础，但还不能认为已经具备参考工程完整的
  普通导航、管廊导航、PID/DWA 切换能力。
- 当前文件本身是一条单链。它有 117 个顶点、116 条边，所有边都是 `dir=0`，没有分支
  和回环。后续最短路径应使用智能图 `topoGraph_data.json`，而不是把单链文件作为最终
  权威图。

## 3. 当前文件的实际内容

当前文件顶层为：

```json
{
  "version": 0,
  "type": 0,
  "frame_id": "map",
  "vertices": {},
  "edges": {},
  "slopeAnnotation": {}
}
```

统计如下：

| 项目 | 当前值 |
| --- | ---: |
| 顶点数 | 117 |
| 边数 | 116 |
| 坡点数 | 57 |
| 拐点数 | 17 |
| 坡段数 | 6 |
| 边方向 | 116 条全部为 `dir=0` |
| 边权总和 | 约 96.619 |

顶点示例：

```json
"1": {
  "pos": [-3.1249, 0.5902, -0.3002],
  "rpy": [-0.0148, 0.0322, 2.9947],
  "meta": {
    "type": 0,
    "typeId": 0,
    "isCorner": false,
    "source": "route3d_targets",
    "sourceDistance": 0.0,
    "sourceNodeIndex": 0,
    "turnDeg": 0.0,
    "isSlope": false
  },
  "pcd": "",
  "acc": 0.5,
  "turnable": true
}
```

边示例：

```json
"1": {
  "v": [1, 2],
  "weight": 0.9966,
  "meta": {
    "dir": 0,
    "source": "route3d_targets"
  }
}
```

当前 `weight` 是相邻目标点的三维欧氏距离。`slopeAnnotation` 是本项目坡度分析器产生的
图级分析报告，不参与当前点边连接关系。

## 4. 与 nav2_route3d 原生 JSON 的区别

### 4.1 nav2_route3d 的原生结构

当前工作区 `nav2_route3d` 的原生 JSON 为：

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

节点格式：

```json
{
  "id": 1,
  "pose": [timestamp, x, y, z, qx, qy, qz, qw],
  "metadata": {},
  "real_neighbors": [],
  "fake_neighbors": []
}
```

边格式：

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

### 4.2 顶层结构对比

| 含义 | TopoSingle | nav2_route3d |
| --- | --- | --- |
| 格式标识 | 无 | `format: nav2_route3d` |
| 版本 | `version` | `version`，当前固定写 1 |
| 坐标系 | `frame_id` | `frame_id` |
| 节点容器 | `vertices` 对象 | `nodes` 数组 |
| 边容器 | `edges` 对象 | `edges` 数组 |
| 图级扩展信息 | `slopeAnnotation` 等独立字段 | `metadata` |

TopoSingle 使用字符串键保存 ID：

```json
"vertices": {
  "1": {...},
  "2": {...}
}
```

nav2_route3d 把 ID 放在数组元素内部：

```json
"nodes": [
  {"id": 1, ...},
  {"id": 2, ...}
]
```

### 4.3 节点字段对比

| 含义 | TopoSingle | nav2_route3d |
| --- | --- | --- |
| ID | `vertices` 的字符串键 | `node.id` |
| 位置 | `pos: [x,y,z]` | `pose[1:4]` |
| 姿态 | `rpy: [roll,pitch,yaw]` | 四元数 `pose[4:8]` |
| 时间戳 | 没有统一字段 | `pose[0]` |
| 业务属性 | `meta` | `metadata` |
| 邻接信息 | 不在顶点中，以边为准 | 可保存 `real_neighbors` |
| 不可连接候选 | 无 | `fake_neighbors` |
| 点云地图 | 顶点顶层 `pcd` | 默认没有专用字段，可放 metadata |
| 到点精度 | 顶点顶层 `acc` | 默认没有专用字段，可放 metadata |
| 可否掉头 | 顶点顶层 `turnable` | 默认没有专用字段，可放 metadata |

两个格式的姿态不能直接复制：

```text
TopoSingle:   roll, pitch, yaw
nav2_route3d: qx, qy, qz, qw
```

必须进行 RPY 到四元数转换。

### 4.4 边字段对比

| 含义 | TopoSingle | nav2_route3d |
| --- | --- | --- |
| 边 ID | `edges` 的字符串键 | `edge.id` |
| 两个端点 | `v: [a,b]` | `start_id`, `end_id` |
| 基础代价 | `weight` | `cost` |
| 方向 | `meta.dir` | `bidirectional` + start/end 顺序 |
| 扩展属性 | `meta` | `metadata` |
| 路线事件 | 无统一结构 | `operations` |

当前 nav2_route3d 的边评分器可以读取如下 metadata：

```json
{
  "closed": false,
  "terrain_class": "normal",
  "penalty": 0.0,
  "risk": 0.0,
  "max_slope": 0.7,
  "slope_cost_scale": 1.0,
  "edge_type": "steep_slope",
  "speed_limit_mps": 0.2
}
```

当前 TopoSingle 的 `isSlope` 位于节点上，nav2_route3d 的规划代价则主要从边 metadata
读取。因此仅转换字段名称还不够，还需要决定“哪些相邻边属于坡段”。

### 4.5 为什么不能直接加载

`nav2_route3d::loadGraph3D()` 的非 GeoJSON 分支直接遍历：

```text
payload.nodes[]
payload.edges[]
```

并要求节点存在 `id/pose`，边存在 `id/start_id/end_id`。当前文件提供的是
`vertices.{id}.pos/rpy` 和 `edges.{id}.v`，所以不能直接传给 nav2_route3d。

当前工作区已经有转换函数 `build_nav2_from_topology()`，用于把这种对象式拓扑图转换成
nav2_route3d 数组式图。但是现有实现还有以下限制：

- 只接受 `dir=0` 的无向边。
- 把节点地面高度加上 `body_height` 后写入 nav2 图。
- 只保留少量已知顶点属性。
- 边 metadata 只输出 `edge_source/edge_type/topology_source`。
- `slopeAnnotation` 不会进入 nav2_route3d 图级 metadata。
- 后续添加的速度、控制器、风险等未知字段目前不会自动透传。

因此转换器需要在控制功能开发前增加“未知 metadata 透传”和明确的字段映射。

## 5. 与 nav2_route3d GeoJSON 的区别

nav2_route3d 还支持 GeoJSON `FeatureCollection`：

```json
{
  "type": "FeatureCollection",
  "properties": {
    "frame_id": "map",
    "metadata": {}
  },
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [1.0, 2.0, 0.0]
      },
      "properties": {
        "kind": "node",
        "id": 1,
        "timestamp": 0.0,
        "orientation": [0, 0, 0, 1]
      }
    }
  ]
}
```

TopoSingle 不是 GeoJSON：

- 没有 `FeatureCollection`。
- 节点不是 `Point Feature`。
- 边不是 `LineString Feature`。
- 坐标和属性没有按 GeoJSON 的 geometry/properties 分层。

GeoJSON 更适合通用地图工具和图编辑器；TopoSingle 更接近参考业务系统的专用配置文件。

## 6. 与 graph_pid_ws_refactor 的区别

### 6.1 结构层面高度相似

参考工程的 `GraphManager::loadGraphFromJson()` 直接读取：

```text
version
isTunnel（可选）
vertices.{vertex_id}
edges.{edge_id}
```

顶点主要读取：

```text
pos
rpy
meta.type
meta.typeId
meta.chargingType（可选）
pcd
turnable（可选，默认 true）
acc
adjustYaw（可选，默认 true）
```

边主要读取：

```text
v
weight
turnable（可选，默认 true）
meta.gait
meta.v
meta.w
meta.h
meta.obs
meta.dir
meta.angle
meta.ob
meta.gridMap（obs=4 时必需）
```

当前文件已经具备参考加载器要求的关键必填字段：

| 参考加载器要求 | 当前文件 | 结果 |
| --- | --- | --- |
| 顶层 `version` | 有，值为 0 | 可以读取 |
| 顶层 `vertices` 对象 | 有 | 可以读取 |
| 顶层 `edges` 对象 | 有 | 可以读取 |
| 顶点 `pos[3]` | 有 | 可以读取 |
| 顶点 `rpy[3]` | 有 | 可以读取 |
| 顶点 `meta.type/typeId` | 有，均为 0 | 可以读取 |
| 顶点 `pcd` | 有，但为空字符串 | 语法可读，业务上需处理 |
| 顶点 `acc` | 有，值为 0.5 | 可以读取 |
| 边 `v[2]` | 有 | 可以读取 |
| 边 `weight` | 有 | 可以读取 |
| 边 `meta.dir` | 有，全部为 0 | 按双向边使用 |

所以从 JSON 结构和参考加载器代码判断，当前文件属于参考格式的一个精简子集。

### 6.2 缺省后参考工程会采用的值

当前边没有参考工程的多数控制属性。加载器会采用默认值：

| 字段 | 当前是否存在 | 参考加载后的默认行为 |
| --- | --- | --- |
| `meta.gait` | 否 | `0` |
| `meta.v` | 否 | `1.0` |
| `meta.w` | 否 | `0.0` |
| `meta.h` | 否 | `0.0` |
| `meta.obs` | 否 | `0`，普通停障/默认分支 |
| `meta.dir` | 是 | `0`，无向/双向 |
| `meta.angle` | 否 | `0.0` |
| `meta.ob` | 否 | 四个边界值均为 `0.0` |
| 边 `turnable` | 否 | `true` |
| 顶点 `adjustYaw` | 否 | `true` |
| 顶层 `isTunnel` | 否 | `false`，普通模式 |

这意味着参考工程读取当前文件后会进入普通图语义，不会自动进入管廊模式，也不会自动选择
Grid A* + DWA。

### 6.3 当前属性中会被参考工程忽略的字段

参考加载器没有读取以下当前字段：

```text
frame_id
type（顶层）
slopeAnnotation
meta.isCorner
meta.isSlope
meta.source
meta.sourceDistance
meta.sourceNodeIndex
meta.turnDeg
```

因此：

- `isSlope=true` 不会让参考工程自动切换步态、速度或控制器。
- `isCorner=true` 不会直接改变参考 PID 跟踪策略。
- `turnDeg` 不会进入参考工程的转弯代价。
- `slopeAnnotation` 的 6 个坡段只供本项目使用。
- `frame_id=map` 虽然写在文件中，但参考加载器不执行坐标变换。

如果要让坡段影响参考式控制，应把坡段结果映射为边属性，例如速度、步态、障碍模式或
控制器策略，而不能只依赖顶点 `isSlope`。

### 6.4 pcd 为空是实际接入风险

当前每个顶点都是：

```json
"pcd": ""
```

参考加载器本身允许空字符串，并会将空字符串注册成一个 PCD ID。但是后续规划接口会根据
调用参数中的 `pcdName` 查找当前楼层/地图：

```text
pcdNameToIdMap.find(pcdName)
```

如果调用者传入真实地图名，而图中只有空字符串，规划会报告当前 `pcdName` 不存在。
所以：

- 只做格式解析测试时，空 `pcd` 可以通过。
- 接入参考工程的多地图/多楼层规划时，必须填入与定位系统一致的真实 PCD 名称。

### 6.5 Z 坐标语义需要统一

当前 TopoSingle 在 Effi 链路中按“地面路径”使用，局部规划器会再增加机器人机身高度。
nav2_route3d 的 `pose.tz` 和参考 GraphManager 都直接使用文件里的 Z，不会自动理解它是
地面高度还是机身中心高度。

因此格式转换必须明确：

```text
TopoSingle.pos.z = 地面高度？
nav2_route3d.pose.tz = 机身/雷达轨迹高度？
参考工程 GraphNode.pose_z = 定位输出高度？
```

如果三者差一个 `body_height`，最近节点、楼层判断、坡度和局部规划都会受到影响。不能因为
字段名字都叫 Z 就直接复制。

### 6.6 参考工程的特殊边属性

参考工程通过边属性控制规划与执行。典型完整边可以写成：

```json
"12": {
  "v": [5, 6],
  "weight": 1.2,
  "turnable": true,
  "meta": {
    "dir": 0,
    "gait": 0,
    "v": 0.4,
    "w": 0.0,
    "h": 0.0,
    "obs": 0,
    "angle": 0.0,
    "ob": [0.0, 0.0, 0.0, 0.0]
  }
}
```

需要 Grid A* + DWA 的边则使用类似：

```json
"meta": {
  "dir": 0,
  "obs": 4,
  "gridMap": "floor_5_grid"
}
```

当前文件没有这些属性，所以加载后采用普通默认控制行为。

## 7. 三种格式的字段映射

### 7.1 顶点映射

| TopoSingle | nav2_route3d | graph_pid_ws_refactor |
| --- | --- | --- |
| 字符串键 `"1"` | `nodes[].id=1` | 字符串键 `"1"` |
| `pos[0:3]` | `pose[1:4]` | `pos[0:3]` |
| `rpy` | 转四元数到 `pose[4:8]` | `rpy` |
| `meta` | `metadata` | `meta`，但只解析部分固定字段 |
| `pcd` | 建议放 `metadata.pcd` | 顶点必需字段 |
| `acc` | 建议放 `metadata.acc` | 顶点必需字段 |
| `turnable` | 建议放 `metadata.turnable` | 顶点可选字段 |
| `isCorner` | `metadata.is_corner` | 默认忽略 |
| `isSlope` | `metadata.is_slope` | 默认忽略 |

### 7.2 边映射

| TopoSingle | nav2_route3d | graph_pid_ws_refactor |
| --- | --- | --- |
| 字符串键 `"1"` | `edges[].id=1` | 字符串键 `"1"` |
| `v[0]` | `start_id` | `v[0]` |
| `v[1]` | `end_id` | `v[1]` |
| `weight` | `cost` | `weight` |
| `meta.dir=0` | `bidirectional=true` | 双向边 |
| `meta.dir=1` | `bidirectional=false`，保持端点顺序 | `v[0] -> v[1]` |
| `meta.dir=2` | `bidirectional=false`，交换端点 | 加载时交换端点 |
| `meta.source` | `metadata.topology_source` | 默认忽略 |
| 坡度业务属性 | `metadata.edge_type/max_slope/...` | 应转换成 `gait/v/obs/...` |

## 8. 直接兼容性测试结论

### 8.1 直接传给 nav2_route3d

结论：不可直接加载。

主要原因：

```text
vertices object != nodes array
pos+rpy != pose[8]
v+weight+dir != start_id+end_id+cost+bidirectional
```

应先运行/完善项目中的 TopoGraph 到 nav2_route3d 转换器。

### 8.2 直接传给 graph_pid_ws_refactor

结论：加载器结构上基本兼容，但仅适合初步解析和普通单地图测试。

已经兼容：

- 顶点和边容器形式。
- 顶点 ID、位置、RPY。
- `type/typeId`。
- `acc/turnable`。
- 边端点、权重和方向。

仍需补齐或确认：

- 真实 `pcd` 名称。
- 地面 Z 与定位 Z 的基准差。
- 是否需要 `isTunnel`。
- 各边的速度、步态和障碍策略。
- 特殊 DWA 边的 `obs=4 + gridMap`。
- 坡点/拐点如何映射为参考工程真正读取的属性。

## 9. 对后续工程的建议

### 9.1 保持 topoGraph_data.json 为业务权威图

建议保留本项目格式作为骨架生成和人工标注的权威数据：

```text
topoGraph_data.json
        ├── 转换为 nav2_route3d JSON
        └── 转换为 graph_pid 兼容 JSON
```

不要让三套模块分别维护三份互相独立的地图，否则节点和边修改后很容易不同步。

### 9.2 最短路径不要使用 topoSingle

当前文件是线性兼容产物，边基本为：

```text
1-2, 2-3, 3-4, ...
```

它不能表达当前智能骨架中的任意回环和分支。后续 A* 应加载：

```text
topoGraph_data.json
```

`topoSingle_data_slope.json` 可以继续用于旧线性控制器兼容和坡度算法验证。

### 9.3 增加独立 schema_version

建议新增：

```json
"schema_version": 2
```

当前 `version` 可以继续表示图内容更新版本，避免和文件结构版本混用。

### 9.4 新属性必须端到端透传

加载器遇到未知 `meta` 字段时可以忽略，但转换和重新保存时必须保留。否则以后加入：

```text
controller
local_planner
speed_limit_mps
terrain_type
risk
enabled
obs
gridMap
gait
```

经过坡度标注或格式转换后会消失。

### 9.5 区分几何属性与执行属性

建议将属性分为：

```text
几何/生成属性：
source, sourceDistance, isCorner, isSlope, turnDeg, component

全局规划属性：
enabled, direction, weight, risk, terrain_type

局部规划/控制属性：
local_planner, controller, speed_limit_mps, gait, obs, gridMap
```

骨架生成器只负责可靠生成几何属性；后续标注器或地图编辑器再补充业务执行属性。

## 10. 最终判断

当前 `topoSingle_data_slope.json`：

- 与当前 `nav2_route3d` 的原生 JSON/GeoJSON 有明显结构差异，不能直接加载。
- 与 `graph_pid_ws_refactor` 的点边文件结构基本一致，可以作为兼容输入的基础。
- `isCorner/isSlope/slopeAnnotation` 是本项目扩展，参考工程不会自动使用。
- `pcd=""` 和 Z 高度基准是直接接入参考工程前必须解决的问题。
- 当前单链只适合旧控制器兼容；带分支、回环和最短路径的正式系统应使用
  `topoGraph_data.json`。

因此，不建议为了接某个模块而放弃现有格式。更合适的方案是确定一份权威
`topoGraph_data.json` schema，并维护两个经过测试的单向适配器，将它分别转换成
nav2_route3d 和参考 graph_pid 所需的运行格式。
