# Route3D Schema V2 点边数据格式

## 1. 概述

当前导航系统使用 `Route3D Topology Schema V2`。图文件以 JSON 保存，由顶点集合
`vertices` 和边集合 `edges` 构成。

实机导航使用的权威文件为：

```text
data/<地图名称>/topoGraph_data.json
```

最小顶层结构：

```json
{
  "version": 1,
  "schema": {
    "name": "route3d_topology",
    "version": 2
  },
  "type": 0,
  "frame_id": "camera_init",
  "sceneMode": "normal",
  "status": "complete",
  "vertices": {},
  "edges": {}
}
```

## 2. 顶层字段

| 字段 | 类型 | 示例或默认值 | 作用 |
| --- | --- | --- | --- |
| `version` | integer | `10` | 图内容修订号；不是格式版本 |
| `schema.name` | string | `route3d_topology` | 固定的格式名称 |
| `schema.version` | integer | `2` | Schema 接口版本，当前必须为 `2` |
| `type` | integer | `0` | 保留的拓扑类别 |
| `frame_id` | string | `camera_init` | 所有顶点坐标所属的坐标系 |
| `sceneMode` | string | `normal` | 场景模式 |
| `status` | string | `recording` 或 `complete` | 图的生成状态 |
| `topology_mode` | string | `incremental_graph` | 拓扑生成模式 |
| `vertices` | object | `{"1": {...}}` | 顶点集合；键为稳定顶点 ID |
| `edges` | object | `{"1": {...}}` | 边集合；键为稳定边 ID |
| `generation` | object | 生成器填写 | 退回、闭环、拆边及连通分量统计 |
| `slopeAnnotation` | object | 启用坡度分析时生成 | 坡段检测参数、数量和区间 |

`version` 和 `schema.version` 不可混用：前者表示图修改次数，后者决定解析格式。

## 3. 顶点格式

顶点完整路径为 `vertices.<vertex_id>`。对象键是字符串形式的正整数，例如 `"1"`。

| 字段 | 类型 | 默认值 | 当前用途 |
| --- | --- | --- | --- |
| `pos` | `[number, number, number]` | 必填 | 地面点 `[x,y,z]`，单位 m |
| `rpy` | `[number, number, number]` | 必填 | `[roll,pitch,yaw]`，单位 rad |
| `pcd` | string | `""` | 关联关键帧 PCD 名称 |
| `acc` | number | `0.5` | 该点作为最终目标时的到达容差，单位 m |
| `turnable` | boolean | `true` | 是否允许在该点转向 |
| `alignFinalYaw` | boolean | `true` | 到达最终目标时是否对齐目标 yaw |
| `mustPassThrough` | boolean | 拐点 `true`，其他点 `false` | 作为中间点时是否必须进入通过半径 |
| `passRadiusM` | number | 拐点 `0.20`，其他点 `0.45` | 中间点通过半径，单位 m |
| `meta.type` | integer | `0` | 业务点类型 |
| `meta.typeId` | integer | `0` | 业务点类型编号 |
| `meta.chargingMode` | integer | `0` | 充电或停靠模式；`0` 为普通点 |
| `meta.isCorner` | boolean | `false` | 是否为有效拐点 |
| `meta.isSlope` | boolean | `false` | 是否位于坡段或坡段缓冲区 |
| `meta.isJunction` | boolean | `false` | 是否为交汇点或拆边点 |
| `meta.turnDeg` | number | `0.0` | 检测到的最大转角，单位 degree |
| `meta.source` | string | 生成器填写 | 点来源，例如 `incremental_topology` |
| `meta.sourceStamp` | number | 生成器填写 | 来源位姿时间戳，单位 s |
| `meta.sourceDistance` | number | 可选 | 原始路线累计距离，单位 m |
| `meta.sourceNodeIndex` | integer | 可选 | 原始轨迹下标 |
| `meta.component` | integer | `0` | 所属连通分量编号 |
| `meta.state` | string | `confirmed` | 点的确认状态 |

顶点示例：

```json
"1": {
  "pos": [0.155879, -2.033063, -0.386078],
  "rpy": [0.0, 0.0, -2.116827],
  "meta": {
    "type": 0,
    "typeId": 0,
    "isCorner": true,
    "isSlope": false,
    "isJunction": false,
    "chargingMode": 0,
    "turnDeg": 98.7,
    "source": "incremental_topology",
    "sourceStamp": 1788422493.53,
    "component": 0,
    "state": "confirmed"
  },
  "pcd": "",
  "acc": 0.5,
  "turnable": true,
  "alignFinalYaw": true,
  "mustPassThrough": true,
  "passRadiusM": 0.2
}
```

### 3.1 到点约束

`acc` 和 `passRadiusM` 用途不同：

- 路线最终目标使用 `acc`；
- 中间必经点使用 `mustPassThrough + passRadiusM`；
- 普通目标点不需要伪装为拐点；
- `isCorner`、`isSlope`、`isJunction` 可以同时为 `true`，不是互斥枚举。

## 4. 边格式

边完整路径为 `edges.<edge_id>`。对象键是字符串形式的正整数；`v` 中的端点 ID 是整数。

| 字段 | 类型 | 默认值 | 当前用途 |
| --- | --- | --- | --- |
| `v` | `[integer, integer]` | 必填 | 边连接的两个顶点 ID |
| `weight` | number | 三维欧氏距离 | Dijkstra 的基础搜索代价 |
| `rotationAllowed` | boolean | `true` | 是否允许在该边执行转向 |
| `meta.dir` | integer | `0` | `0` 双向、`1` 首到次、`2` 次到首 |
| `meta.travelMode` | string | `bidirectional` | 可读方向配置 |
| `meta.source` | string | `discovery` | `discovery`、`edge_split`、`loop_closure` 等 |
| `meta.controllerMode` | string | `auto` | 控制器选择提示 |
| `meta.locomotionMode` | integer | `0` | 步态或运动模式编号 |
| `meta.linearSpeedMps` | number | `1.0` | 该边期望或上限线速度，单位 m/s |
| `meta.angularSpeedRadps` | number | `0.0` | 指定角速度，单位 rad/s；`0` 表示未单独配置 |
| `meta.heightOffsetM` | number | `0.0` | 车身高度补偿，单位 m |
| `meta.obstacleMode` | integer | `0` | 障碍处理模式编号 |
| `meta.headingAngleRad` | number | `0.0` | 指定航向，单位 rad |
| `meta.obstacleBoxM` | `[number,number,number,number]` | 全 `0.0` | 局部障碍框 `[x_min,x_max,y_min,y_max]`，单位 m |
| `meta.gridMapName` | string | `""` | 该边绑定的外部栅格地图名称 |

边示例：

```json
"1": {
  "v": [1, 2],
  "weight": 0.890073,
  "rotationAllowed": true,
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
  }
}
```

### 4.1 行驶方向

| `meta.dir` | `meta.travelMode` | 含义 |
| ---: | --- | --- |
| `0` | `bidirectional` | 双向 |
| `1` | `first_to_second` | 只允许 `v[0] -> v[1]` |
| `2` | `second_to_first` | 只允许 `v[1] -> v[0]` |

`dir` 和 `travelMode` 必须一致，否则严格解析器会拒绝加载该图。

### 4.2 控制器模式

| `controllerMode` | 行为 |
| --- | --- |
| `auto` | 平地自动选择 PID，坡段自动选择 `efficient_3d_local_planner` |
| `pid` | 强制使用 PID 控制器 |
| `efficient_3d_local_planner` | 强制使用三维局部规划器 |
| `local_planner` | 预留给显式栅格或其他局部规划任务 |

### 4.3 障碍模式

| `obstacleMode` | 行为 |
| ---: | --- |
| `0` | 平地停障等待；障碍消失后恢复原任务 |
| `1` | 近场停车并请求重规划，远场请求后台重规划 |
| `3` | 忽略普通地形障碍，但保留严重碰撞和急停保护 |
| `4` | 显式外部栅格任务；必须填写非空 `gridMapName` |

## 5. 坡点、控制器和步态解析规则

一条边的任意端点满足 `meta.isSlope=true`，该边就被视为坡段。因此，进入第一个坡点前
会提前切换步态，离开最后一个坡点后再恢复普通步态。

| 条件 | 有效控制器 | 有效步态 | 有效障碍模式 |
| --- | --- | --- | ---: |
| 两端均为普通点，`controllerMode=auto` | `pid` | `static_walk` | 保持边配置，默认 `0` |
| 任意端点为坡点，`controllerMode=auto` | `efficient_3d_local_planner` | `switch_gait_3` | 默认 `0` 自动提升为 `3` |
| `controllerMode=pid` | `pid` | 仍根据有效运动模式决定 | 保持边配置 |
| `controllerMode=efficient_3d_local_planner` | `efficient_3d_local_planner` | 仍根据有效运动模式决定 | 保持边配置 |

默认运动模式解析：

| 场景 | 原始 `locomotionMode` | 有效模式 | 步态命令 |
| --- | ---: | ---: | --- |
| 普通路面 | `0` | `0` | `static_walk` |
| 坡段 | `0` | `2` | `switch_gait_3` |

如果边已经显式填写了非默认 `locomotionMode`，切片器尊重边属性，不使用坡点默认值覆盖。

## 6. 完整示例

```json
{
  "version": 1,
  "schema": {
    "name": "route3d_topology",
    "version": 2
  },
  "type": 0,
  "frame_id": "camera_init",
  "sceneMode": "normal",
  "status": "complete",
  "topology_mode": "incremental_graph",
  "vertices": {
    "1": {
      "pos": [0.0, 0.0, -0.4],
      "rpy": [0.0, 0.0, 0.0],
      "meta": {
        "type": 0,
        "typeId": 0,
        "isCorner": false,
        "isSlope": false,
        "isJunction": false,
        "chargingMode": 0,
        "source": "incremental_topology",
        "sourceStamp": 0.0,
        "component": 0,
        "turnDeg": 0.0,
        "state": "confirmed"
      },
      "pcd": "",
      "acc": 0.5,
      "turnable": true,
      "alignFinalYaw": true,
      "mustPassThrough": false,
      "passRadiusM": 0.45
    },
    "2": {
      "pos": [1.0, 0.0, -0.3],
      "rpy": [0.0, 0.1, 0.0],
      "meta": {
        "type": 0,
        "typeId": 0,
        "isCorner": false,
        "isSlope": true,
        "isJunction": false,
        "chargingMode": 0,
        "source": "incremental_topology",
        "sourceStamp": 1.0,
        "component": 0,
        "turnDeg": 0.0,
        "state": "confirmed"
      },
      "pcd": "",
      "acc": 0.5,
      "turnable": true,
      "alignFinalYaw": true,
      "mustPassThrough": false,
      "passRadiusM": 0.45
    }
  },
  "edges": {
    "1": {
      "v": [1, 2],
      "weight": 1.005,
      "rotationAllowed": true,
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
      }
    }
  }
}
```

在这个示例中，点 `2` 是坡点，因此边 `1` 会被解析为坡段任务：

```text
controller = efficient_3d_local_planner
gait       = switch_gait_3
obstacle   = 3
```

## 7. 严格校验要求

- `schema.name` 必须为 `route3d_topology`；
- `schema.version` 必须为 `2`；
- `frame_id` 和 `sceneMode` 不能为空；
- 顶点 ID 和边 ID 必须是正整数形式的字符串；
- `pos`、`rpy` 必须各包含三个有限数值；
- 边的 `v` 必须包含两个已存在且不同的顶点 ID；
- 同一对端点不能重复建边；
- `weight`、`acc`、`passRadiusM` 不能为负数；
- `obstacleBoxM` 必须正好包含四个数值；
- `dir` 与 `travelMode` 必须一致；
- `obstacleMode=4` 时，`gridMapName` 必须非空。
