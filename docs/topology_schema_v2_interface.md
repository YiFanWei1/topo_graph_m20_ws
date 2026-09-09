# Route3D Topology Schema V2 接口说明

## 1. 适用范围

从本版本开始，产品 launch 生成的以下四个拓扑文件统一使用 Schema V2：

| 文件 | 生成阶段 | 图语义 |
| --- | --- | --- |
| `live/topoSingle_live.json` | 按 `1` 后实时更新 | 未去重的线性预览 |
| `live/topoGraph_live.json` | 按 `1` 后实时更新 | 退回去重、拆边和闭环后的智能图 |
| `topoSingle_data.json` | 按 `2` 后离线生成 | 旧线性控制器兼容文件 |
| `topoGraph_data.json` | 按 `2` 后离线重放 | 智能拓扑权威文件，供后续图搜索使用 |

本格式继续使用参考工程同类的 `vertices + edges` 点边容器，但新增控制属性采用本项目
自己的名称。原有的拐点、坡点、交汇点、时间戳、component、边来源和构图统计均保留。

> `graph_pid_ws_refactor` 的旧加载器不会识别本文新增的自有字段名。若要让旧加载器直接
> 控制机器人，需要单独做字段适配；当前字段是为本项目后续 A*、控制器选择和 PID/局部
> 规划切换预留的接口。

## 2. 最小结构示例

```json
{
  "version": 239,
  "schema": {
    "name": "route3d_topology",
    "version": 2
  },
  "type": 0,
  "frame_id": "camera_init",
  "sceneMode": "normal",
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
        "turnDeg": 0.0
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
      "weight": 1.0,
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

## 3. 顶层接口

| JSON 路径 | 类型 | 默认值 | 生成器是否填写 | 含义 |
| --- | --- | --- | --- | --- |
| `version` | integer | 生成器内部值 | 是 | 图内容修订号；实时图变化时递增，不是格式版本 |
| `schema.name` | string | `route3d_topology` | 是 | 本项目格式标识 |
| `schema.version` | integer | `2` | 是 | JSON 接口版本 |
| `type` | integer | `0` | 是 | 保留的拓扑类别 |
| `frame_id` | string | 来自里程计/图文件 | 是 | 所有 `pos` 所属坐标系 |
| `sceneMode` | string | `normal` | 是 | 场景模式；预留 `normal`、`tunnel` 等模式 |
| `status` | string | 无 | 实时图/智能图 | `recording` 或 `complete` |
| `topology_mode` | string | 无 | 智能图 | `incremental_forest` 或 `incremental_graph` |
| `vertices` | object | `{}` | 是 | key 为稳定顶点 ID 的对象 |
| `edges` | object | `{}` | 是 | key 为稳定边 ID 的对象，允许 ID 有空洞 |
| `generation` | object | 无 | 智能图 | 退回、闭环、拆边和 component 统计 |
| `slopeAnnotation` | object | 无 | 启用坡度分析时 | 坡段检测参数、计数和区间报告 |

`version` 与 `schema.version` 不可混用：前者表示这一张图修改了多少次，后者决定解析器
应按哪一版接口读取字段。

## 4. 顶点接口

顶点完整路径为 `vertices.<vertex_id>`。

| 字段 | 类型 | 默认值 | 当前用途 |
| --- | --- | --- | --- |
| `pos` | `[number, number, number]` | 无 | 地面点 `[x,y,z]`，单位 m |
| `rpy` | `[number, number, number]` | 无 | roll、pitch、yaw，单位 rad |
| `pcd` | string | `""` | 关联 PCD 名称；当前生成器默认不绑定单独名称 |
| `acc` | number | `0.5` | 该点作为任务最终目标时的到达容差，单位 m |
| `turnable` | boolean | `true` | 保留的顶点可转向属性 |
| `alignFinalYaw` | boolean | `true` | **新增**；到达该点时是否需要对齐目标 yaw |
| `mustPassThrough` | boolean | 拐点为 `true`，其余为 `false` | **新增**；作为中间途经点时是否必须进入其通过半径 |
| `passRadiusM` | number | 拐点 `0.20`，其余 `0.45` | **新增**；中间途经约束半径，单位 m，与 `acc` 相互独立 |
| `meta.type` | integer | `0` | 保留的点类型 |
| `meta.typeId` | integer | `0` | 保留的点类型编号 |
| `meta.chargingMode` | integer | `0` | **新增**；充电/停靠模式，`0` 表示普通点 |
| `meta.isCorner` | boolean | `false` | 是否为有效转角点 |
| `meta.isSlope` | boolean | `false` | 是否位于坡段或坡段缓冲区 |
| `meta.isJunction` | boolean | `false` | 是否为拆边产生的交汇点；智能图提供 |
| `meta.turnDeg` | number | `0.0` | 该点检测到的最大转角，单位 degree |
| `meta.source` | string | 由生成器决定 | 点来源，例如 `incremental_topology` |
| `meta.sourceStamp` | number | 无 | 来源 pose 时间戳；智能图提供，单位 s |
| `meta.sourceDistance` | number | 无 | 在线/线性路线累计距离，单位 m |
| `meta.sourceNodeIndex` | integer | 无 | 线性离线结果对应的原始轨迹下标 |
| `meta.component` | integer | 无 | 连通分量编号；智能图提供 |
| `meta.state` | string | `confirmed` | 在线确认状态 |

生成器对三种几何语义采用明确的优先级：平地交汇点
`isJunction=true` 会同时标成 `isCorner=true`；坡点优先级更高，任何
`isSlope=true` 的点都会强制 `isCorner=false`。因此交汇点可以同时是坡点，
但拐点和坡点互斥。

`acc` 与 `passRadiusM` 不应混用：Dijkstra 请求中的终点无论是不是普通点，都使用该点
的 `acc`；路径中间的拐点或人工指定的必经普通点使用 `mustPassThrough + passRadiusM`。
因此无需把普通目标点伪装成拐点，也无需依赖“目标点恰好是拐点”才能提高到点精度。

## 5. 边接口

边完整路径为 `edges.<edge_id>`。

| 字段 | 类型 | 默认值 | 当前/后续用途 |
| --- | --- | --- | --- |
| `v` | `[integer, integer]` | 无 | 两个端点 ID；不再假定为 `i` 与 `i+1` |
| `weight` | number | 三维欧氏距离 | A*/Dijkstra 的基础代价，单位 m |
| `rotationAllowed` | boolean | `true` | **新增**；是否允许在该边执行转向动作 |
| `meta.dir` | integer | `0` | 现有兼容字段：`0` 双向、`1` 首到次、`2` 次到首 |
| `meta.source` | string | 由生成器决定 | `discovery`、`edge_split`、`loop_closure` 等 |
| `meta.locomotionMode` | integer | `0` | **新增**；步态/运动模式编号 |
| `meta.linearSpeedMps` | number | `1.0` | **新增**；边期望或上限线速度，单位 m/s |
| `meta.angularSpeedRadps` | number | `0.0` | **新增**；边角速度配置，单位 rad/s；`0` 表示未专门配置 |
| `meta.heightOffsetM` | number | `0.0` | **新增**；该边的车身高度补偿，单位 m |
| `meta.obstacleMode` | integer | `0` | **新增**；障碍处理模式编号，`0` 为默认策略 |
| `meta.travelMode` | string | `bidirectional` | **新增**；可读方向：`bidirectional`、`first_to_second`、`second_to_first` |
| `meta.headingAngleRad` | number | `0.0` | **新增**；该边的期望航向，单位 rad；`0` 目前表示未专门配置 |
| `meta.obstacleBoxM` | `[number,number,number,number]` | 全 `0.0` | **新增**；局部障碍框 `[x_min,x_max,y_min,y_max]`，单位 m |
| `meta.gridMapName` | string | `""` | **新增**；该边绑定的栅格地图名称 |
| `meta.controllerMode` | string | `auto` | **新增**；控制器提示，预留 `auto`、`pid`、`local_planner` |

生成时 `travelMode` 由 `dir` 转换得到。后续新解析器建议优先读取 `travelMode`，旧文件
缺失时再回退到 `dir`；若两者同时存在但矛盾，应拒绝加载并报告具体边 ID。

## 6. 与参考工程字段的语义映射

| 参考工程字段 | 本项目 Schema V2 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `isTunnel` | `sceneMode` | `normal` | 布尔值改为可扩展字符串 |
| `vertices.*.meta.chargingType` | `vertices.*.meta.chargingMode` | `0` | 名称独立，语义预留 |
| `vertices.*.adjustYaw` | `vertices.*.alignFinalYaw` | `true` | 到点姿态对齐 |
| `edges.*.turnable` | `edges.*.rotationAllowed` | `true` | 边上的转向许可 |
| `edges.*.meta.gait` | `edges.*.meta.locomotionMode` | `0` | 步态/运动模式 |
| `edges.*.meta.v` | `edges.*.meta.linearSpeedMps` | `1.0` | 明确单位 |
| `edges.*.meta.w` | `edges.*.meta.angularSpeedRadps` | `0.0` | 明确单位 |
| `edges.*.meta.h` | `edges.*.meta.heightOffsetM` | `0.0` | 明确单位 |
| `edges.*.meta.obs` | `edges.*.meta.obstacleMode` | `0` | 障碍策略编号 |
| `edges.*.meta.dir` | `edges.*.meta.travelMode` | `bidirectional` | 当前同时保留原 `dir` 兼容字段 |
| `edges.*.meta.angle` | `edges.*.meta.headingAngleRad` | `0.0` | 明确单位 |
| `edges.*.meta.ob` | `edges.*.meta.obstacleBoxM` | `[0,0,0,0]` | 明确单位与顺序 |
| `edges.*.meta.gridMap` | `edges.*.meta.gridMapName` | `""` | 栅格地图引用 |
| 无直接对应 | `edges.*.meta.controllerMode` | `auto` | 为后续 PID/局部规划器切换预留 |

参考工程没有的 `isCorner`、`isSlope`、`isJunction`、`sourceStamp`、`component`、
`generation`、`slopeAnnotation` 等字段不会因升级丢失。

## 7. 当前默认值的运行边界

Schema V2 当前同时服务新旧两条控制链：

- 旧线性导航仍读取 `topoSingle_data.json`，并主要使用位置、边顺序和拐点属性。
- 新 C++ Dijkstra 读取 `topoGraph_data.json` 的真实任意端点边；语义切片器随后消费路径
  中点边属性，PID 消费切片后的 `RouteTaskArray`。
- 新生成的 Schema V2 文件会显式写入 `mustPassThrough` 与 `passRadiusM`；早于本次修改
  已生成的 V2 文件可继续加载，C++ 解析器会按 `isCorner` 推导相同默认值。
- `linearSpeedMps`、`obstacleMode`、`controllerMode` 已接入执行器；`auto` 在平地解析为 PID，
  在坡段解析为 `efficient_3d_local_planner`，`obstacleMode=0` 为平地停障等待。
- PID 与 effi 同时常驻，Go2 适配器依据 `/route3d_controller/active_source` 自动仲裁；当前
  自动坡段链不使用 DWA。显式的其他外部控制器仍会进入 `HANDOVER_REQUIRED`。
- 坡点会产生步态切换意图和 `WAITING_TRANSITION`。Go2 适配器执行并确认步态后，执行器
  自动切换到下一控制器，无需人工调用 `continue`。

## 8. `regu` 实测结果

本版本通过以下真实链路生成验证数据：

```bash
ros2 launch route3d_product_demo live_route_product.launch.py
# 按 1
ros2 bag play /home/wei/bag/regu --clock --rate 4.0
# 播放结束后按 2，保存名 regu_schema_v2
```

结果目录为 `data/regu_schema_v2`：

| 检查项 | 结果 |
| --- | ---: |
| 离线智能图顶点 | 109 |
| 离线智能图边 | 108 |
| 退回次数 | 7 |
| 抑制重复距离 | 4.914 m |
| 拆边次数 | 1 |
| 闭环次数 | 0 |
| 实时/离线智能图结构 | 一致 |
| 四个 Topology JSON 的 Schema | 均为 `route3d_topology / 2` |
