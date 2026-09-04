# Route3D 实时骨架与智能拓扑生成逻辑

本文档描述 `live_route_product.launch.py` 当前实际运行的完整逻辑。内容以当前源码和
2026-09-02 的远端实机测试为准，重点说明：数据怎样进入系统、节点和边怎样生成、原路
退回怎样抑制、闭环怎样确认、PCD 起什么作用，以及当前仍然存在的限制。

> 当前系统已经是一个完整的“采集—在线构图—离线复核—保存—RViz 展示”骨架 Demo，
> 但闭环后的持续路线吸附、端点立即闭环和重复小环折叠仍需优化。因此它是可演示版本，
> 不是最终生产版拓扑生成器。

## 1. 系统目标与边界

智能拓扑生成器要把机器人连续位姿轨迹转换为无向三维拓扑图：

- 未知区域每累计约 `1.0 m` 增加一个稳定 ID 节点。
- 沿已经生成的相邻边原路退回时，只移动图上的活动游标，不重复创建节点和边。
- 从旧边中间离开时，在离开点拆边并从交汇点创建新分支。
- 重新进入图距离足够远的旧边时，经过连续几何匹配和 PCD 验证后增加闭环边。
- 定位发生大跳变时创建新的不连通 component，跳变两端不连边。
- 在线和离线使用同一个 `route3d_topology_core`，离线重放结果作为最终权威结果。

当前明确不做的事情：

- 不根据 PCD 自动创建机器人没有走过的捷径边。
- 不进行全局点云建图、位姿图优化或 ICP 回环优化。
- 不把相距较近的整条平行轨迹自动压缩为同一条中心线。
- 不包含环图最短路径规划和导航控制；这些属于后续功能包。
- Nav2 自带的图生成代码不参与实时智能拓扑判定。

## 2. 包和文件的职责

| 模块 | 当前职责 |
| --- | --- |
| `route3d_product_demo` | 启动键盘工作流、RViz、记录器和在线骨架节点；按 `2` 后组织离线处理与保存。 |
| `route3d_data_recorder` | 近似同步 `/lio_odom` 与 `/cloud_registered_body`，写入 pose/PCD，并发布同步后的位姿。 |
| `route3d_online_skeleton` | 同时维护旧线性预览和新智能拓扑，发布实时 RViz Path/Marker。 |
| `route3d_topology_core` | 在线与离线共享的图结构、状态机、原路退回、全局重入、拆边、闭环和图感知坡点逻辑。 |
| `route3d_topology_core/pcd_validation.py` | 使用同步关键帧的体素重合率验证闭环候选。 |
| `nav2_route3d route_graph_builder_3d` | 按 `2` 后生成旧流程使用的原始 `route3d_graph.json`，不负责实时智能闭环。 |
| `route3d_bag_tools` | 生成旧线性目标与 `topoSingle_data.json`。 |
| `topo_graph_tools` | 把 `topoGraph_data.json` 转成保留任意端点关系的 `selected_route3d_graph.json`，供最终 RViz 检查。 |

核心源码：

- [`topology.py`](../src/route3d_topology_core/route3d_topology_core/topology.py)
- [`pcd_validation.py`](../src/route3d_topology_core/route3d_topology_core/pcd_validation.py)
- [`online_route_skeleton_node`](../src/route3d_online_skeleton/scripts/online_route_skeleton_node)
- [`route3d_live_keyboard`](../src/route3d_product_demo/scripts/route3d_live_keyboard)
- [`live_route_product.yaml`](../src/route3d_product_demo/config/live_route_product.yaml)

## 3. 从 launch 到位姿输入的完整数据流

```text
ros2 launch route3d_product_demo live_route_product.launch.py
        |
        +-- 可选静态 TF：map -> camera_init
        +-- 可选静态 TF：body -> base_link
        +-- route3d_live_keyboard
                  |
                  +-- 启动 RViz
                  |
                按 1
                  |
                  +-- online_route_skeleton_node
                  +-- route3d_data_recorder_node
                              |
         /lio_odom + /cloud_registered_body
                              |
                    近似时间同步并先写盘
                              |
                 /route3d/synchronized_pose
                              |
             +----------------+----------------+
             |                                 |
       旧线性实时预览                    智能图状态机
  live/topoSingle_live.json       live/topoGraph_live.json
             |                                 |
             +--------------- RViz ------------+
```

记录器使用 `sync_slop=0.05 s` 和队列深度 `100`。只有成功同步并写入关键帧的数据才会发布
到 `/route3d/synchronized_pose`，因此在线骨架使用的位姿能够对应到
`key_frames/<frame_index>.pcd`。

每个输入位姿为：

```text
[timestamp, x, y, z, qx, qy, qz, qw]
```

其中 `z` 是机身/定位位姿高度。写入拓扑 JSON 时统一减去 `body_height=0.40 m`，得到当前
业务使用的地面高度。几何判定本身使用未减机身高度的原始位姿。

## 4. 为什么同时存在两个实时 JSON

### 4.1 `live/topoSingle_live.json`

这是旧线性预览：

- 完全按实际运动距离累计节点。
- 原路退回和重复绕圈仍会产生新的线性节点。
- 边只连接线性序列中的相邻节点。
- 主要用于兼容旧控制器语义和对照实际运动。

### 4.2 `live/topoGraph_live.json`

这是当前需要关注的智能拓扑：

- 使用真正的无向邻接图。
- 支持稳定节点 ID、任意端点连边、原路退回抑制、拆边、分支和闭环。
- RViz 的绿色、橙色和青色边都来自这个图。
- 后续最短路径规划应该读取这一套结构，而不是 `topoSingle`。

两个文件里的同一个数字 ID 不保证表示同一个逻辑节点。分析闭环关系时必须先确认读取的
是 `topoGraph_live.json`，不能用线性预览的节点编号判断智能图连边。

## 5. 智能图的基础数据结构

### 5.1 Vertex

每个节点保存：

- 稳定递增 `vertex_id`，从 1 开始，创建后不重新编号。
- 创建时的 `PoseSample`：时间戳、XYZ、四元数和同步 PCD 帧序号。
- 所属 `component`。
- `isCorner`、`turnDeg`、`isSlope`、`isJunction`。

节点 ID 当前是连续递增的。拆边只增加新节点，不修改已有节点 ID。

### 5.2 Edge

每条边保存：

- 稳定递增 `edge_id`。
- 两个无向端点 `first`、`second`。
- 来源 `source`：`discovery`、`edge_split` 或 `loop_closure`。

边被拆分时，旧边会删除并创建两条新边，因此 edge ID 允许出现空洞。重复端点边不会重复
创建，自环边被禁止。

JSON 中边的 `weight` 是两个端点之间的三维欧氏距离，`meta.dir=0` 表示双向。

### 5.3 活动游标

构图器不仅保存“最后一个节点”，还保存当前活动位置：

- 探索状态下：`current_vertex_id` 和不足一个节点间距的 `discovery_cursor`。
- 旧路跟踪状态下：`current_retrace_edge_id` 和边上的投影比例
  `current_retrace_ratio`。
- 离开确认期间：最后可信的旧顶点或旧边投影锚点，以及从首次分离开始缓存的位姿。

## 6. 输入预处理与普通节点生成

每个同步位姿依次执行以下处理：

1. 检查所有数值是否有限，并归一化四元数。
2. 把完整位姿加入 `raw_samples`，供 RViz 灰色轨迹和 PCD 历史帧查找使用。
3. 比较相邻原始消息的三维距离：若大于 `relocation_distance=0.50 m`，按定位跳变处理。
4. 比较当前位姿与上一个接受的几何位姿：若小于 `dedup_distance=0.05 m`，认为是静止抖动，
   不累计路线距离。
5. 根据当前状态进入探索、候选确认、旧路跟踪或离开确认逻辑。

在 `DISCOVERING` 状态，系统沿原始三维折线累计距离。每跨过
`target_spacing=1.0 m`，在对应线段上插值一个位姿并创建节点，再把它与当前活动节点用
`source: discovery` 边连接。停止时，末段不足 `1.0 m` 但达到 `0.05 m` 的真实终点仍会保留。

如果相邻输入位移大于 `0.50 m`：

- 结束当前状态；
- component 加 1；
- 在新位置创建新节点；
- 不连接跳变两端；
- 记录 `pose_jump` 事件并在 RViz 中显示紫色球。

## 7. 四态状态机

```text
                         候选失效：缓存轨迹按新路重放
                       +--------------------------------+
                       |                                |
                       v                                |
DISCOVERING --发现候选--> RETRACE_PENDING --确认成功--> RETRACING
     ^                        |                             |
     |                        | PCD拒绝                     | 偏离旧路
     |                        +-----------------------------+--> EXIT_PENDING
     |                                                              |
     +--------------------确认离开，缓存轨迹建分支-------------------+
                                      |
                                      +--回到同一旧路--> RETRACING
```

`RETRACE_PENDING` 由 `pending_kind` 再区分两类：

- `retrace`：从当前探索位置沿相邻旧边原路退回。
- `loop`：从当前探索位置重新进入图距离较远的旧边，可能创建闭环。

## 8. 相邻旧边原路退回：`retrace`

### 8.1 候选范围

在探索状态中，原路退回只搜索 `current_vertex_id` 的相邻边，不搜索任意空间近邻边。
投影必须满足：

- 到有限边段的 XY 距离不超过 `0.30 m`。
- 当前 Z 与边插值 Z 的差不超过 `0.20 m`。
- 位移方向与边方向夹角不超过 `35°`。
- 投影落在边段内部，并且是从活动顶点向边内部移动。

边是无向的。方向误差计算使用点积绝对值，因此正向和反向沿边都能匹配，不依赖机器人
yaw。单次 XY 位移小于 `motion_min_distance=0.10 m` 时方向误差暂时按 0 处理，避免低速
抖动导致方向不稳定。

### 8.2 反向转折的末端提交

如果连续运动方向发生约 `145°` 以上反转，并且当前未满 1 m 的探索末段已经超过
`0.05 m`，系统会先把反转前的真实位置提交为节点，再开始判断相邻边退回。这样能够保留
“走到 5 后掉头”的真实端点。

### 8.3 进入确认

候选连续匹配累计 `entry_confirmation_distance=0.30 m` 后进入 `RETRACING`：

- 不再创建重复节点和边。
- 每个有效位移累加到 `suppressed_distance_m`。
- 产生 `retrace_enter` 事件，`retrace_count` 加 1。

经过旧顶点时，只从该顶点的相邻边中选择后续边，排序规则是 XY 距离、方向误差、edge ID。
因此结果可复现，也不会在这一逻辑中跳到一条非相邻平行边。

## 9. 全局旧路重入与闭环：`loop_closure`

### 9.1 为什么需要独立于 retrace

绕一圈回到起点、靠近较早路线或在交叉口重新进入旧路时，目标旧边通常不是当前活动节点
的相邻边。因此第二期增加全局候选搜索。

### 9.2 候选旧边过滤

只有同时满足下列条件的边才参与：

- `loop_closure.enabled=true`。
- 当前图至少已有 3 条边。
- 与当前活动节点属于同一个 component。
- 从当前活动节点到候选边任一端点的最短图距离至少 `3.0 m`。
- 投影落在有限边段内部。
- XY 距离不超过 `0.40 m`。
- Z 差不超过 `0.20 m`。
- 位移方向与边方向的无向夹角不超过 `45°`。
- 不处于 PCD 拒绝后的 `1.0 m` 搜索冷却距离内。

图距离使用 Dijkstra 按边端点三维距离计算。多个全局候选按 XY 距离、方向误差和 edge ID
排序，选择唯一最优项。

发现候选时会缓存：

- 候选出现时的当前活动节点 `pending_source_vertex_id`。
- 首次命中的旧边 `pending_join_edge_id`。
- 首次命中的边上投影比例 `pending_join_ratio`。
- 候选开始时间和后续位姿缓存。

并记录 `route_reentry_candidate` 事件。

### 9.3 连续确认

候选出现不会立即连边。系统要求：

- 累计有效匹配距离至少 `0.80 m`。
- 候选首末位姿的三维净位移至少为 `0.70 × 0.80 = 0.56 m`，避免原地抖动累计距离。
- 经过候选边端点时，可以继续匹配该端点的相邻旧边。
- 拐角附近短暂匹配失败时，如果仍靠近端点，允许累计不超过 `0.80 m` 的过渡 gap。

如果确认前候选失效，所有缓存位姿会按普通探索轨迹重新播放，因此不会丢失真实新路线。

### 9.4 PCD 历史帧选择

连续距离满足后才执行 PCD 验证。当前帧是候选确认时的同步帧；历史参考帧从
`raw_samples` 中选择：

1. 时间戳必须早于“候选开始时间减 `5.0 s`”。
2. 在这些历史帧中，选择三维位置最接近确认时旧边投影点的一帧。
3. 该参考帧仍需满足 XY 不超过 `0.55 m`、Z 差不超过 `0.20 m`。

注意：当前 `minimum_time_separation=5.0 s` 用于选择 PCD 历史参考帧，并不是全局候选边的
独立硬过滤条件。当 `required=false` 且历史参考帧不可用时，系统允许退化为纯几何确认。

### 9.5 PCD 体素重合验证

当前实现不是 ICP，而是轻量体素重合：

1. 读取二进制且字段严格为 `x y z` 的关键帧 PCD。
2. 过滤非有限点以及水平距离不在 `[0.5, 12.0] m` 的点。
3. 使用对应位姿把两帧点云变换到世界坐标。
4. 按 `0.30 m` 体素量化并去重。
5. 要求每帧至少有 `100` 个有效体素。
6. 在 XY `±0.60 m`、Z `±0.30 m` 的离散平移范围搜索最大重合。
7. `交集体素数 / 两帧较小体素数` 不低于 `0.45` 时通过。

`pcd_validation.required=false` 的语义是：

- PCD 存在且成功计算，但重合率低于 `0.45`：拒绝闭环。
- PCD 文件缺失、帧号缺失、体素过少或参考帧不可用：允许退化为几何确认。

若改为 `required=true`，上述“不可用”情况也会拒绝闭环。

拒绝后产生 `route_reentry_rejected`，缓存轨迹重新按探索路线生成，并在继续移动 `1.0 m`
前暂停新的全局搜索，防止同一位置反复验证。

### 9.6 创建闭环边

PCD 通过后，系统使用候选首次出现时缓存的旧边投影作为连接位置：

- 投影距离旧边某个端点不超过 `0.45 m`：复用距离最近的端点。
- 否则拆分旧边，在投影位置创建交汇节点。
- 从候选开始时的活动节点连接到该复用/新建节点。
- 新边标记 `source: loop_closure`。
- 如果同一对端点已有边，则复用已有边，不重复创建。

闭环建立后，活动游标不是停在最初闭环点，而是重新投影到确认时所在的旧边位置，然后以
`tracking_kind=loop` 继续沿旧图移动。理论上后续重复绕圈不再生成节点。

## 10. 离开旧路、迟滞与拆边

旧路跟踪使用比进入更宽的阈值，形成迟滞：

| 类型 | 进入 XY | 退出 XY | 进入确认 | 退出确认 |
| --- | ---: | ---: | ---: | ---: |
| 相邻原路退回 | 0.30 m | 0.45 m | 0.30 m | 0.30 m |
| 全局重入/闭环后跟踪 | 0.40 m | 0.55 m | 0.80 m | 0.80 m |

跟踪过程中：

- 匹配当前边成功：更新边 ID 和投影比例，不增加节点。
- 超过边端点：尝试从该端点相邻边中选择连续旧边。
- 没有匹配边：进入 `EXIT_PENDING`，缓存首次分离以来的位姿。
- 在确认距离内回到同一旧图位置：取消退出，整段距离继续计入抑制距离。
- 持续离开达到退出确认距离：从真实离开锚点开始创建新路线。

确认离开时：

- 锚点在旧顶点吸附范围内：复用顶点。
- 锚点位于旧边中部：删除旧边，插入 junction 节点，再创建两条 `edge_split` 边。
- 从锚点开始重放缓存位姿，避免丢失迟滞区间。

拆边示例：

```text
拆分前：8 -------- 9       old edge id = 8

拆分后：8 ---- 10 ---- 9   new edge ids = 9, 10
                 |
                 +---- 新分支
```

旧 edge 8 被删除，节点 8、9 保持原 ID，新 junction 使用下一个稳定节点 ID 10。

## 11. 拐点处理

拐点检测器维护一个局部位姿窗口：

- 相对窗口起点 XY 半径不超过 `0.50 m`。
- 窗口内 Z 范围不超过 `0.10 m`。
- yaw 展开后的范围超过 `45°`。
- 持续时间至少 `0.50 s`。

窗口结束时取时间中点附近位姿作为候选：

- 距已有节点不超过 `0.35 m`：直接标记已有节点。
- 否则尝试投影到不超过 `0.35 m` 的已有边，必要时拆边插入拐点。
- 同一点多次检测只保留较大的 `turnDeg` 及其姿态。

在线线性预览还使用 `corner_min_separation=1.0 m` 合并相邻拐点；智能图核心主要依赖空间
合并与边投影，不按数字 ID 假定相邻关系。

## 12. 坡点处理

智能图不能再按 ID 序列执行坡度分析，因为有分支、拆边和闭环。当前做法是：

1. 在度数不等于 2 的端点/交汇点处分割图。
2. 得到端点或交汇点之间的最大无分支链。
3. 纯环形且所有节点度数都为 2 时，把整个环作为一条闭链处理。
4. 每条链独立运行现有局部线性 `Z/XY distance` 坡度算法。
5. 交汇节点只要在任一相邻链被判为坡点，就执行 OR 保留坡点标志。

默认坡度参数：拟合半径 `2.0 m`、坡度阈值 `0.12`、核心段最短 `2.0 m`、最小高度变化
`0.20 m`、最大核心间隔 `1.0 m`、前后缓冲 `1.0 m`。

## 13. RViz 实时显示

在线节点每 `0.20 s` 检查并发布变化：

| 显示 | 含义 |
| --- | --- |
| 灰色路径 | 机器人完整真实轨迹，包括重复走过和退回部分。 |
| 绿色细边 | 智能图中的 discovery 与 edge_split 边。 |
| 橙色粗边 | 已确认的 `loop_closure` 边。 |
| 青色粗边 | 当前候选或正在跟踪的旧边。 |
| 蓝色节点 | 普通智能拓扑节点。 |
| 黄色节点 | 坡点。 |
| 红色节点 | 已确认拐点。 |
| 橙色球 | 尚未结束窗口的拐点候选。 |
| 紫色球 | 定位跳变位置。 |

节点标签使用智能图稳定 ID。前缀 `C` 表示拐点、`S` 表示坡点、`SC` 表示同时满足。

灰色轨迹出现两圈不代表智能图一定有两圈；是否真的生成了重复拓扑，必须查看绿色/橙色边
或直接检查 `topoGraph_live.json` 的边表。

## 14. 按键生命周期与最终保存

### 按 `1`

1. 在 `/tmp` 创建 `route3d_live_*` 临时目录。
2. 启动在线骨架节点。
3. 等待 DDS 发现约 `1.5 s`。
4. 启动同步记录器。
5. 实时写入 PCD、事件和两个 live JSON。

### 按 `2`

1. 先向记录器发送 SIGINT，等待 `pose.json` 和 PCD 完整收尾。
2. 再停止在线骨架节点，使它执行 `finalize()`。
3. 保留 `pose_realtime.json`。
4. 运行旧 Nav2 Route3D 图构建与线性目标生成流程。
5. 使用 `pose.json` 离线重放同一个智能拓扑核心，输出 `topoGraph_data.json`。
6. 从智能图转换 `selected_route3d_graph.json`，保留任意端点连接。
7. 写入在线/离线结构哈希和统计摘要。
8. 输入未使用的目录名后，将完整临时会话移动到 `data/<名称>`。

最终主要产物：

| 文件 | 用途 |
| --- | --- |
| `pose.json` | 收尾后的完整同步原始位姿。 |
| `pose_realtime.json` | 当前版本与 pose.json 相同，为未来优化轨迹保留原始副本。 |
| `key_frames/*.pcd` | 与 pose 行序号一一对应的关键帧点云。 |
| `events.jsonl` | 节点、退回、重入、闭环、拆边和跳变事件。 |
| `live/topoSingle_live.json` | 在线旧线性预览。 |
| `live/topoGraph_live.json` | 在线智能图结果。 |
| `route3d_graph.json` | 旧 Nav2 Route3D 原始图。 |
| `topoSingle_data.json` | 旧线性控制器使用的最终文件。 |
| `topoGraph_data.json` | 离线重放得到的智能拓扑权威文件。 |
| `selected_route3d_graph.json` | 保留智能图边关系的最终 RViz 检查图。 |
| `processing_summary.json` | 在线/离线节点数、边数、统计与结构哈希一致性。 |

### 按 `3`、`q` 与 Ctrl-C

- 按 `3`：正常停止记录器、骨架节点和 RViz，不执行最终离线处理；临时数据保留在 `/tmp`。
- 记录中按 `q`：只提示应使用 `2` 或 `3`，不会直接杀死子节点。
- 空闲时按 `q`：退出。
- Ctrl-C/SIGTERM/SIGHUP：统一进入安全清理，依次 SIGINT、SIGTERM、必要时 SIGKILL，并回收
  子进程；未命名临时会话保留。

## 15. JSON 结构

简化示例：

```json
{
  "frame_id": "camera_init",
  "status": "recording",
  "topology_mode": "incremental_graph",
  "vertices": {
    "10": {
      "pos": [-0.067, -0.632, -0.612],
      "rpy": [0.0, 0.0, 0.0],
      "meta": {
        "isCorner": false,
        "isSlope": false,
        "isJunction": true,
        "sourceStamp": 0.0,
        "component": 0
      }
    }
  },
  "edges": {
    "11": {
      "v": [10, 2],
      "weight": 1.06,
      "meta": {"dir": 0, "source": "loop_closure"}
    }
  },
  "generation": {
    "retrace_count": 1,
    "route_reentry_count": 1,
    "loop_closure_count": 1,
    "loop_validation_available_count": 1,
    "loop_validation_rejection_count": 0,
    "suppressed_distance_m": 10.37,
    "edge_split_count": 1,
    "component_count": 1
  }
}
```

`topology_mode=incremental_forest` 表示尚未创建闭环边；只要成功创建过闭环边就变为
`incremental_graph`。它不表示图一定没有错误或重复小环。

## 16. 事件语义

| 事件 | 含义 |
| --- | --- |
| `session_started` / `session_stopped` | 在线会话开始/结束。 |
| `vertex_added` | 创建稳定 ID 节点。 |
| `retrace_enter` | 相邻旧边原路退回确认成功。 |
| `route_reentry_candidate` | 发现图距离较远的旧边候选。 |
| `route_reentry_rejected` | PCD 验证拒绝全局重入。 |
| `route_reentry_enter` | PCD/几何确认通过，开始沿全局旧路跟踪。 |
| `loop_closure` | 创建或复用了闭环连接。 |
| `retrace_exit` | 持续离开旧路，从锚点恢复探索。 |
| `edge_split` | 旧边中部插入 junction 并拆成两条边。 |
| `pose_jump` | 相邻原始位姿超过定位跳变阈值。 |
| `frame_changed` | 位姿 frame 在同一会话中变化，该消息被丢弃。 |

## 17. 当前实机例子：为什么最终是 `10–2`，不是 `10–1`

远端会话 `/tmp/route3d_live_yr2s8uxl` 的实测过程为：

1. 首圈按探索顺序生成 `1–2–3–4–5–6–7–8–9`。
2. 机器人越过起点附近后又退回，先匹配到末端相邻的旧边 `8–9`。
3. 从该退回边离开时，在接近起点的位置把 `8–9` 拆成 `8–10–9`。
4. `retrace` 退出需要累计 `0.30 m`，在这段迟滞完成之前不会启动全局重入搜索。
5. 等状态回到 `DISCOVERING` 并首次发现全局候选时，机器人已经位于 `1–2` 边的中后段，
   该候选投影距离节点 2 小于 `loop_closure.vertex_snap_distance=0.45 m`。
6. 系统缓存的首次闭环锚点因此是节点 2，而不是节点 1。
7. 继续匹配 `0.948 m` 并通过 PCD 验证，重合率约 `0.809`。
8. 最终创建 `edge 11: [10, 2]`。

当前代码确实缓存候选首次出现时的投影；问题并非“确认完成时才选择闭环点”，而是上一段
旧路退出迟滞让全局候选开始得太晚，再叠加 `0.45 m` 端点吸附范围，使首次候选直接吸附
到了节点 2。

## 18. 当前已知限制与后续修改重点

### 18.1 回到起点停住不会立即闭环

全局重入统一要求继续匹配 `0.80 m`。机器人完整绕圈回到起点后停止时，虽然可能已经产生
`route_reentry_candidate`，但不会进行 PCD 验证或创建闭环。需要新增“远距离图路径的端点
闭环”规则：回到旧顶点附近且 PCD 通过时，可直接闭合，不要求继续沿旧边行驶。

### 18.2 相邻退回退出迟滞可能让闭环锚点后移

当前四态状态机一次只处理一种跟踪关系。在从末端退回边切换到全局旧边时，必须先完成
`retrace` 的 `0.30 m` 退出，再开始全局候选搜索。这会错过真实交汇点，实测中导致
`10–1` 变成 `10–2`。

后续应允许在 `RETRACING/EXIT_PENDING` 中检测“当前位置同时接近另一条非相邻旧边”，并把
首次交叉位置缓存为闭环锚点，而不是等待退出完成。

### 18.3 闭环后仍可能退出并长出重复小环

当前跟踪以“当前单条边及其相邻边”为主。圆环两圈存在 `0.4–0.7 m` 横向偏差或在拐角处
方向变化较大时，可能超过 `0.55 m/45°`，持续 `0.80 m` 后退出旧图并创建平行新路线；
随后又闭环，最终得到多个小环。

下一步应升级为连续旧折线的地图匹配，结合一段时间内的几何代价、拓扑连续性和 PCD
一致性维持吸附，而不是只判断当前单边。对于两端都回到同一路线的窄平行链，还需要增加
重复链折叠或图后处理。

### 18.4 PCD 只是局部验证，不会修正轨迹

体素验证只回答“两个局部关键帧是否足够相似”，不会优化位姿、把新轨迹拉回旧中心线，
也不会证明两条边之间整段可通行。高重合率不能自动解决重复平行边。

### 18.5 `required=false` 允许无 PCD 闭环

PCD 存在且重合率不足会拒绝；但 PCD 不可用时默认允许几何闭环。实机生产环境若要求每个
闭环都有点云证据，应设为 `required=true`，同时先保证关键帧写盘和读取时序可靠。

### 18.6 当前图不应直接用于控制

`topoGraph_data.json` 已具备最短路径所需的任意端点图结构，但在以上骨架问题解决前，错误
小环或错误闭环端点会直接影响路径选择。建议先稳定骨架，再新增独立的
`route3d_graph_planner`：把当前位姿和目标投影到最近有效边/顶点，必要时在内存中临时拆边，
再用 Dijkstra/A* 输出节点序列与 `nav_msgs/Path`。

## 19. 参数总表与调参影响

### 19.1 基础与原路退回

| 参数 | 当前值 | 增大后的主要影响 |
| --- | ---: | --- |
| `target_spacing` | 1.00 m | 节点更稀疏。 |
| `dedup_distance` | 0.05 m | 更强地过滤低速移动和抖动。 |
| `relocation_distance` | 0.50 m | 更不容易判定定位跳变，但可能误连真实跳变。 |
| `retrace.corridor_xy_tolerance` | 0.30 m | 更容易进入相邻旧边退回，也更容易误吸附。 |
| `retrace.corridor_exit_xy_tolerance` | 0.45 m | 退回跟踪更不容易退出。 |
| `retrace.z_tolerance` | 0.20 m | 更容易把不同高度路线误认为同一路线。 |
| `retrace.heading_tolerance_degrees` | 35° | 更能容忍转弯，同时增大交叉口误匹配风险。 |
| `retrace.entry_confirmation_distance` | 0.30 m | 更稳定但进入更慢。 |
| `retrace.exit_confirmation_distance` | 0.30 m | 更稳定但真实分支锚点确认更晚。 |
| `retrace.vertex_snap_distance` | 0.30 m | 更容易复用端点，减少拆边但可能吸到错误端点。 |

### 19.2 闭环与 PCD

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `loop_closure.corridor_xy_tolerance` | 0.40 m | 全局候选进入横向范围。 |
| `loop_closure.corridor_exit_xy_tolerance` | 0.55 m | 闭环后持续跟踪横向范围。 |
| `loop_closure.z_tolerance` | 0.20 m | 防止上下层 XY 重合误闭环。 |
| `loop_closure.heading_tolerance_degrees` | 45° | 全局候选及跟踪的无向角度范围。 |
| `loop_closure.confirmation_distance` | 0.80 m | 候选创建闭环前的连续匹配距离。 |
| `loop_closure.exit_confirmation_distance` | 0.80 m | 闭环跟踪持续偏离后确认新分支的距离。 |
| `loop_closure.minimum_graph_separation` | 3.00 m | 排除当前图位置附近的边，避免局部重复连接。 |
| `loop_closure.minimum_time_separation` | 5.00 s | PCD 历史参考帧时间间隔。 |
| `loop_closure.vertex_snap_distance` | 0.45 m | 候选投影复用旧端点的范围。 |
| `loop_closure.rejection_cooldown_distance` | 1.00 m | PCD 拒绝后重新搜索前需移动的距离。 |
| `pcd_validation.voxel_size` | 0.30 m | 体素尺寸；越大越宽容、细节越少。 |
| `pcd_validation.minimum_overlap` | 0.45 | 最小体素重合率。 |
| `pcd_validation.search_radius` | 0.60 m | XY 离散平移搜索范围。 |
| `pcd_validation.z_search_radius` | 0.30 m | Z 离散平移搜索范围。 |
| `pcd_validation.minimum_voxels` | 100 | 两帧参与比较的最小有效体素数。 |

只调大走廊阈值不能根治当前问题：它可能减少重复边，但同时会增加平行路线、交叉路线和
不同通道被误合并的概率。更合理的方向是连续折线地图匹配、端点闭环特判和重复链折叠。

## 20. 在线与离线一致性

在线模式给每个同步位姿附加递增的 `frame_index`；离线模式按 `pose.json` 行号恢复相同索引，
并使用同一个核心和同一组参数重放。`processing_summary.json` 对比：

- 节点数和边数。
- 拐点、坡点和交汇点数量。
- retrace、route reentry、loop closure 和 PCD 拒绝次数。
- 抑制距离和拆边次数。
- 节点/边关键结构的 SHA-256。

在线可能受 DDS 启动发现或消息丢失影响，最终以离线 `topoGraph_data.json` 为准。一致性
不代表算法语义一定正确，只代表同一输入下在线与离线实现没有漂移。

## 21. 推荐验收场景

修改状态机后至少复测：

1. `1→2→3→4→5→4→3→2→1→6`，只保留旧链和 `1–6` 新分支。
2. 绕一圈回到起点后立即停止，直接产生正确端点闭环。
3. 绕一圈越过起点再走 1 m，然后退回，闭环锚点仍是首次真实交汇位置。
4. 同向重复绕两圈且横向偏差约 `0.5–0.7 m`，最终只保留一个环。
5. 从旧边中部离开，正确拆边并生成三条有效边。
6. 靠近非相邻平行路线或交叉路线，PCD/拓扑连续性不足时不误闭环。
7. 上下层 XY 重合但 Z 差超过 `0.20 m`，不判定为同一路线。
8. 定位跳变产生新 component，跳变两端不连边。
9. 同一 pose/PCD 序列在线与离线结构哈希一致。

