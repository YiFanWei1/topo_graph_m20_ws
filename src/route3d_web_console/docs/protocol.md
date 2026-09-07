# Route3D WebSocket protocol v1

连接地址为 `/ws`。控制消息和状态事件是 UTF-8 JSON 文本帧；点云是小端序二进制帧。

## 文本消息

客户端命令必须包含 `type` 与唯一 `request_id`。服务器用以下响应确认：

```json
{"type":"ack","request_id":"...","success":true,"message":"..."}
```

支持的命令：

- `map.list`
- `map.select {id}`：选择定位 PCD；一个地图条目可绑定多张拓扑图。
- `topology.list`
- `topology.select {id}`：选择当前 PCD 下的规划拓扑。
- `map.upsert {map}`
- `topology.load`
- `topology.save {data, revision}`
- `topology.record.start`：等价于原键盘流程按 `1`。
- `topology.record.stop {name}`：等价于按 `2` 并输入保存目录名。
- `topology.record.abort`：等价于按 `3`，不生成正式结果。
- `control.claim`、`control.heartbeat`、`control.release`
- `process.start {target, enable_motion}`、`process.stop {target}`
- `localization.set_initial_pose {position:[x,y,z], yaw}`
- `navigation.plan {start_id, goal_id}`
- `navigation.pause`、`navigation.resume`、`navigation.cancel`

服务器事件包括 `snapshot`、`pose`、`velocity`、`ros.status`、`process.log`、
`map.catalog`、`topology.catalog`、`topology.snapshot`、`topology.preview`、
`topology.record.saved`、`topology.saved`、`map.cloud_ready` 和 `control.expired`。

## 点云二进制帧

所有整数和浮点数均为小端序。固定头为 60 字节：

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 | char[4] | `R3PC` |
| 4 | uint8 | 协议版本，当前为 1 |
| 5 | uint8 | 流：1 静态 PCD，2 注册点云，3 原始雷达 |
| 6 | uint16 | flags，bit0 表示带 intensity |
| 8 | uint32 | 帧序号 |
| 12 | uint64 | ROS 时间戳 ns |
| 20 | uint32 | 本分片点数 |
| 24 | uint16 | 单点步长，12 或 16 字节 |
| 26 | uint16 | frame_id UTF-8 字节数 |
| 28 | uint32 | 分片序号，从 0 开始 |
| 32 | uint32 | 分片总数 |
| 36 | float32[6] | min xyz、max xyz |
| 60 | byte[] | frame_id，随后为交错 float32 xyz/xyzi |

实时流对每个客户端只保留最新帧。静态地图按分片排队，单客户端待发送二进制数据上限
为 12 MiB，超过后丢弃最旧分片。
