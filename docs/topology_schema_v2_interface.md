# Route3D M20 拓扑语义

顶点至少包含 `pos`、`rpy`、`meta`、`alignFinalYaw`、`passRadiusM`。新顶点默认
`alignFinalYaw=false`；最终导航目标仍无条件使用 `rpy.yaw` 对准。

边至少包含端点 `v`、`weight`、`rotationAllowed` 和 `meta`。M20 新边默认值：

```json
{
  "dir": 0,
  "travelMode": "bidirectional",
  "controllerMode": "auto",
  "linearSpeedMps": 1.0,
  "angularSpeedRadps": 0.0,
  "heightOffsetM": 0.0,
  "obstacleMode": 1,
  "headingAngleRad": 0.0,
  "obstacleBoxM": [0.0, 0.0, 0.0, 0.0],
  "gridMapName": ""
}
```

模式表：

| obstacleMode | 控制器 | 普通点云停障 |
|---:|---|---|
| 0 | `controllerMode`，auto 为 PID | PID 开启 |
| 1 | 强制 Efficient 3D | 由 Efficient 绕障 |
| 2 | 强制 PID | 关闭 |
| 3 | 服从 `controllerMode` | 关闭 |
| 4 | 外部栅格 | 关闭，要求 `gridMapName` |

`isSlope` 不参与控制器选择。新图不保存 `locomotionMode`；旧图中该字段可被读取但被忽略。
所有模式都保留全局急停、碰撞等级以及里程计/点云超时保护。
