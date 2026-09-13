# M20 Route Slicer 接口

切片器把 Dijkstra 路径按以下属性边界拆成 RouteTask：控制器、障碍模式、速度、方向、
高度补偿、外部栅格和必须停车的姿态对准点。

控制器解析优先级：

1. `obstacleMode=1`：Efficient 3D。
2. `obstacleMode=2`：PID。
3. `obstacleMode=4`：外部栅格。
4. 其余模式服从显式 `controllerMode`，`auto` 为 PID。

坡点不会修改上述结果。RouteTask 不含运动模式请求或确认字段，启动、暂停恢复和停障恢复
不会等待额外确认。

最终目标总是需要 yaw 对准。中间点只有 `alignFinalYaw=true` 才形成停车边界；非 PID 路段
结束在需要对准的顶点时，切片器追加单点 PID 对准任务。

拐点通过 `passRadiusM=0.20 m`、提前 0.70 m 降速和 0.20 m/s 拐点速度防止切弯。
