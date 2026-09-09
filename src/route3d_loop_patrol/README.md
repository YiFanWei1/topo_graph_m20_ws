# route3d_loop_patrol

在两个拓扑顶点之间往返巡检。节点依次核对 Dijkstra 起终点、切片任务的
`route_sequence/start/goal`，并且只在收到同一 `route_sequence` 对应的
`/route3d_pid_controller/status` 明确 `state=FINISHED` 且 `active=false`
之后，才会发送反向的下一条 Dijkstra 请求。

## 配置

编辑 `config/loop_patrol.yaml`：

- `patrol.start_vertex_id`、`patrol.goal_vertex_id`：往返端点。
- `patrol.request_mode`：`start_goal` 使用原有双 ID 请求；`goal_only` 每程只发送终点，
  由规划器根据当前位置重新匹配起点。
- `patrol.dwell_time_s`：明确到达后，在端点停留的秒数。
- `patrol.max_round_trips`：`0` 表示无限循环。
- `patrol.auto_start`：启动节点后是否自动开始。
- `patrol.wait_for_idle_before_first_request`：默认等待当前路线明确结束，避免启动循环节点时覆盖正在执行的路线。
- `timeouts.arrival_s`：建议保持 `0`；启用超时后也只会进入错误状态，绝不会跳到下一段。

## 使用

先启动正常的 `go2_pid_route.launch.xml`，再启动：

```bash
source install/setup.bash
ros2 launch route3d_loop_patrol loop_patrol.launch.py
```

查看状态：

```bash
ros2 topic echo /route3d_loop_patrol/status
```

手动启动或停止循环：

```bash
ros2 service call /route3d_loop_patrol/start std_srvs/srv/Trigger '{}'
ros2 service call /route3d_loop_patrol/stop std_srvs/srv/Trigger '{}'
```

`stop` 只停止后续循环请求，不会取消机器人正在执行的当前路线。
