# M20 Route3D 快捷脚本

脚本通过自身位置解析工作空间，不依赖机器的绝对路径。执行前需先完成编译。

## 自动打点

```bash
./sh/01_start_auto_waypoint.sh
```

可选的第一个参数是产品 YAML；默认使用 M20 的 0.57 m 高度和模式 1 新边。

## 启动规控

```bash
./sh/02_start_planning_control.sh \
  ./data/79792/topoGraph_data.json false true
```

参数依次是拓扑 JSON、`enable_motion`、是否启动 RViz。`enable_motion`默认为
`false`，未显式修改时 M20 adapter 持续输出零速度。

## 发送路线或单目标

```bash
./sh/04_send_goal.sh 1 11
./sh/06_send_goal_only.sh 11
```

单目标模式要求当前位置在某拓扑点的匹配半径内。

## 循环巡航

先修改 `sh/loop_patrol.yaml` 或 `sh/goal_only_loop_patrol.yaml` 中的端点，再执行：

```bash
./sh/05_start_loop_patrol.sh
./sh/07_start_goal_only_loop.sh
```

循环节点只在收到当前路线明确的 `FINISHED` 后才发送反向路线。

## 停止

```bash
./sh/03_stop_all_ros.sh
```

脚本会先请求停止循环和当前运动，再关闭当前用户启动的 ROS 2 进程。

## Bag 回放

Bag 模式使用独立入口，固定禁用实机 bridge 和运动输出：

```bash
./sh/08_start_bag_planning_control.sh \
  ./data/m20_route_001/topoGraph_data.json true 0.57
./sh/09_play_sensor_bag.sh /home/wei/bag/m20_route_001 1.0 false
```

Bag 回放会过滤录制的 `/tf`，并由规控栈从 `/lio_odom_hf` 重新发布
`camera_init -> base_link`，防止 `base_link` 固定在原点或 `body` 出现两个父坐标系。

第三个参数 `0.57` 是 M20 原生 Bag 的录制机身高度。历史 Go2 Bag 才使用 `0.40`；M20
原生 Bag 不做 `+0.17 m` 补偿。完整的实机和仿真步骤见根目录
[`M20_RUNBOOK.md`](../M20_RUNBOOK.md)。

## M20 独立手动速度

```bash
./sh/10_start_m20_manual_velocity.sh true
```

参数为 `true` 时才允许实机运动；省略时默认禁止运动。脚本会拒绝与已经运行的 Route3D
adapter 或 `basic_server_bridge` 重复启动。速度输入为 `/m20/manual/cmd_vel`，完整说明见
[`m20_velocity_control`](../src/m20_velocity_control/README.md)。

## Web 控制台（18088）

```bash
cd /home/langyi/workspace/wyf/topo_graph_m20_ws
./sh/11_start_web_console_18088.sh
```

浏览器访问：

```text
http://192.168.121.1:18088
```

脚本默认使用实机的 `ROS_DOMAIN_ID=10`，且只启动网页服务，不会自动启动运动控制。
网页启动规控时先保持“允许实机运动”关闭；确认定位、地图、拓扑和路径均正常后再显式开启。

## 录制 regu 同类传感器 Bag

默认保存到工作空间的 `bags/`，bag 名称自动包含当前时间：

```bash
./sh/12_record_regu_topics.sh
```

指定输出父目录：

```bash
./sh/12_record_regu_topics.sh /home/langyi/bag
```

同时指定 bag 名称：

```bash
./sh/12_record_regu_topics.sh /home/langyi/bag m20_regu_001
```

脚本按照 `/home/wei/bag/regu` 录制以下四个话题：

- `/cloud_registered_body`
- `/lio_odom`
- `/lio_odom_hf`
- `/tf`

使用单个 MCAP 文件持续录制；完成后按 `Ctrl+C` 正常停止并生成 `metadata.yaml`。
