# 远端实机测试脚本

这些脚本固定使用远端工作空间：

```text
/home/langyi/workspace/wyf/topo_graph_ws
```

在远端执行：

```bash
cd /home/langyi/workspace/wyf/topo_graph_ws
```

## 自动打点

```bash
./sh/01_start_auto_waypoint.sh
```

也可以传入另一份产品配置的绝对路径。

## 启动规控

默认加载 `data/ceshi_1/topoGraph_data.json`，使用 `enp2s0`，开启真机运动和 RViz：

```bash
./sh/02_start_planning_control.sh
```

可选参数依次为拓扑图、网卡、是否开启运动、是否启动 RViz：

```bash
./sh/02_start_planning_control.sh \
  /home/langyi/workspace/wyf/topo_graph_ws/data/ceshi_1/topoGraph_data.json \
  enp2s0 true true
```

## 发送单次目标

原有接口同时指定起点和终点：

```bash
./sh/04_send_goal.sh 1 11
```

也可以只指定终点。规划器会使用当前定位在 1 m 内的最近拓扑点作为起点；超过 1 m
则拒绝请求：

```bash
./sh/06_send_goal_only.sh 11
```

## 启动循环巡检

先编辑 `/home/langyi/workspace/wyf/topo_graph_ws/sh/loop_patrol.yaml` 中的
`start_vertex_id` 和 `goal_vertex_id`，确认规控已经启动，再执行：

```bash
./sh/05_start_loop_patrol.sh
```

循环节点只有收到当前路线明确的 `FINISHED` 信号后，才会发送反向路线。

仅终点接口的循环巡检使用独立配置和脚本，同样等待 `FINISHED`，每一程的起点均根据
当前定位重新匹配：

```bash
# 先修改 sh/goal_only_loop_patrol.yaml 中的两个端点
./sh/07_start_goal_only_loop.sh
```

## 关闭 ROS 2 节点

```bash
./sh/03_stop_all_ros.sh
```

脚本先请求停止循环和当前运动，再关闭当前用户启动的 ROS 2 进程。
