# route3d_web_console

Route3D 浏览器控制台。本版本把 **PCD 地图、定位配置、拓扑 JSON** 三者解耦，并把本地拓扑编辑器与顶点定位初始化整合到网页。

## 主要功能

- 扫描 `maps.allowed_roots` 下的本地 `.pcd` 文件并独立加载可视化。
- 扫描 `maps.allowed_roots` 下所有合法 `route3d_topology` v2 JSON，并独立于 PCD 选择。
- 点击顶点/边后持续高亮。
- 顶点属性、边属性可在网页修改。
- `保存拓扑` 直接写回当前选中的本地 JSON，`TopologyStore` 会在同目录 `.route3d_web_backups/` 自动保存旧版本。
- `连接边` 模式下依次点击两个顶点创建边；第一个顶点绿色高亮，完成后新边自动选中。
- 选中顶点后点击 `使用此点发送定位初值`，把当前网页选择的 topology 路径和 vertex id 发给 `route3d_vertex_initializer`。
- 如果 initializer 未运行，Web Console 会直接按顶点 `pos + rpy` 生成 `/initialpose` 作为后备。
- 控制链启动只读取当前选中的 topology JSON；定位启动只读取当前选中的 PCD + 定位配置模板。

## 资源关系

PCD 与拓扑不再绑定：

```text
本地 PCD ----------------------> 点云可视化 / 定位 global_map_path
定位配置模板 ------------------> 定位程序其他参数
本地 topoGraph_data.json ------> 拓扑编辑 / Dijkstra / RouteSlicer / 控制链
                                   |
                                   +--> 顶点定位初值
```

## 启动

同时构建 `route3d_web_console` 和 `route3d_vertex_initializer` 后：

```bash
ros2 launch route3d_web_console route3d_web_console.launch.py
```

默认会同时启动 `route3d_vertex_initializer`。如需关闭：

```bash
ros2 launch route3d_web_console route3d_web_console.launch.py launch_vertex_initializer:=false
```

浏览器访问：

```text
http://机器人IP:8080
```

## 本地文件扫描

配置：`config/web_console.yaml`

```yaml
maps.allowed_roots:
  - /home/langyi/workspace/map
  - /home/langyi/workspace/wyf/topo_graph_ws/data
resources.scan_max_files: 1000
resources.scan_max_depth: 6
```

只有在允许目录下的文件可以由网页读取和修改。

## 保存拓扑

为避免“磁盘文件已经改变、正在运行的 planner 仍持有旧图”的不一致，运行 planner 时仍禁止保存 topology。停止 planner 后点击保存即可直接覆盖原 JSON，并自动备份旧文件。

## Local map/topology discovery

The current UI does not require a control lease. Local resources are deliberately independent:

- PCD maps are discovered only as `/home/langyi/workspace/map/<name>/map/<name>.pcd` (or the only `.pcd` in that `map/` directory as a compatibility fallback). The selector shows `<name>`.
- Topology maps are discovered only as `/home/langyi/workspace/wyf/topo_graph_ws/data/<name>/topoGraph_data.json`. The selector shows `<name>` and ignores `events.jsonl`, `route3d_graph.json`, `topoSingle_data.json`, etc.
- The PCD voxel/downsampling size is editable directly in the page and only affects the browser static-map visualization.

## 运行状态探测

网页每 500 ms 同时检查由 Web Console 管理的进程状态以及 ROS 实际通信状态。即使功能是从 SSH/其它终端启动，页面也能显示 ROS 在线状态：

- 雷达：`topics.raw_cloud` 是否有 publisher。
- 建图：`/save_pcd_service`、`topics.mapping_odometry`、`topics.mapping_cloud`。
- 定位：`topics.odometry` 是否有 publisher。
- 规控：Route3D planner/slicer/controller/adapter 节点是否存在。
- 自动打点：在线 skeleton / recorder 节点是否存在。
- 顶点初始化：initializer 节点或 vertex-id topic subscriber 是否存在。

## 建图和保存地图

网页的“开始建图”执行：

```bash
cd /opt/mapping_ws && ./run_mapping_nodes.sh mode:=mapping
```

“关闭建图节点”停止由网页启动的建图进程。自动打点和建图互不排斥；自动打点允许使用定位 `/lio_odom_hf` 或建图 `/lio_odom` 中任意一路新鲜里程计。

保存地图时页面只输入一个名称，例如 `510`。固定分辨率标签由 YAML 参数控制：

```yaml
mapping.save_resolution_tag: "0.1"
```

输入 `510` 后会调用等价于：

```bash
mkdir -p /home/langyi/workspace/map/510/map
ros2 service call /save_pcd_service moveit_msgs/srv/SaveMap \
  "{filename: '/home/langyi/workspace/map/510/map/510-0.1'}"
```

服务成功且生成 `510-0.1.pcd` 后，自动创建/更新：

```text
/home/langyi/workspace/map/510/map/510.pcd -> 510-0.1.pcd
```

因此保存完成后刷新资源列表即可直接按目录名 `510` 选择该地图。

## 现场版雷达 / 定位 / 建图约定

- 雷达由 `driver.service` 统一管理。网页启动/停止按钮分别执行 `sudo -n systemctl restart driver.service` 与 `sudo -n systemctl stop driver.service`，不再启动第二套驱动。
- 网页节点继承启动终端的 `ROS_DOMAIN_ID`（默认 0），用于和 systemd 雷达服务及远程终端处于同一个 ROS 图。
- 定位无需手工选择定位模板。选择 PCD 后直接启动定位；后台自动匹配 catalog 中的旧模板，匹配不到时使用 `localization.config_template`。
- 实时点云订阅 `/cloud_registered_body`，并使用 `/lio_odom_hf`（建图时回退 `/lio_odom`）将 body-frame 点云变换到全局位置后显示。
- 建图启动目录为 `/opt/mapping_ws`：`cd /opt/mapping_ws && ./run_mapping_nodes.sh mode:=mapping`。

## 2026-09 UI/module and goal-only updates

- The left workflow is now a module selector. Radar, mapping, localization, topology recording,
  topology editing and navigation expose their current status in the selector; only the selected
  module's detailed controls are expanded.
- Real-time cloud streaming is opt-in. `/cloud_registered_body` (or raw fallback) is converted and
  sent only to browser sessions that have checked `实时点云（按需传输）`. If no browser requests the
  stream, the backend skips cloud conversion/encoding entirely.
- Goal-only navigation matches `sh/06_send_goal_only.sh`: the web console publishes an Int32 vertex
  id to `/route3d_dijkstra/goal_request`, letting Dijkstra choose the nearest current topology
  vertex as the start. The command is available both in the navigation module and from the selected
  vertex inspector (`导航到此点`).
