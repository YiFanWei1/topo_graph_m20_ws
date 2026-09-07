# Route3D Web Console

独立的 ROS 2 网页控制台，用浏览器完成雷达、已有 PCD 定位、初始位姿、拓扑编辑和规划控制。
本包只调用现有系统接口，不修改其他 Route3D 功能包。

## 构建与启动

```bash
cd /home/langyi/workspace/wyf/topo_graph_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select route3d_web_console
source install/setup.bash
ros2 launch route3d_web_console route3d_web_console.launch.py \
  ros_domain_id:=42 \
  port:=8080
```

浏览器访问 `http://<机器人电脑IP>:8080`。首次使用前编辑本包安装前的
`config/map_catalog.json`，或在运行时目录
`~/.ros/route3d_web_console/map_catalog.json` 中登记 PCD、定位配置和多张拓扑图的绑定。
网页的自动打点按钮复用 `route3d_product_demo live_route_product.launch.py`：开始/停止分别
模拟原键盘流程的 `1` 与 `2 + 保存名`，最终文件生成后自动加入当前 PCD 的拓扑列表。

Launch 会让网页节点及其子进程使用同一个 `ROS_DOMAIN_ID` 和 RMW。默认值与远端原有环境
一致：Domain 42、`rmw_cyclonedds_cpp`、配置文件
`/opt/mapping_ws/cyclonedds_remote.xml`。如需覆盖可传入 `rmw_implementation`、
`cyclonedds_uri` 和 `cyclonedds_config_file`。如果 `8080` 已被其他网页占用，可将 `port`
改为 `18080` 等空闲端口。

## 安全顺序

1. 启动雷达。
2. 选择地图并启动定位。
3. 在点云上拖动设置初始位姿，确认实时位置稳定。
4. 编辑并保存拓扑；保存期间规划器必须停止。
5. 先以关闭实机运动的方式启动控制链进行验证。
6. 遥控器急停就绪后，才允许勾选实机运动并重新启动控制链。

浏览器必须先取得控制租约才能执行写操作。控制租约心跳丢失超过三秒时，后台调用
`/route3d_pid_controller/cancel`。

## 配置注意事项

- `process.*_command` 是由后台启动的 shell 命令，运行环境会先 source ROS 和工作空间。
- 网页管理的 CycloneDDS 进程使用包内 `cyclonedds_web.xml`，同时启用 `lo` 与
  `enp2s0`：前者承载本机大点云，后者使 Fast DDS 的 Go2 SDK 适配器能与控制节点互通。
  该配置不修改 `/opt/mapping_ws/cyclonedds_remote.xml`。
- 默认定位命令还会加载 `/opt/mapping_ws/install/setup.bash`，并将
  `/opt/mapping_ws/glio_mapping` 加入 `LD_LIBRARY_PATH`，供定位程序查找专用消息和动态库。
- 网页启动定位时会像原 `run_mapping_nodes.sh` 一样，同时发布 `map -> camera_init`
  静态 TF；停止定位时该 TF 子进程会一并退出。
- 定位启动时，后台复制所选定位 YAML 到运行时目录，仅替换 `global_map_path`。
- 网页节点会轻量订阅 `/global_map`；这是 GLIO 无 RViz 启动时完成初始化所必需的，
  地图在浏览器中的显示仍使用经过降采样的 PCD 缓存，不会把完整全局地图反复转发到网页。
- 实时显示优先使用全局坐标系中的 `/cloud_registered`；仅在它超过配置时间没有数据时
  才回退到雷达坐标系中的 `/livox/lidar`，避免两种坐标系的点云交替跳变。
- `maps.allowed_roots` 限制网页可访问的 PCD 和拓扑路径。
- 发布 `/initialpose` 的消息为 `geometry_msgs/msg/PoseWithCovarianceStamped`。
- `/initialpose` 默认使用原 RViz 流程要求的 `map` 坐标系；它与 PCD/拓扑使用的
  `camera_init` 显示坐标系相互独立。
- “初始位姿”工具与 RViz 的 2D Pose Estimate 一样在 `map` 的 XY 平面取点，发布
  高度固定为 `localization.initial_pose_z`（默认 `0.0`），不会误选墙面或楼板高度。
- 规划器只在启动时加载图，因此保存拓扑后需要重新启动控制链。
- `web/` 已包含离线构建产物，远端运行不需要 npm。修改前端源码时进入 `web_src/`
  执行 `npm ci && npm run build`。

详细帧格式见 [protocol.md](docs/protocol.md)。
