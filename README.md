# topo_graph_ws

这个工作空间把实时路线记录、离线路线处理、拐点提取和 topoSingle JSON 转换串起来。

## 数据流程

```text
map_data/
  pose.json + key_frames/*.pcd
        |
        | route_graph_builder_3d
        v
route3d_graph.json 
        |
        | route_corner_target_generator
        v
route_targets_with_corners.txt
        |
        +--> topoSingle_data.json          （控制器直接使用）
        |
        | nav2_to_topo_single（仅生成可视化子图）
        v
selected_route3d_graph.json
```

目标文件中的每个点为 `id x y z_ground NORMAL|CORNER`。`z_ground` 是地面高度；转换器匹配原始 Nav2 图中的机身位姿来恢复四元数和 RPY，因此只在“地面坐标”和“机身坐标”之间使用一次 `body_height`，默认值为 `0.40 m`。

## 编译

```bash
cd /home/wei/github_code/topo_graph_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
`nav2_util` 的上游测试依赖 `test_msgs` 没有包含在当前源码集合中，所以默认关闭测试；运行组件和本流程包均正常编译。

## 实时记录路线

先启动机器人或回放节点，再启动记录器。输出目录必须是新的目录，程序收到 Ctrl-C 后会把 `pose.json.partial` 完整收尾为 `pose.json`。

```bash
ros2 launch route3d_data_recorder record_route.launch.py \
  output_directory:=/home/wei/github_code/topo_graph_ws/data/my_route \
  odom_topic:=/lio_odom \
  cloud_topic:=/cloud_registered_body \
  sync_slop_s:=0.05
```

默认每个成功匹配的点云记录一帧，里程计和点云按 header 时间戳在 `sync_slop_s` 内近似同步。记录结果为：

```text
my_route/
├── pose.json
├── key_frames/0.pcd, 1.pcd, ...
└── manifest.json
```

记录阶段不扣机身高度：`pose.json` 的 z 是机身/传感器位姿高度，PCD 点保留在点云消息自己的 frame 中。

记录结束后构图：

```bash
ros2 run nav2_route3d route_graph_builder_3d \
  --map-path /home/wei/github_code/topo_graph_ws/data/my_route \
  --poses-file pose.json \
  --pcd-dir key_frames \
  --output-json route3d_graph.json
```

## 单独运行离线处理

```bash
cd /home/wei/github_code/topo_graph_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run route3d_bag_tools route_corner_target_generator \
  --graph data/regu/route3d_graph.json \
  --output data/regu/route_targets_with_corners.txt \
  --output-topo data/regu/topoSingle_data.json \
  --target-spacing 1.0 \
  --body-height 0.40
```

这一步已经直接生成控制器使用的 `topoSingle_data.json`。如果需要 RViz 验证，再额外生成简化的 Nav2 子图：

```bash
ros2 run topo_graph_tools nav2_to_topo_single \
  --graph data/regu/route3d_graph.json \
  --targets data/regu/route_targets_with_corners.txt \
  --output-nav2 data/regu/selected_route3d_graph.json \
  --body-height 0.40
```

输出的 topo 节点在 `meta.isCorner` 中标记拐点；`meta.sourceNodeId` 表示它匹配到的原始 Nav2 图节点。边默认按路线顺序建立，`weight` 为相邻目标点之间的三维距离。

## 一键生成并可视化

先关闭 RViz 验证完整数据链：

```bash
ros2 launch topo_graph_tools topo_conversion_visualization.launch.py \
  launch_rviz:=false
```

需要图形界面时：

```bash
ros2 launch topo_graph_tools topo_conversion_visualization.launch.py \
  launch_rviz:=true
```

默认使用 `data/regu/route3d_graph.json`。也可以替换数据集：

```bash
ros2 launch topo_graph_tools topo_conversion_visualization.launch.py \
  graph_filepath:=/home/wei/github_code/topo_graph_ws/data/0825_1/route3d_graph.json \
  target_file:=/home/wei/github_code/topo_graph_ws/data/0825_1/route_targets_with_corners.txt \
  topo_output:=/home/wei/github_code/topo_graph_ws/data/0825_1/topoSingle_data.json \
  reduced_graph_output:=/home/wei/github_code/topo_graph_ws/data/0825_1/selected_route3d_graph.json
```

RViz 中显示的是转换后的 `selected_route3d_graph.json`，并叠加目标点路径；这样可以同时确认点的数量、拐点位置、边连接关系以及高度语义。拐点提取阈值可直接传给目标点生成器，当前一键启动文件使用默认阈值。

## 产品演示：键盘分步打通完整流程

### 实机实时模式（推荐验收入口）

推荐通过产品 launch 启动。它会读取同一个 YAML，并按配置选择是否发布
`map -> camera_init` 和 `body -> base_link` 静态 TF：

```bash
cd /home/wei/github_code/topo_graph_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_product_demo live_route_product.launch.py
```

默认参数文件：

```text
src/route3d_product_demo/config/live_route_product.yaml
```

使用另一份实机配置：

```bash
ros2 launch route3d_product_demo live_route_product.launch.py \
  config_file:=/absolute/path/to/live_route_product.yaml
```

YAML 中 `static_tf.*.publish` 控制静态 TF。默认发布两条单位变换：

```yaml
static_tf:
  map_to_camera_init:
    publish: true
    parent_frame: map
    child_frame: camera_init
    translation: [0.0, 0.0, 0.0]
    rotation_rpy: [0.0, 0.0, 0.0]
  body_to_base_link:
    publish: true
    parent_frame: body
    child_frame: base_link
    translation: [-0.15, 0.0, -0.21]
    rotation_rpy: [0.0, 0.0, 0.0]
```

如果定位模块后来开始发布同名父子 TF，必须把对应 `publish` 改为 `false`，避免 TF
树出现双发布者。平移或安装姿态不是零时，应填写实测外参，不能继续使用单位变换。

仍可绕过 launch 直接运行键盘控制器：

```bash
ros2 run route3d_product_demo route3d_live_keyboard \
  --config /home/wei/github_code/topo_graph_ws/src/route3d_product_demo/config/live_route_product.yaml
```

实时模式只有两个主要按键：

```text
1  开始同步记录 pose/PCD、在线构建骨架并实时更新 RViz
2  停止订阅、完成写盘、离线复核，输入名称后保存全部结果
q  未记录时退出
```

RViz 中灰色线是实时原始轨迹，蓝色点是普通拓扑点，红色点是确认拐点，
橙色点是尚未结束的候选拐点，绿色线是拓扑边，紫色点表示检测到定位跳变。
在线数据来自记录器成功写盘后发布的 `/route3d/synchronized_pose`，因此显示过的每一帧
都能在 `pose_realtime.json` 和 `key_frames/*.pcd` 中溯源。

按 `2` 后最终目录为：

```text
data/<名称>/
├── pose.json
├── pose_realtime.json
├── key_frames/*.pcd
├── manifest.json
├── events.jsonl
├── processing_summary.json
├── live/topoSingle_live.json
├── route3d_graph.json
├── route_targets_with_corners.txt
├── selected_route3d_graph.json
└── topoSingle_data.json
```

`live/topoSingle_live.json` 用于记录实时显示结果，控制器只使用停止后复核生成的
`topoSingle_data.json`。构图器会从 `manifest.json.odom_frame` 继承坐标系；默认 RViz
Fixed Frame 是 `map`，通过可选的 `map -> camera_init` 静态 TF 显示定位轨迹。如果关闭
这条静态 TF，应把 RViz Fixed Frame 改为定位实际发布的 odom frame。

不启动 RViz 的 bag 自动化测试入口：

```bash
ros2 run route3d_product_demo route3d_live_keyboard --no-rviz
```

### 原离线三键模式

先在一个终端启动 ROS 2 环境和演示程序：

```bash
cd /home/wei/github_code/topo_graph_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run route3d_product_demo route3d_product_demo
```

程序只在终端读取单键：

```text
1  开始记录 /lio_odom 和 /cloud_registered_body，并写入临时目录
2  停止记录，完成 pose.json、key_frames/*.pcd、manifest.json
3  处理最近一次记录，输入名称后保存到 data/<名称>，并启动 RViz
q  退出并清理子进程
```

第三步会依次执行三维路线建图、等间隔采样与拐点提取、TopoSingle 转换，最终控制器直接使用：

```text
/home/wei/github_code/topo_graph_ws/data/<名称>/topoSingle_data.json
```

录 bag 测试时，终端 A 运行上面的演示程序；终端 B 播放已有 bag：

```bash
cd /home/wei/github_code/topo_graph_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play /home/wei/bag/regu --clock
```

bag 开始播放后在终端 A 按 `1`，播放完成后按 `2`、`3`，输入例如 `regu_demo`。也可以在播放前按 `1`，记录器会等待对应话题。

可通过参数覆盖默认配置：

```bash
ros2 run route3d_product_demo route3d_product_demo \
  --target-spacing 1.0 --body-height 0.40 --sync-slop 0.05
```

注意：记录阶段不会减去机身高度；`pose.json` 的 z 是机身/传感器位姿高度，`--body-height` 只在路线处理、目标点和可视化阶段使用。

## 主要源码

- `src/route3d_bag_tools/scripts/route_corner_target_generator`：按间隔取点并加入局部原地转向拐点。
- `src/route3d_data_recorder`：实时同步记录里程计和点云，生成 `map_data` 目录。
- `src/topo_graph_tools/scripts/nav2_to_topo_single`：将带 `NORMAL/CORNER` 的目标文件转换为 topoSingle，并可生成用于可视化的 Nav2 子图。
- `src/topo_graph_tools/launch/topo_conversion_visualization.launch.py`：串行执行生成、转换和 RViz 可视化。
- `src/route3d_product_demo`：用 `1/2/3/q` 键盘按键控制记录、处理、保存和可视化。
- `src/route3d_online_skeleton`：从已落盘同步位姿增量生成在线拓扑和 RViz Marker。
- `src/path_tracking_benchmark`：已纳入当前工作空间，用于后续固定路线跟踪评测。
- `data/regu/`：已复制的样例 Nav2 图、目标点和转换结果。

## 规划器的两种全局路径模式

`path_source:=topology` 使用产品生成的 TopoSingle JSON，通过点编号选择目标，并保留
`meta.isCorner` 的严格拐点到达逻辑：











































colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

自动记点
cd /home/wei/github_code/topo_graph_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_product_demo live_route_product.launch.py
```bash
ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=topology \
  route_file:=/home/wei/github_code/topo_graph_ws/data/510/topoSingle_data.json \
  use_sim_time:=false

ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=topology \
  route_file:=/home/langyi/workspace/wyf/topo_graph_ws/data/510/topoSingle_data.json \
  use_sim_time:=false


```

发布目标编号：

```bash
ros2 topic pub --once \
  /recorded_waypoint_route/goal_id \
  std_msgs/msg/Int32 \
  "{data: 231}"
```

`path_source:=plan` 不启动 JSON 加载器和路线序列器，直接使用 PCTPlanner 发布的完整
地面 `nav_msgs/msg/Path`。局部规划器会从机器人在完整路径上的投影位置截取前视段：

```bash
ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=plan \
  global_plan_topic:=/plan \
  use_sim_time:=false
```

两种模式的输入都按地面高度处理，统一增加 `body_height`（默认 0.40 m）后再进行局部
三维规划。PCT 模式没有目标编号和 `isCorner` 状态机，路径几何拐角仍会作为 Guided A*
参考线。

### `use_sim_time` 的选择

`use_sim_time` 只决定 ROS 节点从哪里取得当前时间，与使用拓扑 JSON 还是 `/plan` 无关：

- 实机实时话题：使用 `use_sim_time:=false`，节点读取计算机系统时间。
- rosbag 或仿真：使用 `use_sim_time:=true`，并确保有 `/clock`；rosbag 必须加 `--clock`。
- 只加载 JSON 看静态路线、不播放 bag：使用 `false`。
- 拓扑 JSON 配合 bag 的点云和里程计测试：使用 `true`，同时用 `--clock` 播包。

如果设置为 `true` 却没有任何节点发布 `/clock`，ROS 时间会停在 0，带时间戳的 TF、
消息过滤和超时判断可能无法正常工作。规划、地图、控制和 RViz 应保持相同的
`use_sim_time` 设置。

PCT `/plan` 的 bag 测试：

```bash
ros2 launch efficient_3d_local_planner up_and_down_demo.launch.py \
  path_source:=plan \
  global_plan_topic:=/plan \
  use_sim_time:=true
```

另一个终端播放 bag：

```bash
ros2 bag play /home/wei/bag/full_nav_replay_with_plan \
  --clock \
  --exclude-topics /tf_static
```
