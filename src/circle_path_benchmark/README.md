# Circle Path Controller Benchmark

该包绕过地图构建和 A*，只测试现有 `local_path_follower_node` 对固定圆的跟踪效果。
绿色是离线参考圆，紫色是实时里程计轨迹。每次运行保存 rosbag、`trajectory.csv`
和 `metrics.json`，其中负的径向误差表示向圆内侧切弯。

## 1. 离线生成并验证 0.8m 圆

```bash
ros2 run circle_path_benchmark generate_circle_path \
  --pcd /home/wei/510/map/510.pcd \
  --output /home/wei/github_code/topo_graph_m20_ws/data/circle_benchmark/circle_510.yaml \
  --center-x 0.0 --center-y -2.0 --path-z 0.0 \
  --radius 0.8 --spacing 0.05 --start-angle-deg 0 --direction ccw
```

默认起点为 `(0.8,-2.0,0.0)`，逆时针切向朝向 `+Y`（yaw=90°）。生成器会检查
机器人高度范围内的 PCD 点；圆周净空小于 `robot_radius=0.30m` 时拒绝生成。

## 2. 使用启动时定位作为圆心，只看路径

```bash
ros2 launch circle_path_benchmark circle_tracking_benchmark.launch.py \
  path_file:=/home/wei/github_code/topo_graph_m20_ws/data/circle_benchmark/circle_510.yaml \
  pcd_file:=/home/wei/510/map/510.pcd \
  output_root:=/home/wei/github_code/topo_graph_m20_ws/data/circle_benchmark/runs \
  enable_motion:=false
```

默认 `center_from_initial_odometry:=true`。节点收到第一帧 `/lio_odom_hf` 后，将该帧
`x/y` 永久锁定为圆心，路径 Z 使用该帧 Z 加 `path_height_offset=0.57m`，并使用该帧
yaw 确定圆周起点切向；之后机器人移动不会改变圆心。为了只测试控制器，默认关闭
PCD 净空和圆周起点距离检查，使能后控制器会从圆心直接接入固定圆路径。

RViz 使用固定 `camera_init` 下的正交俯视视角，不跟随机器人位置或姿态改变视角。
如果确实需要使用 YAML 中保存的旧圆心，显式设置
`center_from_initial_odometry:=false`。

## 3. 实机一圈测试

把机器人移动到绿色圆周起点并保持启动时的朝向，确认急停和控制适配器后运行：

```bash
ros2 launch circle_path_benchmark circle_tracking_benchmark.launch.py \
  path_file:=/home/wei/github_code/topo_graph_m20_ws/data/circle_benchmark/circle_510.yaml \
  pcd_file:=/home/wei/510/map/510.pcd \
  output_root:=/home/wei/github_code/topo_graph_m20_ws/data/circle_benchmark/runs \
  enable_motion:=true
```

完成一圈或超过 45s 后自动发布空路径停车。`enable_motion` 只控制路径是否送入控制器，
真正发往机器人仍需要单独启动 `robot_control_adapter`。如需恢复起点检查，可将
`safety.require_start_pose` 设为 true；如需恢复 PCD 检查，可将
`safety.check_pcd_clearance` 设为 true。

## 4. 生成离线对比图

```bash
ros2 run circle_path_benchmark plot_circle_result \
  --session /home/wei/github_code/topo_graph_m20_ws/data/circle_benchmark/runs/<本次目录>
```

输出 `comparison.png`。左图叠加固定圆和里程计轨迹，右图显示有符号径向误差；负值
表示机器人位于参考圆内侧，也就是切弯方向。
