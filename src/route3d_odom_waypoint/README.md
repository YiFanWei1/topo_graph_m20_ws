# route3d_odom_waypoint

只使用里程计生成 Route3D `topoGraph_data.json`，不订阅也不保存点云。默认启用纯几何全局
闭环，根据位置、高度、方向和连续匹配距离重新接入旧路线；沿相邻旧边原路退回的识别也保留。

## 编译

```bash
cd /home/wei/github_code/topo_graph_m20_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select route3d_odom_waypoint
source install/setup.bash
```

## 模式一：读取 `/lio_odom`

节点默认立即开始记录，并每秒更新一次输出文件；调用保存服务或按 `Ctrl+C` 时完成最终构图。
实时运行脚本会同时打开 RViz，并每秒刷新原始轨迹、拓扑点、拓扑边、属性标签和闭环边。

```bash
./sh/18_start_live_odom_waypoint.sh
```

控制服务和可视化话题：

```bash
ros2 service call /route3d_odom_waypoint/start std_srvs/srv/Trigger '{}'
ros2 service call /route3d_odom_waypoint/stop_and_save std_srvs/srv/Trigger '{}'
ros2 service call /route3d_odom_waypoint/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /route3d_odom_waypoint/status
ros2 topic echo /route3d_odom_waypoint/path
ros2 topic echo /route3d_odom_waypoint/visualization
```

## 模式二：读取姿态文件

支持真正的 JSON 数组，也支持扩展名为 `.json`、实际每行如下 8 列的文本文件：

```text
timestamp x y z qx qy qz qw
```

```bash
ros2 run route3d_odom_waypoint pose_file_to_topology \
  /home/wei/xili_22/map/slam_data/trajectory/pose.json \
  --output /home/wei/github_code/topo_graph_m20_ws/data/xili_22_odom/topoGraph_data.json
```

常用参数：

```text
--frame-id camera_init
--target-spacing 1.0
--body-height 0.57
--relocation-distance 2.0
--obstacle-mode 0
--geometric-loop-closure
--no-geometric-loop-closure
--no-retrace
--no-slope
```

`body-height` 会从里程计 Z 中扣除，默认 `0.57 m` 与 M20 现有拓扑一致。如果输入文件中的
Z 已经是地面高程，请显式传入 `--body-height 0.0`。

文件模式允许相邻采样最大跳动 `2.0 m`，以兼容低频或缺帧轨迹；实时话题模式仍使用更严格的
`0.50 m`。超过阈值会被视为定位跳变并拆分为不连通分量。

纯几何闭环默认开启，但没有点云重合验证。在上下楼层、平行近距离路线等容易混淆的场景，使用
`--no-geometric-loop-closure` 关闭；实时模式可将 YAML 中的
`geometric_loop_closure_enabled` 设置为 `false`。

## 转换并可视化

```bash
./sh/17_convert_pose_and_visualize.sh
```

RViz 同时显示灰色原始轨迹和拓扑结果。顶点颜色为蓝色普通点、黄色坡点、红色拐点、紫色
交汇点；边按 `obstacleMode` 使用不同颜色，纯几何闭环另外显示为青色粗线。每个顶点显示
`V编号`，每条边显示 `E编号`，不显示冗长属性文字。
