# obstacle_occlusion_extension

该节点订阅滚动地图 `/local_voxel_map/grid` 和里程计 `/lio_odom_hf`。对于每个 hard
体素，以机器人当前位置为观察点，沿“机器人到体素”的水平射线继续向障碍后方延伸，
并将新增且不与原 hard 重叠的体素发布为紫色可视化点云：

```text
/local_voxel_map/occlusion_extension  sensor_msgs/msg/PointCloud2
```

同时发布供规划器使用的地图：

```text
/local_voxel_map/grid_with_occlusion  efficient_3d_local_planner_msgs/msg/VoxelGrid
```

`extension.enabled=true` 时，新增体素合并进输出地图的 hard 层；为 `false`、尚未收到
里程计或 TF 暂时不可用时，输出地图原样透传输入，规划地图链路不会中断。原始
`/local_voxel_map/grid` 不会被修改。默认由
`efficient_3d_local_planner/config/up_and_down.yaml` 配置和启动。

主要参数：

- `extension.enabled`：控制是否计算扩展并合入规划 hard 层；
- `extension.distance`：障碍后方延伸距离；
- `extension.minimum_obstacle_range`：忽略过近体素；
- `extension.maximum_obstacle_range`：参与计算的最远水平距离，`0` 表示不限。
- `output.augmented_grid_topic`：输出给规划器的增强体素地图话题。

新增点直接使用输入栅格的分辨率、原点和窗口范围，不会在滚动地图外生成点。
