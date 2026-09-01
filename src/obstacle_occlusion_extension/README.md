# obstacle_occlusion_extension

该节点订阅滚动地图 `/local_voxel_map/grid` 和里程计 `/lio_odom_hf`。对于每个 hard
体素，以机器人当前位置为观察点，沿“机器人到体素”的水平射线继续向障碍后方延伸，
并将新增且不与原 hard 重叠的体素发布为：

```text
/local_voxel_map/occlusion_extension  sensor_msgs/msg/PointCloud2
```

当前输出只用于 RViz 验证，不会写回 `/local_voxel_map/grid`，因此不会影响 A*。
默认由 `efficient_3d_local_planner/config/up_and_down.yaml` 配置和启动。

主要参数：

- `extension.enabled`：启用可视化计算；
- `extension.distance`：障碍后方延伸距离；
- `extension.minimum_obstacle_range`：忽略过近体素；
- `extension.maximum_obstacle_range`：参与计算的最远水平距离，`0` 表示不限。

新增点直接使用输入栅格的分辨率、原点和窗口范围，不会在滚动地图外生成点。
