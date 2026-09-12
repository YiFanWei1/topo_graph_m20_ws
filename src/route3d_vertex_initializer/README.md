# route3d_vertex_initializer

根据 Route3D 顶点 ID 发布定位初始化 `/initialpose`。

## 图来源优先级

1. `/route3d_initial_pose/graph_file`：Web Console 当前选中的本地 topology JSON（优先）
2. `/route3d_dijkstra_planner` 的 `graph_file` 参数
3. YAML 中的 `fallback_graph_file`

因此网页中的 topology 可以与 PCD 地图独立选择，网页发送顶点定位时 initializer 一定使用网页当前选中的 topology。

## 输入输出

输入：

```text
/route3d_initial_pose/graph_file   std_msgs/String
/route3d_initial_pose/vertex_id    std_msgs/Int32
```

输出：

```text
/initialpose                            geometry_msgs/PoseWithCovarianceStamped
/route3d_initial_pose/selected_pose     geometry_msgs/PoseStamped
/route3d_initial_pose/status            std_msgs/String
```

位置使用顶点 `pos=[x,y,z]`，姿态使用顶点完整 `rpy=[roll,pitch,yaw]` 并转换为 quaternion。

## 单独启动

```bash
ros2 launch route3d_vertex_initializer vertex_initializer.launch.py
```

命令行也可直接发顶点：

```bash
ros2 run route3d_vertex_initializer set_initial_pose_by_vertex 57
```
