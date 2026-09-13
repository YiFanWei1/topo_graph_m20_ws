# `regu` 数据生成三维可通行区域的完整流程

本文档对应当前工作区中的实际实现，输入为：

- ROS 2 Bag：`/home/wei/bag/regu/`
- 完整注册地图：`/home/wei/github_code/dddmr_navigation/map/silou180.pcd`
- 工作区：`/home/wei/github_code/topo_graph_m20_ws`

离线提取代码位于：

- `tools/traversability_extraction/extract_bag_trajectory.py`
- `tools/traversability_extraction/extract_traversable_region.cpp`

在线分层与规划代码位于：

- `src/traversability_planner_demo/src/traversability_planner_node.cpp`
- `src/traversability_planner_demo/config/regu_planner.yaml`
- `src/traversability_planner_demo/launch/regu_planner_demo.launch.py`

## 1. 当前方案到底需要哪些输入

当前实现需要两个核心输入：

1. 完整 PCD 地图：提供真实的地面、墙面、栏杆和楼梯点。
2. Bag 中的 `/lio_odom`：提供机器人实际走过的三维轨迹，用作可通行区域生长的种子。

当前版本不读取每一帧 `/cloud_registered_body`。也就是说，现有流程不是重新拼接 Bag 点云，而是利用 Bag 轨迹在完整 PCD 中寻找与轨迹连通的低坡度表面。

本数据中的坐标关系为：

```text
/lio_odom.header.frame_id = camera_init
/lio_odom.child_frame_id  = base_link
PCD 地图坐标系             = camera_init
机器人底盘参考点离地高度     = 0.40 m
```

因此，轨迹地面种子的高度按下面的方式得到：

```text
z_ground = z_body - 0.40
```

如果更换 Bag 或机器人，必须重新标定 `body_height`；否则种子可能落在地面上方或地面下方，后续区域生长会失败或串层。

## 2. 数据流

```text
ROS 2 Bag 的 /lio_odom
        |
        |  提取、按 0.04 m 间距降采样、减去车体离地高度
        v
trajectory_ground.pcd  ----------------------+
                                                |
完整地图 silou180.pcd                           |
        |                                       |
        |  轨迹附近 6 m 裁剪                    |
        v                                       |
map_cropped_downsampled.pcd                     |
        |                                       |
        |  法向、坡度、曲率筛选                  |
        v                                       |
candidate_surface.pcd                           |
        |                                       |
        |  从轨迹种子进行三维连通区域生长 <------+
        v
reachable_before_clearance.pcd
        |
        |  机器人净空检查 + 再次区域生长
        v
traversable_surface_geometric.pcd
        |
        |  与轨迹插值带 route_supported_surface.pcd 合并
        v
traversable_surface.pcd
        |
        |  启动 ROS 节点后计算硬障碍、软膨胀和 A*
        v
绿色 free / 橙色 inflation / 红色 lethal / 黄色 path
```

## 3. 环境和输入检查

先检查 Bag 是否包含需要的里程计：

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info /home/wei/bag/regu
```

这份 Bag 中与当前流程有关的话题为：

```text
/lio_odom               2179 条，低频关键位姿
/cloud_registered_body  2179 条，与关键位姿对应的点云
/lio_odom_hf            43581 条，高频里程计
/tf                     2179 条
```

当前程序只使用 `/lio_odom`。`/cloud_registered_body` 可以用于以后实现更可靠的逐帧地面边界提取，但不参与本版离线结果生成。

检查 PCD 文件存在：

```bash
ls -lh /home/wei/github_code/dddmr_navigation/map/silou180.pcd
head -n 12 /home/wei/github_code/dddmr_navigation/map/silou180.pcd
```

## 4. 第一步：从 Bag 提取轨迹地面种子

执行：

```bash
source /opt/ros/jazzy/setup.bash

python3 /home/wei/github_code/topo_graph_m20_ws/tools/traversability_extraction/extract_bag_trajectory.py \
  --bag /home/wei/bag/regu \
  --output-dir /home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result \
  --topic /lio_odom \
  --body-height 0.40 \
  --min-spacing 0.04
```

关键逻辑如下。先按三维距离对连续位姿降采样：

```python
xyz = (point.x, point.y, point.z)
if last_xyz is not None:
    distance = math.dist(xyz, last_xyz)
    if distance < args.min_spacing:
        continue

poses.append(
    (
        timestamp,
        point.x,
        point.y,
        point.z,
        quat.x,
        quat.y,
        quat.z,
        quat.w,
    )
)
last_xyz = xyz
```

然后将车体位姿转换为地面种子：

```python
for index, (_, x, y, z, *_quat) in enumerate(poses):
    z_ground = z - args.body_height
    stream.write(f"{x:.9f} {y:.9f} {z_ground:.9f} {index:.1f}\n")
```

输出文件为：

```text
trajectory.csv           完整位姿和四元数
trajectory_ground.pcd    只保留的地面种子点
trajectory_summary.json  话题、坐标系、点数和范围
```

当前数据会得到 1476 个轨迹种子点。Bag 原始 `/lio_odom` 有 2179 条，数量减少是因为使用了 `0.04 m` 的空间间距降采样。

## 5. 第二步：编译离线可通行区域提取器

依赖为 PCL、Eigen 和 OpenMP。当前源码可以用下面的命令直接编译：

```bash
cd /home/wei/github_code/topo_graph_m20_ws

g++ -std=c++17 -O3 -fopenmp \
  tools/traversability_extraction/extract_traversable_region.cpp \
  -o tools/traversability_extraction/extract_traversable_region \
  $(pkg-config --cflags --libs \
    pcl_common pcl_io pcl_filters pcl_features pcl_kdtree pcl_search)
```

## 6. 第三步：从 PCD 中提取可通行区域

执行：

```bash
/home/wei/github_code/topo_graph_m20_ws/tools/traversability_extraction/extract_traversable_region \
  /home/wei/github_code/dddmr_navigation/map/silou180.pcd \
  /home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result/trajectory_ground.pcd \
  /home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result
```

以下小节按源码的实际执行顺序说明每一步。

### 6.1 参数

离线参数当前直接写在 `extract_traversable_region.cpp` 的 `Config` 中：

```cpp
struct Config {
  float crop_radius = 6.0F;
  float voxel = 0.10F;
  int normal_k = 24;
  float max_slope_deg = 30.0F;
  float max_curvature = 0.16F;
  float seed_radius = 0.45F;
  float connection_radius = 0.24F;
  float max_neighbor_dz = 0.22F;
  float clearance_radius = 0.32F;
  float clearance_min_z = 0.14F;
  float clearance_height = 0.85F;
  float traversed_override_radius = 0.45F;
  float corridor_radius = 0.65F;
  float route_sample_spacing = 0.04F;
  float route_half_width = 0.25F;
  float route_lateral_spacing = 0.05F;
};
```

### 6.2 去除无效点，并裁剪轨迹附近地图

先去掉 NaN/Inf。随后为轨迹建立 KD-tree，每个地图点只要距离最近轨迹点不超过 `6.0 m` 就保留：

```cpp
pcl::KdTreeFLANN<Point> trajectory_tree;
trajectory_tree.setInputCloud(trajectory);

for (const auto & p : *finite) {
  if (trajectory_tree.nearestKSearch(p, 1, one_index, one_distance) > 0 &&
      one_distance[0] <= cfg.crop_radius * cfg.crop_radius) {
    cropped->push_back(p);
  }
}
```

这里比较的是 KD-tree 返回的距离平方，因此阈值也必须平方。

### 6.3 体素降采样

将裁剪后的地图降采样为 `0.10 m` 体素，以控制法向估计和区域生长的计算量：

```cpp
pcl::VoxelGrid<Point> voxel;
voxel.setInputCloud(cropped);
voxel.setLeafSize(cfg.voxel, cfg.voxel, cfg.voxel);
voxel.filter(*downsampled);
```

结果保存为 `map_cropped_downsampled.pcd`。

### 6.4 重新估计法向、坡度和曲率

原始地图中的法向不能假定有效，所以在降采样点云上重新用 24 个近邻估计法向：

```cpp
pcl::NormalEstimationOMP<Point, pcl::Normal> normal_estimator;
normal_estimator.setNumberOfThreads(0);
normal_estimator.setInputCloud(downsampled);
normal_estimator.setSearchMethod(normal_tree);
normal_estimator.setKSearch(cfg.normal_k);
normal_estimator.compute(*normals);
```

表面坡度由法向的竖直分量得到：

```text
slope = acos(|normal_z|)
```

候选面必须同时满足：

```text
坡度 <= 30 deg
曲率 <= 0.16
```

对应代码：

```cpp
const float min_abs_nz = std::cos(cfg.max_slope_deg * pi / 180.0F);

const float abs_nz = std::abs(n.normal_z);
if (abs_nz < min_abs_nz || n.curvature > cfg.max_curvature) {
  continue;
}

Point point = downsampled->points[i];
point.intensity =
  std::acos(std::clamp(abs_nz, 0.0F, 1.0F)) * 180.0F / pi;
candidates->push_back(point);
```

此时 `intensity` 不再表示激光反射强度，而是坡度角（度）。结果保存为 `candidate_surface.pcd`。

候选面只是“几何上像地面”，还不是最终可通行区域。例如桌面、窗台或别的楼层也可能满足低坡度条件。

### 6.5 从轨迹种子进行三维连通区域生长

每个轨迹种子先匹配 `0.45 m` 内最近的候选面，然后从所有匹配点同时进行广度优先搜索：

```cpp
if (tree->nearestKSearch(seed, 1, index, squared_distance) == 0) {
  continue;
}
if (squared_distance[0] > cfg.seed_radius * cfg.seed_radius) {
  continue;
}

const int id = index[0];
visited[id] = 1;
frontier.push(id);
```

搜索过程中，只连接三维半径 `0.24 m` 内、相邻高度差不超过 `0.22 m` 的点：

```cpp
tree->radiusSearch(
  candidate->points[current], cfg.connection_radius,
  neighbors, squared_distance);

for (const int next : neighbors) {
  const auto & a = candidate->points[current];
  const auto & b = candidate->points[next];
  const float dxy = std::hypot(b.x - a.x, b.y - a.y);
  const float dz = std::abs(b.z - a.z);

  if (dxy < cfg.voxel * 0.35F || dz > cfg.max_neighbor_dz) {
    continue;
  }
  visited[next] = 1;
  frontier.push(next);
}
```

第一次区域生长结果保存为 `reachable_before_clearance.pcd`。

这一步可以排除没有和机器人轨迹连通的其他楼层或水平物体，但它只能保证点云拓扑连通，不能保证机器人身体不会碰到墙面或栏杆。

### 6.6 机器人净空检查

对第一次区域生长得到的每个地面点，在完整降采样地图中查找一个圆柱体范围：

```text
水平半径：0.32 m
地面上方：0.14 m ~ 0.85 m
```

如果圆柱体内存在不属于同一连续地面的点，就将该地面点判为被障碍阻挡：

```cpp
map_tree.radiusSearch(ground, clearance_query, nearby, nearby_distance);
bool blocked = false;

for (const int map_id : nearby) {
  const auto & obstacle = downsampled->points[map_id];
  const float dxy = std::hypot(obstacle.x - ground.x, obstacle.y - ground.y);
  const float dz = obstacle.z - ground.z;

  if (dxy > cfg.clearance_radius ||
      dz < cfg.clearance_min_z ||
      dz > cfg.clearance_height) {
    continue;
  }

  const bool same_surface = downsampled_is_candidate[map_id] &&
    std::abs(dz) <= (0.08F + max_grade * dxy);

  if (!same_surface) {
    blocked = true;
    break;
  }
}
```

随后只允许 `clearance_ok` 的候选点再次进行区域生长，结果保存为 `traversable_surface_geometric.pcd`。

当前源码有一个必须注意的例外：距离历史轨迹不超过 `0.45 m` 的候选点会直接通过净空检查。

```cpp
if (trajectory_tree.nearestKSearch(ground, 1, one_index, one_distance) > 0 &&
    one_distance[0] <=
      cfg.traversed_override_radius * cfg.traversed_override_radius) {
  clearance_ok[candidate_id] = 1;
  continue;
}
```

这个设计的本意是保留机器人实际走过但受墙面、栏杆点云干扰的狭窄区域；副作用是轨迹误差较大时可能放过真实障碍。

### 6.7 对稀疏楼梯生成连续轨迹带

楼梯点云可能很稀疏，单靠 `0.24 m` 邻域会断开。当前代码在相邻轨迹点之间以 `0.04 m` 插值，并沿轨迹法向左右扩展 `0.25 m`：

```cpp
const int along_steps = std::max(
  1, static_cast<int>(std::ceil(distance / cfg.route_sample_spacing)));

for (int along = 0; along <= along_steps; ++along) {
  const float t = static_cast<float>(along) / along_steps;
  const float center_x = a.x + t * dx;
  const float center_y = a.y + t * dy;
  const float center_z = a.z + t * dz;

  for (int lateral = -lateral_steps; lateral <= lateral_steps; ++lateral) {
    const float offset = std::clamp(
      lateral * cfg.route_lateral_spacing,
      -cfg.route_half_width, cfg.route_half_width);

    Point point;
    point.x = center_x + last_lateral_x * offset;
    point.y = center_y + last_lateral_y * offset;
    point.z = center_z;
    point.intensity = slope_deg;
    raw->push_back(point);
  }
}
```

结果保存为 `route_supported_surface.pcd`，随后与几何可通行面直接合并：

```cpp
*continuous_raw = *traversable_geometric;
*continuous_raw += *route_ribbon;

continuous_voxel.setInputCloud(continuous_raw);
continuous_voxel.setLeafSize(0.05F, 0.05F, 0.05F);
continuous_voxel.filter(*traversable);
```

最终保存为 `traversable_surface.pcd`。

> 重要：轨迹带是由里程计轨迹人工插值得到的，不是从 PCD 中重新检测出的真实地面。当前代码没有对轨迹带的每个插值点做“附近是否存在真实地面支撑”的检查。因此，轨迹有漂移、穿墙或跨越缺失点云时，轨迹带可能制造一条假的可规划通道。这正是当前方案最主要的安全风险。

### 6.8 生成障碍点云

当前离线程序把所有未通过坡度/曲率候选面筛选的降采样点作为障碍：

```cpp
auto obstacles = selectCloud(
  downsampled,
  [&](std::size_t i) { return !downsampled_is_candidate[i]; });
```

结果保存为 `obstacle_points.pcd`。墙面通常会进入该文件，但原地图缺点、墙体太稀疏或法向误判时，障碍层也会出现缺口。

## 7. 离线输出文件及含义

输出目录为：

```text
/home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result/
```

主要文件：

| 文件 | 含义 | 是否进入当前规划器 |
|---|---|---|
| `trajectory_ground.pcd` | Bag 轨迹减去车体高度后的地面种子 | 是，用于轨迹核心豁免 |
| `map_cropped_downsampled.pcd` | 轨迹附近裁剪并降采样后的地图 | 否，诊断用 |
| `candidate_surface.pcd` | 坡度和曲率合格的所有候选面 | 否，RViz 诊断用 |
| `reachable_before_clearance.pcd` | 第一次区域生长结果 | 否，诊断用 |
| `traversable_surface_unfiltered.pcd` | 不执行高度范围点去除的可行面 | 是，开关为 `false` 时使用 |
| `traversable_surface_original.pcd` | 保留旧版 `0.14～0.85 m` 规则的对照结果 | 否，RViz 对照用 |
| `traversable_surface_geometric.pcd` | 通过净空检查后的真实几何可通行面 | 间接进入最终面 |
| `route_supported_surface.pcd` | 根据轨迹人工插值得到的连续带 | 间接进入最终面 |
| `traversable_surface.pcd` | 执行当前高度范围过滤后的可行面 | 是，开关为 `true` 时使用 |
| `obstacle_points.pcd` | 未通过候选面筛选的点 | 是 |
| `traversed_corridor.pcd` | 轨迹附近可通行区域，供显示 | 否，RViz 诊断用 |
| `extraction_summary.json` | 点数和参数统计 | 否 |

当前这组数据的结果为：

```text
完整地图点数                    5,816,660
轨迹种子点数                       1,476
轨迹附近裁剪点数                 2,744,445
0.10 m 降采样点数                 359,891
候选表面点数                      115,002
第一次连通区域点数                 33,814
净空检查阻挡点数                   15,955
几何可通行点数                     14,623
轨迹插值带点数                     31,075
无高度过滤可行面点数               56,580
旧规则可行面点数                   37,389
当前高度过滤可行面点数             47,478
轨迹走廊点数                       32,260
```

## 8. 在线生成绿色、橙色和红色分层

提取器会同时保存无高度过滤和高度过滤结果。启动 `traversability_planner_node` 后，由 YAML 开关选择规划面；开启时再根据每个候选节点到障碍物的距离进一步分类。

当前运行参数位于 `regu_planner.yaml`：

```yaml
traversability_planner:
  ros__parameters:
    frame_id: camera_init
    surface_pcd: /home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result/traversable_surface_unfiltered.pcd
    filtered_surface_pcd: /home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result/traversable_surface.pcd
    obstacle_pcd: /home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result/obstacle_points.pcd
    route_pcd: /home/wei/github_code/topo_graph_m20_ws/data/regu/traversability_result/trajectory_ground.pcd

    # false：完全不执行高度范围点去除；true：启用下面的高度范围。
    height_range_filter.enabled: false
    height_range_filter.min_height: 0.40
    height_range_filter.max_height: 0.41

    graph.connection_radius: 0.16
    graph.max_neighbor_dz: 0.10

    inflation.hard_radius: 0.18
    inflation.soft_radius: 0.80
    inflation.traversed_core_radius: 0.12
    inflation.max_free_support_distance: 0.30

    cost.inflation_weight: 10.0
    cost.slope_weight: 0.015
    snap.search_radius: 1.50
    path.z_offset: 0.08
```

### 8.1 第一次分层

当 `height_range_filter.enabled: true` 时，对于每个可通行面点，只统计相对地面高度 `0.40 m ~ 0.41 m` 内的障碍物，然后计算最小水平距离 `minimum_xy`。当开关为 `false` 时直接使用 `traversable_surface_unfiltered.pcd`，所有节点进入自由层，不执行以下判断：

```cpp
const float dz = obstacle.z - ground.z;
if (dz < obstacle_min_height_ || dz > obstacle_max_height_) {
  continue;
}

minimum_xy = std::min(
  minimum_xy,
  std::hypot(obstacle.x - ground.x, obstacle.y - ground.y));

if (minimum_xy <= hard_radius_) {
  // 红色：硬障碍，A* 禁止进入
  lethal_[i] = true;
  costs_[i] = 1.0F;
  lethal_ids_.push_back(i);
} else if (minimum_xy < soft_radius_) {
  // 橙色：软膨胀，允许进入但代价较高
  costs_[i] =
    (soft_radius_ - minimum_xy) / (soft_radius_ - hard_radius_);
  inflation_ids_.push_back(i);
} else {
  // 绿色：自由区域
  free_ids_.push_back(i);
}
```

颜色含义：

```text
绿色 free       距离有效障碍 >= 0.80 m
橙色 inflation  距离有效障碍在 0.18 m 和 0.80 m 之间
红色 lethal     距离有效障碍 <= 0.18 m，A* 不允许通过
```

软膨胀代价归一化为：

```text
inflation = (soft_radius - distance) / (soft_radius - hard_radius)
```

### 8.2 橙色点必须有 0.30 m 内的绿色点支撑

当前代码对所有绿色点建立三维 KD-tree。每个橙色点到最近绿色点的三维距离若超过 `0.30 m`，就从橙色改成红色：

```cpp
pcl::KdTreeFLANN<Point> free_tree;
free_tree.setInputCloud(free_cloud);

const float max_distance_squared =
  max_free_support_distance_ * max_free_support_distance_;

for (const int id : inflation_ids_) {
  if (free_tree.nearestKSearch(
        surface_->points[id], 1,
        nearest_id, nearest_distance) > 0 &&
      nearest_distance[0] <= max_distance_squared) {
    supported_inflation.push_back(id);
  } else {
    lethal_[id] = true;
    costs_[id] = 1.0F;
    lethal_ids_.push_back(id);
    ++rejected_soft_count_;
  }
}
```

这里必须使用三维 KD-tree，不能只按 XY 判断，否则上下重叠的两个楼层可能互相提供错误支撑。

当前启动统计为：

```text
surface       37,389
green/free    14,600
orange        12,665
red/lethal    10,124
橙转红          4,372
```

### 8.3 当前仍存在的轨迹核心豁免

在线代码在检查障碍之前，会把距离历史轨迹 `0.12 m` 内的可通行面点直接分类为绿色：

```cpp
if (route_tree_->nearestKSearch(
      ground, 1, route_id, route_distance) > 0 &&
    route_distance[0] <=
      traversed_core_radius_ * traversed_core_radius_) {
  free_ids_.push_back(i);
  continue;
}
```

因此，橙色到绿色 `0.30 m` 检查能消除孤立的软膨胀岛，但无法证明这个绿色轨迹核心本身一定安全。如果历史轨迹穿墙或高度偏移，仍可能保留错误绿色核心。

## 9. A* 如何在三维点云上规划

规划图不预先保存边，而是在扩展节点时通过 KD-tree 查询 `0.16 m` 范围内的邻居。满足以下条件才允许连接：

```text
邻居不是红色 lethal
两点三维距离 <= 0.16 m
两点高度差 <= 0.10 m
```

核心代码：

```cpp
surface_tree_->radiusSearch(
  surface_->points[current],
  connection_radius_, neighbors, squared_distances);

for (std::size_t i = 0; i < neighbors.size(); ++i) {
  const int next = neighbors[i];
  if (next == current || lethal_[next] || closed[next]) {
    continue;
  }

  const auto & a = surface_->points[current];
  const auto & b = surface_->points[next];
  if (std::abs(a.z - b.z) > max_neighbor_dz_) {
    continue;
  }
```

每条边的代价为：

```text
step_cost = distance *
            (1 + inflation_weight * inflation^2
               + slope_weight * normalized_slope)
```

对应实现：

```cpp
const float inflation = costs_[next];
const float slope = std::clamp(
  std::abs(b.intensity) / 45.0F, 0.0F, 2.0F);

const float step = distance *
  (1.0F + inflation_weight_ * inflation * inflation +
   slope_weight_ * slope);
```

所以橙色区域不是绝对禁止，而是越靠近障碍代价越高；红色区域才是硬禁止。

## 10. 编译并启动 RViz

编译 ROS 2 包：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select traversability_planner_demo \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

启动规划器和 RViz：

```bash
cd /home/wei/github_code/topo_graph_m20_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch traversability_planner_demo \
  regu_planner_demo.launch.py \
  use_rviz:=true
```

RViz 中主要话题为：

| 话题 | 内容 |
|---|---|
| `/traversability/candidate` | 仅通过坡度/曲率筛选的候选面 |
| `/traversability/corridor` | 历史轨迹附近走廊 |
| `/traversability/trajectory` | Bag 地面轨迹 |
| `/traversability/planning_surface` | 带代价值的完整规划面 |
| `/traversability/free` | 绿色自由点 |
| `/traversability/inflation` | 橙色软膨胀点 |
| `/traversability/lethal` | 红色硬障碍点 |
| `/traversability/raw_obstacles` | 原始障碍点云 |
| `/traversability/plan` | A* 路径 |

### 10.1 发送起点和终点

多楼层场景建议使用 RViz 的 `Publish Point`，因为它保留鼠标点击处的三维 Z：

1. 第一次点击设置起点。
2. 第二次点击设置终点并立即规划。
3. 第三次点击会开始下一轮，重新设置起点。

也可以使用 `2D Pose Estimate` 和 `2D Goal Pose`，但这两个工具通常没有可靠的楼层高度提示；在上下楼层 XY 重叠时可能吸附到错误楼层。

终端中可检查节点和路径：

```bash
ros2 topic list | rg '^/traversability/'
ros2 topic echo /traversability/plan --once
```

## 11. 一次性复现命令

下面的命令会重新生成离线结果、编译规划器并启动 RViz：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/wei/github_code/topo_graph_m20_ws

python3 tools/traversability_extraction/extract_bag_trajectory.py \
  --bag /home/wei/bag/regu \
  --output-dir data/regu/traversability_result \
  --topic /lio_odom \
  --body-height 0.40 \
  --min-spacing 0.04

g++ -std=c++17 -O3 -fopenmp \
  tools/traversability_extraction/extract_traversable_region.cpp \
  -o tools/traversability_extraction/extract_traversable_region \
  $(pkg-config --cflags --libs \
    pcl_common pcl_io pcl_filters pcl_features pcl_kdtree pcl_search)

tools/traversability_extraction/extract_traversable_region \
  /home/wei/github_code/dddmr_navigation/map/silou180.pcd \
  data/regu/traversability_result/trajectory_ground.pcd \
  data/regu/traversability_result

colcon build \
  --packages-select traversability_planner_demo \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
ros2 launch traversability_planner_demo \
  regu_planner_demo.launch.py \
  use_rviz:=true
```

## 12. 参数调整建议

| 目标 | 参数 | 调整方向 | 主要副作用 |
|---|---|---|---|
| 更远离障碍 | `inflation.soft_radius` | 增大 | 橙色范围变大，窄通道可能代价过高 |
| 增大绝对禁行边界 | `inflation.hard_radius` | 增大 | 红色变多，窄楼梯可能完全不连通 |
| 提高橙色区域可信度 | `inflation.max_free_support_distance` | 减小 | 孤立橙色更容易转红，也可能误删窄通道 |
| 增大离线车体净空 | `clearance_radius` | 增大 | 几何可通行面明显减少 |
| 允许更陡楼梯 | `max_slope_deg` | 增大 | 墙边斜面和噪声更容易被当成候选地面 |
| 连接稀疏楼梯 | `connection_radius` | 增大 | 可能跨过真实间隙或连接邻近楼层 |
| 限制上下串层 | `max_neighbor_dz` | 减小 | 楼梯踏步更容易断开 |
| 加宽轨迹补带 | `route_half_width` | 增大 | 假通道和穿墙风险同步增加 |

参数修改位置不同：

- 离线几何参数在 `extract_traversable_region.cpp`，修改后必须重新编译并重新生成 PCD。
- 在线膨胀与 A* 参数在 `regu_planner.yaml`，修改后重启 launch 即可。

## 13. 当前方案的安全边界和下一步改进

当前结果适合算法验证和 RViz 演示，但不能直接作为实车的唯一安全层，原因如下：

1. 轨迹插值带直接并入最终规划面，没有逐点验证真实 PCD 地面支撑。
2. 离线阶段距离轨迹 `0.45 m` 内会跳过净空检查。
3. 在线阶段距离轨迹 `0.12 m` 内会先判为绿色，再跳过障碍检查。
4. `obstacle_points.pcd` 由“非候选表面”得到；地图本身缺点时，不会自动产生封闭墙体。
5. 当前 Bag 不含原始工程的 `/lego_loam_ground_edge`，因此没有直接复用原工程逐扫描线生成的地面外缘。

如果要解决楼梯附近假通道和穿墙，推荐按下列顺序改进：

```text
第一优先级：轨迹带每个点必须在 0.15~0.25 m 内找到真实候选地面支撑；
            找不到支撑就不加入 traversable_surface.pcd。

第二优先级：轨迹核心只能放宽坡度或连通性，不能绕过硬障碍检查。

第三优先级：同步读取每帧 /cloud_registered_body 和 /lio_odom，
            在传感器视角中检测地面边缘、台阶边缘和不可观测边界，
            再变换到 camera_init 中累积为硬边界层。

第四优先级：实车运行时叠加实时局部障碍地图，静态 PCD 只负责全局引导。
```

其中第三项需要每帧点云和对应位姿。当前 Bag 中两者各有 2179 条，具备同步重建的基础；但 `/cloud_registered_body` 没有激光 `ring` 字段，所以无法逐字复现原始 LeGO-LOAM 的扫描线地面边缘算法，需要改成基于局部法向、栅格高度差或射线可见性的边界提取。

## 14. 结果验收清单

每次重新生成后至少检查：

- `trajectory_ground.pcd` 是否贴在地面，而不是悬空或埋入地面。
- 楼梯段 `candidate_surface.pcd` 是否存在足够候选点。
- `traversable_surface_geometric.pcd` 是否连续，断点发生在法向、曲率、净空还是连通阈值。
- `route_supported_surface.pcd` 是否穿墙；它若穿墙，最终面也会被污染。
- 红色层是否覆盖墙边和栏杆根部。
- 橙色层是否始终能在三维 `0.30 m` 内找到绿色区域。
- A* 黄色路径是否只经过绿色或橙色，从不经过红色。
- 对五楼到四楼的可疑位置 `(-26.7, -1.28, -4.37)` 单独放大检查真实地图、轨迹带、绿色、橙色和红色五层是否一致。
