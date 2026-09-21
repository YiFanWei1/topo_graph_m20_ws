# Route3D M20 专用工作空间

本工作空间位于 `/home/wei/github_code/topo_graph_m20_ws`，由通用 Route3D 功能包和
M20 `basic_server` bridge 组成。它不包含机器人步态切换链路；控制器不会发布步态命令，
路线执行也不会等待步态确认。

实机自动打点、拓扑加载、启用控制、发送目标、M20 原生 Bag 录制/回放的逐终端操作流程见
[M20 实机与 Bag 回放操作手册](M20_RUNBOOK.md)。

## 安全启动

```bash
cd /home/wei/github_code/topo_graph_m20_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch route3d_m20_adapter m20_pid_route.launch.xml \
  graph_file:=/home/wei/github_code/topo_graph_m20_ws/data/79792/topoGraph_data.json \
  enable_motion:=false
```

`enable_motion` 默认是 `false`。此时规划、PID、Efficient 3D、Web 和诊断可以运行，
但 M20 adapter 始终向 `/cmd_vel_smoothed` 输出零速度。完成离线检查后才应显式设为
`true`。

实机准备顺序固定为：确认普通使用模式、必要时请求 Stand、请求并确认 RL 状态 17。
准备节点根据协议结果和 `/m20/state/motion_state` 持续重试，发布：

- `/m20/control/ready`
- `/m20/control/preparation_status`

adapter 只在 `enable_motion=true`、ready、RL 状态反馈新鲜、控制源有效且命令未超时时
放行速度，否则持续发布零速度。诊断话题为：

- `/route3d_m20_adapter/status`
- `/route3d_m20_adapter/selected_command`

普通跟踪限幅为 `|vx| <= 0.8 m/s`、`vy = 0`、`|wz| <= 0.5 rad/s`；最终姿态
对准阶段限幅为 `|vx| <= 0.2`、`|vy| <= 0.3`、`|wz| <= 0.5`。

## 拓扑语义

新记录的边和 Web 新建边默认 `obstacleMode=1`；新顶点默认
`alignFinalYaw=false`。最终导航目标无条件按顶点 `rpy.yaw` 对准，中间点仅在
`alignFinalYaw=true` 时停车对准。

| obstacleMode | M20 行为 |
|---:|---|
| 0 | 服从 `controllerMode`，`auto` 默认 PID；PID 启用三维扫掠停障 |
| 1 | 强制 Efficient 3D，整条边主动绕障 |
| 2 | 强制 PID，关闭普通点云停障 |
| 3 | 保留“忽略普通障碍”语义，控制器服从边属性 |
| 4 | 外部栅格控制交接，必须填写 `gridMapName` |

`isSlope` 只描述地形，不再隐式选择控制器或修改障碍模式。新数据不写
`locomotionMode`；加载器可读取旧字段但会忽略它。

拐点保持 `passRadiusM=0.25 m`、提前 `0.70 m` 降速、拐点速度 `0.20 m/s`。

## 坐标与停障

总启动入口发布静态 TF：

```text
body -> base_link: xyz = [0.0, 0.0, 0.0], rpy = [0, 0, 0]
```

当前 M20 定位数据约定 `body` 与 `base_link` 重合，因此两者使用单位变换。
雷达安装位置 `[0.32028, 0.0, -0.013]` 只供 Efficient mapper 设置射线起点，不能再次叠加到 TF。

`/cloud_registered_body` 已在 `base_link` 下，端点不重复应用雷达外参；Efficient mapper
仅用它设置雷达射线起点。导航定位继续使用 `/lio_odom`/`/lio_odom_hf`，
`/m20/state/odometry` 只作诊断。

Bag 回放会过滤录制的旧 `/tf`，由 `/lio_odom_hf` 生成动态
`camera_init -> base_link`。实机 TF 树则为
`map -> camera_init -> body -> base_link`。实机 launch 默认不启用 Bag 专用 TF 重建。

M20 路径高度补偿统一为 `0.57 m`。模式 0 的 PID 三维扫掠使用实体尺寸
`0.82 x 0.43 x 0.57 m` 和带平面余量的检测体 `0.92 x 0.53 x 0.57 m`，按点云
时间戳同步完整三维里程计姿态，过滤机身自点、路径/台阶表面及其上方 0.10 m。

调试输出：

- `/route3d_pid_controller/swept_volume`
- `/route3d_pid_controller/swept_collision_points`
- `/route3d_pid_controller/nearest_hit_distance`
- 默认逐帧日志 `/tmp/route3d_m20_swept_volume.jsonl`

外部急停、碰撞等级和里程计/点云超时对所有模式始终有效。

## 自动打点与 Web

常用入口都通过脚本所在位置自动解析工作空间，不依赖旧机器路径：

```bash
./sh/1_start_auto_waypoint.sh
./sh/2_start_planning_control.sh
./sh/4_send_goal.sh START_ID GOAL_ID
./sh/5_start_loop_patrol.sh
./sh/3_stop_all_ros.sh
```

Web 控制台保留地图、实时雷达、自动打点、点选导航、循环巡航和顶点初始化。边编辑器
使用带说明的 `obstacleMode` 下拉框，不提供步态编辑项。

历史拓扑迁移工具：

```bash
python3 tools/migrate_m20_topology.py --data-root data --apply
```

它只修改拓扑 JSON：自动图全 0 的边迁移到模式 1，自动图全 true 的顶点迁移到 false，
混合值和人工配置保持不变，并在 `data/.m20_migration_backup` 备份受影响文件。

## 三套 bag 验证

`xili`、`regu` 和 `full_nav_replay_with_plan` 都是由高度 **0.40 m** 的录制机器人采集。
重新自动打点必须显式使用 `--body-height 0.40` 还原地面高程；M20 实际运行及扫掠体仍
使用 `0.57 m`。离线 M20 回放因此把检测中心相对录制轨迹上移 `0.17 m`。

验证产物位于：

- `data/m20_validation/{xili,regu,full_nav_replay_with_plan}`：同步位姿、关键帧和重建拓扑
- `validation/bags/*_topology_build.txt`：自动打点报告
- `validation/bags/*_swept_volume.jsonl`：逐帧停障报告
- `validation/bags/*_swept_volume_summary.json`：汇总报告
- `validation/m20_bag_topology_plans.json`：三套重建图的正反向 Dijkstra 验证
- `validation/m20_bag_runtime_replay.json`：`use_sim_time` 整栈 bag 回放与零速门控验证
- `validation/m20_runtime_checks.json`：M20 ready 状态机和 adapter 门控验证
- `validation/mode0_test_topology.json`：不影响正式数据的模式 0 三维停障测试路线
- `data/m20_validation/regu_mode0_1_40`：保留完整 regu 点边、仅将 1–40 改为模式 0/PID 的对比图
- `validation/bags/regu_mode0_1_40_runtime.json`：regu 1–40 停障回放及跨 40 号点控制器切换报告
- `data/m20_validation/regu_mode0_all`：保留完整 regu 点边、全部 108 条边使用模式 0/PID
- `validation/bags/regu_mode0_all_runtime.json`：regu 全路线停障规划和回放报告
- `validation/m20_topology_migration.json`：正式数据迁移报告

完整接口说明见 [M20 端到端说明](docs/route3d_end_to_end_pipeline.md) 和
[拓扑语义](docs/topology_schema_v2_interface.md)。

独立的实机手动速度入口见
[`m20_velocity_control`](src/m20_velocity_control/README.md)。它订阅
`/m20/manual/cmd_vel`，执行 ready/RL 17、急停、超时和限速门控后再发送给专用 bridge；
不得与 Route3D 实机控制栈同时启动。
