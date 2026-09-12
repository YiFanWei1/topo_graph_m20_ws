# 本次修改

1. 实时点云改为按需传输：页面默认不勾选“实时点云”。后端按 WebSocket 客户端记录订阅状态，只有勾选的浏览器才收到二进制点云；无人勾选时不做 PointCloud2 转换、降采样或 WebSocket 编码。
2. 新增直接目标点导航：等价于上传的 `06_send_goal_only.sh`，发布 `/route3d_dijkstra/goal_request` (`std_msgs/msg/Int32`)。规划模块可直接输入目标点，拓扑点属性面板也增加“导航到此点”。
3. 左侧重构为六个模块：雷达、建图、定位、打点、拓扑、规控。模块入口始终显示运行状态；点击模块后才显示其详细控制项。
4. 非“拓扑”模块时隐藏右侧点/边属性面板，扩大 3D 地图区域；进入拓扑模块后恢复属性编辑器。
5. 保留此前所有修改：driver.service 雷达管理、/opt/mapping_ws 建图、地图保存、本地 PCD/拓扑独立选择、点位初始化、cloud_registered_body + odometry 全局可视化等。
