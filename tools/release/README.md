# Route3D M20 二进制 Debian 包

构建只在内部 Ubuntu 24.04 x86_64 / ROS 2 Jazzy 机器执行。构建机需安装本项目
已有的编译依赖，以及 `cython3`、`python3-setuptools`、`python3-dev`、`dpkg-dev`。
例如在构建机执行 `sudo apt install cython3 python3-setuptools python3-dev dpkg-dev`。
客户机仅安装
生成的 `.deb`，不执行 `colcon build`。

```bash
cd ~/github_code/topo_graph_m20_ws
/usr/bin/python3 tools/release/build_deb.py \
  --version 0.1.0-1 \
  --output dist/route3d-m20-runtime_0.1.0-1_amd64.deb
dpkg-deb -c dist/route3d-m20-runtime_0.1.0-1_amd64.deb
```

构建器使用全新的非软链接 Release 安装目录。C++ 以 ELF 可执行文件或共享库交付；
Python 节点及 launch 实现经 Cython 编译为 CPython 3.12 扩展，留在包内的
`.launch.py` 和可执行入口仅负责调用扩展。头文件、静态库、测试、构建目录和源码
不会放进 `.deb`。审核发现未编译的 Python 实现或意外软链接时，打包立即失败。

安装后：

```bash
sudo dpkg -i route3d-m20-runtime_0.1.0-1_amd64.deb
source /opt/ros/jazzy/setup.bash
source /opt/route3d/install/setup.bash
ros2 pkg prefix route3d_product_demo
ros2 launch route3d_product_demo live_route_product.launch.py --show-args
```

- 可编辑的 YAML 位于 `/etc/route3d/<ROS 包名>/` 和 `/etc/route3d/sh/`。
  ROS 安装目录中的原路径是指向这些 YAML 的软链接。Debian 把它们登记为
  `conffiles`，升级时保留客户修改并按 Debian 规则处理冲突。
- 运行数据位于 `/var/lib/route3d/`。安装脚本会在目标机存在 `langyi` 用户时
  将目录交给该用户；其他账户需由部署人员调整权限。
- 现有目标机的地图和拓扑仍在旧工作区 `data/`。首次切换到 `.deb` 版本时，
  需按容量和业务停机窗口把这些运行数据迁移到 `/var/lib/route3d/`；安装包
  不会删除或覆盖旧数据。
- 启动脚本位于 `/opt/route3d/sh/`。本包有意不包含 `sync_to_m20.sh`、
  `sync_from_m20.sh` 或工作区的 `src/`、`build/`、`log/`。
- 目标机需要 ROS 2 Jazzy 及本项目运行依赖。当前包针对现有 M20 环境；迁移到
  新型号或新系统时，应在相同架构、系统和 Python 版本的干净机器上验收。

**保密边界**：客户拥有 root 权限时，可复制和逆向任何本地二进制。Cython/strip
降低直接读取源码的机会，不能保证算法绝对不可恢复。Web 控制台的浏览器
JavaScript 必须视为可见代码；核心商业逻辑不应放入前端。当前开发目标机曾经
保存过 `test/src/` 与 `test.zip` 源码副本；正式交付须使用从未放过源码的干净镜像。
