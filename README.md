# CorrGen C++：手动选点 → A* → 完整参数走廊

启动地图后，在 RViz 手动选择起点和目标点，计算三维 A* 路径并生成覆盖全程的可微参数走廊。**不进行轨迹优化。** 交互参考 `ros1_ws/firi_benchmark` 的 `global_planning` 流程。

## 直接运行

现有 Docker 容器 `ros1_noetic` 内：

```bash
source /root/ws/corr_gen/devel/setup.bash
roslaunch corrgen_cpp medium.launch
```

按以下顺序操作：

1. 等待地图出现、状态显示 `WAITING_FOR_START`。启动不会自动选点或生成路径。
2. 点击 RViz 工具栏 **2D Nav Goal**，在自由空间点击并拖出箭头，设定起点（绿色球）。
3. 再次点击 **2D Nav Goal**，选择目标点（红色球）。此时才开始 A* 和走廊生成。
4. 橙色为完整 A*，青色为全程光滑参考，紫色为参数走廊。状态显示 `PASS` 后可继续选下一对起终点。
5. 新一轮选点会清除旧路径和走廊。占据/越界点会提示 `INVALID_POINT`；无路或求解失败会提示原因，不保留旧走廊冒充新结果。

也可用 **2D Pose Estimate** 明确重设起点，再用 **2D Nav Goal** 指定目标。`Publish Point` 使用消息的实际 xyz（点到障碍表面通常会被拒绝）。当前不进行跨 frame 的 TF 转换，RViz Fixed Frame 必须为 `map`。

### 高度

- `sparse/medium/dense` 默认 `height_mode:=arrow`，与 FIRI 示例相同，箭头的旋转决定高度：`z_min + inset + |q_z| (z_size−2 inset)`，`inset=ceil((inflation+robot_radius+safety_margin+reference_clearance)/resolution)*resolution+resolution/2`。
- `balls/multi_gap/maze` 默认固定高度 1.5 m，箭头方向不影响高度。
- 所有 launch 都支持 `height_mode:=fixed height:=1.5`，避免手动调高度。

例如：

```bash
roslaunch corrgen_cpp medium.launch height_mode:=fixed height:=1.5
```

## 地图入口

每次运行一个；关闭 RViz 会同时结束本次 launch。

| launch | 原版 kr_param_map 地图 |
|---|---|
| `sparse.launch` | 稀疏结构化三维地图 |
| `medium.launch` | 中等密度结构化三维地图 |
| `dense.launch` | 稠密结构化三维地图 |
| `balls_demo.launch` | 圆柱障碍图像地图 |
| `multi_gap_demo.launch` | 交错门图像地图 |
| `maze_demo.launch` | 迷宫图像地图 |

这些入口不包含固定起终点。结构化地图对齐 FIRI `global_planning.launch`：**50×50×5 m，原点 (-25,-25,0)，分辨率 0.25 m**，RViz 使用按 Z 高度着色的 Boxes。图像地图仍为 20×10×3 m、分辨率 0.2 m。旧 `sparse_demo/medium_demo/dense_demo.launch` 保留为相同配置的兼容入口。

```bash
roslaunch corrgen_cpp sparse.launch
roslaunch corrgen_cpp medium.launch
roslaunch corrgen_cpp dense.launch
# 等价的统一入口，可换随机种子及单独调整障碍物比例
roslaunch corrgen_cpp global_planning.launch density:=dense map_seed:=1024
```

| 密度 | cylinder_ratio | poly_ratio | circle_ratio | ellip_ratio |
|---|---:|---:|---:|---:|
| sparse | 0.03 | 0.003 | 0 | 0 |
| medium | 0.06 | 0.006 | 0 | 0 |
| dense | 0.10 | 0.010 | 0.01 | 0.01 |

上游多面体生成使用独立 `random_device`，因此相同 `map_seed` 不保证点云逐点一致；每次地图更新会保存实际 `map_<version>.xyz`，请求快照引用该文件供复查。

三档均为 `gate_ratio=0`、`w1/w2/w3=0.10/0.50/2.00`；这些是上游生成器的比例参数，不是固定障碍物个数。`global_planning.launch` 还支持 `map_size_x/y/z`、`map_x/y/z_origin`、`resolution` 和各比例参数。若改变分辨率，可在 RViz Map 显示中同步修改 Size (m)。

通用地图可通过 `demo.launch` 的 `map_type`、`map`、`density`、`seed`、`size_x/y/z`、`resolution` 调整。`inflation` 为附加地图膨胀，默认 0；与 `robot_radius`（0.2 m）和 `safety_margin`（0.05 m）相加形成机体配置空间。`reference_clearance`（结构化地图 0.1 m、图像地图 0.05 m）用于搜索及参考圆滑时预留走廊宽度。`degree` 默认为 6。无桌面测试加 `rviz:=false`。

## 求解、走廊宽度与预算

当前默认 `solver:=highs`，保持 6 阶、35 变量。已删除上一版的全局缩小循环：碰撞点反馈成 LP 新约束重新优化，只有完整实体检查通过才发布 `FEASIBLE_BODY_CONSTRAINED`。检查保留椭圆方向和偏移，避免用大球包住扁长截面造成误拒绝。`shrink_factor=1`，不会靠缩成细管取得 PASS。

A* 默认 `clearance_weight:=4`，倾向有净空的路径；路径简化同时保留此代价。设为 0 可对照纯长度路径，但贴障路线可能限制后续走廊宽度。机器人半径和安全裕量仍参与占据体素膨胀，额外几何切点采用最多 1 cm 内法向裕量。

```bash
roslaunch corrgen_cpp dense.launch
roslaunch corrgen_cpp dense.launch solver:=proxqp
```

默认 `job_budget_ms:=5000`，覆盖整个 worker；`solver_budget_ms:=1000` 是每次内层 LP 预算。六地图正反向回归完整耗时约 1.6–3.7 秒。更宽的实体走廊需要更多几何迭代，不能再用旧版亚秒细管的耗时代表本版。超时且无完整验证结果时失败，不发布旧走廊或缩小初值。reset、新起点、地图变化会回收旧进程并清空输出。

质量对比、连续检查推导和剩余限制见 [走廊质量修正](docs/quality_repair.md)。历史求解加速对比保留在 [上一版工程报告](docs/engineering_delivery.md)。

## 构建

宿主机工作空间：`/home/diffbot/ros1_ws/corr_gen`；容器挂载位置：`/root/ws/corr_gen`。

```bash
cd /home/diffbot/ros1_ws/corr_gen
bash src/CorrGen/scripts/build_ros1.sh ros1_noetic
```

或在容器内：

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws/corr_gen
bash src/CorrGen/scripts/setup_solvers.sh /root/ws/corr_gen
catkin_make -DCMAKE_BUILD_TYPE=Release -DCORRGEN_BUILD_ROS1=ON \
  -DCMAKE_PREFIX_PATH="/root/ws/corr_gen/.deps/highs;/opt/ros/noetic" \
  -DPROXQP_INCLUDE_DIR=/root/ws/corr_gen/.deps/proxsuite/cmeel.prefix/include -j2
cd build/CorrGen && ctest --output-on-failure
```

依赖：Eigen3、GLPK、ROS Noetic、nlohmann-json3-dev。默认 demo 另需 HiGHS 1.10.0，ProxQP 对比需 ProxSuite 0.6.3 的 C++ 头文件；脚本在工作空间 `.deps` 隔离安装它及 HiGHS 1.10.0，C++ 求解过程不调用 Python。上游地图还需要 PCL、SDL1.2、SDL_image。当前容器已安装并编译通过。上游 `src/kr_param_map` 未修改，commit `12dae849e05f28c9fbbcab2b3f18ba793fb536e2`。

纯 C++：

```bash
cmake -S src/CorrGen -B src/CorrGen/build -DCMAKE_BUILD_TYPE=Release
cmake --build src/CorrGen/build -j2
ctest --test-dir src/CorrGen/build --output-on-failure
```

当前宿主 GLPK 位于 `.deps/usr`，配置需增加 `-DCMAKE_PREFIX_PATH=/home/diffbot/ros1_ws/corr_gen/.deps/usr` 和相应 `CMAKE_BUILD_RPATH`；运行时将 `.deps/usr/lib/x86_64-linux-gnu` 加入 `LD_LIBRARY_PATH`。不要在 Ubuntu 20.04 容器中加载宿主 Ubuntu 24.04 库。

## 流程与代码

`原版地图节点 → 占据体素 → 手动选择两点 → 26 邻域 A* → 全程 C² 三次 B-spline 参考 → 全部横截面交点 → 全局 Chebyshev LP → 检查 → 可视化`

- `voxel_planner.*`：三维 A*，禁止穿占据角；可见性简化、局部圆滑和递归 Bézier 包围盒检查。支持反向、多转弯、不同高度和同一自由体素内的不同起终点。
- `path.*`：任意方向的三次段和分段 C² 参考；统一 u∈[0,1]，精确保留两端。标架使用整条路径共同的非退化参考轴，支持竖直段。
- `corridor.*`、`generator.cpp`：论文 3D 对角占优 LP、一二阶解析导数、浮点 Bernstein 正定认证。只有一组全局走廊系数，不将独立截面拼接成假连续走廊。
- `solver.cpp` / `proxqp_adapter.cpp`：统一 LP 后端与有限工作集求解。
- `apps/worker.cpp`：快照读取、规划、并行全投影、求解、连续体素包围检查、原子结果文件。
- `ros/demo_node.cpp`：地图缓存、选点状态、worker 进程生命周期、预算、清除旧结果和可视化。
- `launch/`、`rviz/`：六个交互地图入口，未接入轨迹优化器。

旧的“固定起终点、x 必须递增、单条三次曲线拟合整个 A*”demo 已替换，旧平面拟合器已移除。参考曲线光滑化属于走廊的几何参数化，不是时间参数化或动力学轨迹优化。

## 验证与结果

```bash
docker exec ros1_noetic bash -lc \
  'source /root/ws/corr_gen/devel/setup.bash; python3 /root/ws/corr_gen/src/CorrGen/scripts/test_ros1_demos.py'
```

回归脚本使用独立 master 11347，并向与 RViz 相同的话题发布两点，检查等待选点、完整路径端点、正向/反向、重复规划、非法点、旧结果清除、显式起点和 reset。详见 [当前交互验证报告](docs/interactive.md)。

手动运行输出在 `results/live/<scene>/run_<time>_<pid>/request_<id>/`，自动回归在 `results/interactive/<scene>/run_<time>_<pid>/`。启动会创建独立目录，不覆盖前一次运行。`request.json` 记录地图快照路径、hash、端点、配置；结果包含 A*、参考曲线、Bézier 控制点、标架轴、系数和指标。失败、超时及取消也保存 `completion.json` 或 `failure.json`。当前目录可用 `rosparam get /corrgen_demo/session_dir` 查询。

话题：`/corrgen_demo/map`、`astar_path`、`reference_path`、`corridor`、`selection`、`status`、`diagnostics`。清空选择：

```bash
rostopic pub -1 /corrgen_demo/reset std_msgs/Empty '{}'
```

[编码约束](coding.md) · [数学与论文差异](docs/method.md) · [SDLP 对比](docs/solver.md)

走廊验收增加了连续参数区间的保守包围检查，覆盖膨胀占据体素及地图边界；默认将机器人按 0.2 m 包围球处理，加 0.05 m 裕量。认证使用带舍入保护的浮点 Bernstein 界，不是形式化区间算术证明。安全性依赖输入地图对环境的覆盖；未观测且没有点的体素按自由空间处理。该输出是几何可行域，尚未包含轨迹优化、动力学、定位误差模型或执行控制。旧离线实验与旧同步节点耗时不代表当前流程，当前结果见 [走廊质量修正](docs/quality_repair.md)。
