# 当前交互流程验证

> 历史分析/试验记录；当前实现、默认后端与验收数据以 [工程交付报告](engineering_delivery.md) 为准。

日期：2026-10-27。此报告对应手动选点版本，替代早期固定起终点 demo 的交互结论。

## 实现依据

已查阅本机 `ros1_ws/firi_benchmark/src/FIri_Benchmark/gcopter`：

- `src/global_planning.cpp` 的地图初始化和 `targetCallBack`：地图就绪后接收两次 `/move_base_simple/goal`，下一次选择开始新请求。
- `config/global_planning.rviz` 的 SetGoal/SetInitialPose/PublishPoint 配置。
- `launch/global_planning.launch` 的原版 kr_param_map 结构地图、密度和 RViz 加载流程。

这里复用交互方式，路径搜索为本项目 C++ 26 邻域 A*，不调用 FIRI 或轨迹优化器。保留三张图像地图，并新增 sparse/medium/dense 三个结构化地图入口。

## 50×50×5 m 场景更新

`global_planning.launch` 对齐 FIRI 的结构化地图参数：原点 (-25,-25,0)，分辨率 0.25 m，种子 1024，w1/w2/w3 为 0.10/0.50/2.00。`sparse.launch`、`medium.launch`、`dense.launch` 和旧 `_demo` 名称均使用此配置。RViz 配置 `rviz/structure.rviz` 使用浅色背景、50 格地面、0.25 m Boxes 和 Z 高度彩虹着色；图像地图继续使用独立配置。

Docker 三档加载回归通过，单次加载点数为 sparse 27,072、medium 52,941、dense 104,472；加载后均等待用户两点输入，无自动规划。上游多面体的 `randomInit` 使用 `std::random_device`，所以固定 seed 仍可能产生不同点云。以下路径结果使用每次请求保存的真实点云，而非假设点数相同。

当前跨地图实测两点为 (-24.125,-24.125,0.875) 与 (23.875,23.875,4.375)：稀疏正反向通过，分别 52.50 s / 23.99 s；中等正向通过 26.35 s，中等反向报告 `LP failed: return=5, status=1`，该请求未生成有效走廊。稠密正向报告 `LP primal residual failed`，残差复核拒绝发布该结果，故未继续反向。以上是当前大场景求解稳定性限制，不能视为全场景规划回归通过。旧 request 文件可能来自历史运行，必须结合本次日志及 run 状态判断，失败运行不使用历史指标。

构建：`catkin_make -DCMAKE_BUILD_TYPE=Release -DCORRGEN_BUILD_ROS1=ON -j2`；Docker Release CTest 1/1 通过。地图源仍为 kr_param_map commit `12dae849e05f28c9fbbcab2b3f18ba793fb536e2`，未修改上游生成器。

当前桌面已启动 `medium.launch`，使用独立 master 11348；ROS 状态检查为 `WAITING_FOR_START`。关闭此 RViz 后可运行其他密度。GUI 视觉布局未完成截图复核；自动测试直接检查真实地图话题及 RViz 输入话题。

## 历史小地图实测（结构图为此前 20×10×3 m 配置）

环境：Docker `ros1_noetic`，ROS Noetic / Ubuntu 20.04 / GCC 9.4 / GLPK 4.65，Release；测试 master 11347，与既有 master 分离。

测试程序向真实 RViz 输入话题发布两点，未在 launch 内预设起终点。每个场景验证：加载后不自动规划 → 非法点拒绝 → 起点后等待目标 → 目标触发全程生成 → 交换两点反向重跑 → 新请求清除旧结果 → 显式 initialpose → reset。

| 场景 | 方向 | A* 点数 | 全程参考段数 | 全流程 ms | 两端最大误差 m | 结果 |
|---|---|---:|---:|---:|---:|---|
| balls | 正向 | 98 | 51 | 6743.4 | 3.7e-15 | PASS |
| balls | 反向 | 98 | 51 | 3680.5 | 2.2e-16 | PASS |
| multi_gap | 正向 | 97 | 124 | 3413.8 | 7.3e-15 | PASS |
| multi_gap | 反向 | 97 | 125 | 2722.1 | 8.9e-16 | PASS |
| maze | 正向 | 95 | 53 | 1491.9 | 0 | PASS |
| maze | 反向 | 95 | 54 | 1485.2 | 0 | PASS |
| sparse | 正向 | 95 | 54 | 1129.2 | 4.4e-16 | PASS |
| sparse | 反向 | 95 | 55 | 1113.1 | 3.8e-16 | PASS |
| medium | 正向 | 95 | 54 | 1513.2 | 4.4e-16 | PASS |
| medium | 反向 | 95 | 107 | 1292.9 | 1.8e-15 | PASS |
| dense | 正向 | 95 | 55 | 2261.5 | 1.8e-15 | PASS |
| dense | 反向 | 95 | 56 | 1842.6 | 2.5e-16 | PASS |

这些是单次实测，包括 A*、几何参考构造、点云投影、LP 和输出准备，不是实时性能保证。时间会受场景、方向、构建环境影响。

交错门起终点为约 `(0.3,2.1,1.5)` 到 `(19.5,8.1,1.5)`，迷宫为 `(0.5,0.5,1.5)` 到 `(19.3,9.3,1.5)`，均覆盖跨地图路径。结构地图选择约 `(0.5,0.5,0.7)` 到 `(19.3,9.3,2.3)`，包含高度变化。反向交换两点，不依赖 x 递增。

所有参考段通过递归 Bézier 包围盒与占据体素检查；所有有效点云横截面交点进入全局走廊 LP，随后通过残差与连续区间正定数值检查。A* 与参考路径的第一、最后一点均等于该次用户输入，未用局部截断掩盖拟合失败。

## 单元测试

宿主 GCC 13 和 Docker GCC 9 的 Release CTest 都通过。覆盖：

- Chebyshev 值、端点导数、完整梯度与 Hessian 差分、偏心椭圆边界。
- 负交叉项、采样间非正定、不可行约束、多个横截面投影。
- 分段参考接头 C⁰/C¹/C²、统一标架连续、竖直段、反向三维起终点。
- 三维体素 A* 绕障/无路，以及 Bézier 穿墙拒绝。

## 操作入口

```bash
source /root/ws/corr_gen/devel/setup.bash
roslaunch corrgen_cpp medium.launch
```

等待 `WAITING_FOR_START` 后连续两次使用 **2D Nav Goal**。也可先 **2D Pose Estimate** 再 **2D Nav Goal**。结构地图默认按箭头角度设置高度；需要定高时使用 `height_mode:=fixed height:=1.5`。没有轨迹优化。

复现全部六场景测试：

```bash
docker exec ros1_noetic bash -lc 'source /root/ws/corr_gen/devel/setup.bash; python3 /root/ws/corr_gen/src/CorrGen/scripts/test_ros1_demos.py --scenes balls multi_gap maze sparse medium dense'
```

结果：`results/interactive/<scene>/interaction_checks.json`、`request_1/metrics.json`、`request_2/metrics.json` 和 `*_launch.log`。手动操作单独写入 `results/live/<scene>/`。

## 范围

没有 GUI 鼠标自动化测试；自动回归验证的是与 RViz 相同的输入消息、状态转换和可视化输出。实际 GUI 另外启动并检查了地图与等待状态。

参考中心线 C²、统一标架 C¹，局部坐标 q(u,w) 为全局多项式；没有将世界坐标管道接头描述为 C²。有限点云排除仍不等于连续障碍实体/有限机体的安全证书，地图外边界也不是默认 wrapper 的严格约束。
