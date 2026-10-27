# 历史验证记录：固定参考版本

本页记录交互流程改造之前的实验。当前版本请阅读 [interactive.md](interactive.md)，不能用本页的固定路径结果代替手动选点流程验证。

日期：2026-10-27（Asia/Shanghai）。所有时间为实际单次测量，未取多次平均。

## 环境

- 宿主：Ubuntu 24.04，GCC 13.3，Eigen 3，GLPK 5.0，Release。
- Docker：现有 `ros1_noetic` 容器，镜像 `ros1-noetic:work-saved-gpu`；Ubuntu 20.04、ROS Noetic、GCC 9.4、Eigen 3.3.7、GLPK 4.65。
- 工作空间宿主 `/home/diffbot/ros1_ws/corr_gen`，容器 `/root/ws/corr_gen`。
- 上游版本、阈值、控制点、输入 SHA256 记录于离线结果的 `input.json`；固定配置 `config/*.json`。
- GLPK/SDL 依赖已安装在容器内。原 apt 源返回 502，本次用 `/tmp/corrgen-apt.list` 的 HTTPS TUNA focal 源安装，未改动原源列表。

## 实际 ROS1 launch

三个地图均运行上游未修改的 `read_grid_map` 可执行文件；不是只运行重新绘制的合成障碍。

| 地图 | 原始点数 | A* 路径点 | 横截面约束 | LP生成/求解/复核 ms | 独立 min q | 密采样 min λ | 状态 |
|---|---:|---:|---:|---:|---:|---:|---|
| balls | 12180 | 98 | 12180 | 4471.72 | 1e-05 | 0.111047 | PASS |
| multi_gap | 3585 | 40 | 1530 | 484.01 | 1e-05 | 0.111270 | PASS |
| maze | 4350 | 36 | 1170 | 420.70 | 1e-05 | 0.111098 | PASS |

`solve_ms` 包含约束组装、GLPK、正定认证和残差复核，不含 A*、参考拟合、点云投影和 ROS 发布。当前圆柱场景约 4.47 s，不能据此宣称 5～20 Hz 实时运行。

迷宫选择的是局部通道，交错门选择穿过前两个门的局部路径；不是整张迷宫的全局导航完成率测试。三个 launch 的状态、A* Path、reference Path、MarkerArray 均实际订阅验证通过。自动运行关闭 RViz，未验证容器 GUI 显示。

图中橙色为 A*、青色为光滑参考，紫色为走廊边界。俯视图将不同高度叠加，不能直接当成定高平面安全区域。

![ROS1 A* 实验](../results/ros1/overview.png)

## 数值验证

1. 宿主和 Docker 的 Release CTest 均通过：Chebyshev 值/一二阶导数、完整 q 梯度/Hessian 的有限差分、偏心椭圆边界、负交叉项、采样间非正定反例、三次投影、有限端点、不可行障碍、A* 绕障/非法起点和拟合端点。
2. 从真实 ROS 点云、A* 生成的控制点重新执行独立 C++ CLI；随后 NumPy 重建多项式、枚举全部实根，检查所有横截面约束，而非直接读取 C++ 报告宣布成功。
3. 独立脚本在 2001 个参数上检查最小特征值，C++ 另有 Bernstein 递归区间下界检查。三场景均通过。
4. 离线固定控制点的三地图 × n=3/6 六次运行均通过；n=6 另经独立复核。目录 `results/<scene>/degree_<n>/`。
5. 上游 ROS 图像转换与离线适配器点数一致，排序后最大坐标差小于 1e−6 m（PCL float32 与离线 double 的舍入差）。
6. SDLP 同矩阵对比见 [solver.md](solver.md)：5 维快，20 维显著慢，35 维 30 秒超时。

## 结果边界

这里验证了输入点云中心在连续参数走廊外，使用数值多项式根和浮点容差；不是对连续实体障碍的形式化证书。正定检查也不是有向舍入的形式证明。样本边界到点云最近距离仅作测量，不作成功阈值或机器人净空证明。

地图没有提供连续地板/屋顶障碍，默认圆形 wrapper 半径 3 m，因此部分走廊可伸出地图 z 范围；原始点云测试不应被误读为地图包围盒内的实体安全走廊。需要固定飞行高度/机体半径时，应额外加入这些真实约束。

`outside_finite_sweep` 表示该点对给定有限参考没有横截面交点；不是随机抽稀。非单一投影时一个世界点可产生多个约束。

## 重跑与日志

```bash
bash src/CorrGen/scripts/build_ros1.sh
docker exec ros1_noetic bash -lc 'source /root/ws/corr_gen/devel/setup.bash; python3 /root/ws/corr_gen/src/CorrGen/scripts/test_ros1_demos.py'
```

原始记录：`results/ros1/build.log`、`*_launch.log`、各场景 `ros_metrics.json`、`verified/metrics.json`、`verified/independent_validation.json`。每次运行会覆盖对应场景结果。

宿主复核 ROS 输出：

```bash
for scene in balls multi_gap maze; do
  src/CorrGen/build/corrgen_benchmark \
    src/CorrGen/results/ros1/$scene/cloud.xyz \
    src/CorrGen/results/ros1/$scene/path.xyz \
    src/CorrGen/results/ros1/$scene/verified 6
done
python3 src/CorrGen/scripts/verify_and_plot.py --results src/CorrGen/results/ros1 --ros
```

如果容器以 root 生成共享目录结果，先将该次生成的 `results/ros1` 改为宿主用户所有，或在容器内做复核；不要递归修改其他项目权限。当前结果已恢复为宿主用户所有。
