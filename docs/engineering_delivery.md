# 工程实现、验收与边界

> 本文记录上一版实现。统一缩小策略现已移除，默认后端、预算、质量及完整耗时以 [quality_repair.md](quality_repair.md) 为准。

2026-10-27。本次实现保留全程 A* 和 6 阶、35 变量全局参数走廊，完成求解加速、参考净空、实体体素检查、进程预算及 ROS 手动选点流程。未加入轨迹优化。

## 已实现的处理

1. **统一 LP 接口**：`solver.hpp/solver.cpp` 表示 min cᵀx、Ax≥b、自由变量。GLPK、HiGHS、ProxQP、可选 real_planner SDLP 读取同一模型。ProxQP 的 H=0，未加改变目标的二次惩罚。HiGHS 1.10.0、ProxSuite 0.6.3；求解全部为 C++。
2. **工作集**：固定正定/wrapper 行保留；16 个参数桶×8 个方向桶初始化近点；每轮全量扫描全部横截面交点，按违反程度及参数桶加入最多 128 行。工作集只增不删。正行缩放后求解，在原单位检查残差，障碍裕量 1e-5、允许残差 1e-6。未使用任意远点裁剪。
3. **可行保底**：解析常数截面作为 6 阶模型中的初值；全量验收后才交给回调。受限解有违反时可向初值做凸组合修复；修复结果再次全量验证。连续正定不通过则不接受。求解器最优、修复可行和预算保底分别标记。
4. **几何安全层**：占据体素按 robot_radius+safety_margin+inflation 保守膨胀；地图边界同时内缩。A* 不允许穿占据角；可见性简化和参考圆滑直接检查带 reference_clearance 的 Bézier 包围盒，不能以“在自由格子里”代替净空。
5. **整个管道验收**：对每个参数区间，用 Bernstein 系数界给出最小特征值下界和偏移项上界，从而界定整个截面到参考线的距离；参考 Bézier 控制点包围盒外扩该界，检查膨胀体素及边界。不确定时递归细分，深度耗尽拒绝。管道不通过可按统一因子缩小，再次验证点约束、正定及实体包围；记录 shrink_factor，绝不把缩小解称为原 LP 最优。
6. **全投影并行**：默认 4 线程按输入点分块，仍枚举所有横截面实根。约束复核使用只算 q 的 Clenshaw 函数，不再计算无用梯度/Hessian。
7. **进程调度**：ROS 主节点只负责输入、地图快照、状态和展示；独立 `corrgen_worker` 完成重计算。reset、新起点、地图变化及预算到期回收进程组，无 detached 求解线程。请求编号、地图版本、hash 和独立会话目录保证不覆盖旧实验，不发布过期结果。
8. **诊断**：每次请求保存 request/progress/candidate/metrics/completion，失败另存 failure；先发布已验证可行结果，随后优化。ROS diagnostics 包含规模、质量及验证范围。最终退出后重新读取原子 candidate，避免进程退出与结果读取的竞态。记录首个发布时间、总墙钟及 worker 峰值 RSS。
9. **显示成本**：最多 401 个显示截面，每环 32 边；显示采样不参与安全验收，也不改变连续参考或全量约束。Bézier 控制点完整保存。

ProxQP 与 HiGHS 均通过六张地图正反向各一次、合计 24 次端到端请求回归。默认仍采用与论文相同的 ProxQP 后端；HiGHS 可通过 `solver:=highs` 使用。在下面固定 dense LP 上 HiGHS 工作集更快，不代表所有路径都更快。

## 同一个 LP 的独立校核

输入 `results/performance_dense`：104508 个点、208 段参考、286779 条横截面约束，加 2828 条固定约束，总计 **289607 行、35 变量**。没有改阶数、目标或约束。纯 LP 重放不包含本次机体净空扩展，以确保与旧数据是同一个问题。

| 方法 | generate 时间 | 实际求解行数 | 轮数 | 目标值 |
|---|---:|---:|---:|---:|
| ProxQP 工作集，3 次 | 106.7–108.6 ms | 2932 | 3 | 1632.3741945472 |
| HiGHS 工作集，3 次 | 约 56 ms | 2932 | 3 | 1632.3741945471 |
| HiGHS 全矩阵，1 次 | 3461.0 ms | 289607 | 1 | 1632.3741945472 |

全部返回求解器最优状态，通过完整原始约束和连续正定检查；目标差约 1e-10。这验证了工作集没有靠遗漏约束得到虚假加速。HiGHS 工作集相对其全矩阵约 62 倍加速，属于相同输入/模型的比较。重放墙钟仍约 0.73–0.79 s，主要是串行全投影和加载；不能把 56 ms 写成完整交互耗时。

原始运行参数及输出：`results/engineering/working_set_comparison.json`；复现：

```bash
# Docker ros1_noetic 内
source /root/ws/corr_gen/devel/setup.bash
python3 /root/ws/corr_gen/src/CorrGen/scripts/benchmark_working_set.py
```

旧 SDLP 35 维全矩阵 60 s 超时及官方实现差异见 [upstream_solver_audit.md](upstream_solver_audit.md)。作者默认用 ProxQP，并通过地面/投影/范围筛选把示例压到约 3–4 千行；不是直接用 SDLP 解当前的 29 万行。当前工作集机制是工程扩展，不属于作者原代码。

## 最终 ROS 交互回归

环境：AMD Ryzen 7 9700X，Docker ROS Noetic，GCC 9，Release，单规划 worker、4 个投影线程。每张真实 kr_param_map 地图正反各一次；启动先等待手动选点，测试脚本随后模拟 RViz 同样的输入话题。没有轨迹优化。以下是 ProxQP 默认配置最终验收的 12 次请求，完整墙钟包含结果传输/主节点处理，区别于 worker 内部计时。

| 地图 | 首次已验证发布 | 完成墙钟 | 验收后最小采样半轴 |
|---|---:|---:|---:|
| sparse | 250–254 ms | 357–415 ms | 8.8 cm |
| medium | 244–286 ms | 362–432 ms | 8.6–8.7 cm |
| dense | 724–738 ms | 839–886 ms | 7.7–8.6 cm |
| balls | 39–64 ms | 102–127 ms | 4.6 cm |
| multi_gap | 56–70 ms | 108–214 ms | 4.9–7.0 cm |
| maze | 45–50 ms | 129 ms | 3.8–7.7 cm |

HiGHS 同样通过 12 次交互请求：sparse 273–343 ms、medium 309–381 ms、dense 723–801 ms，图像地图 83–119 ms。其结构化地图由上游重新生成，与 ProxQP 不是逐点相同地图，所以只用于功能验收；严格后端比较采用上一节的固定输入。

这只是小规模回归样本，不能据此承诺任意选点 p95/p99 或硬实时上限。结构化地图原点 (-25,-25,0)、50×50×5 m、0.25 m 分辨率；图像地图 20×10×3 m、0.2 m。默认 robot_radius=0.2 m、safety_margin=0.05 m；结构化地图 reference_clearance=0.1 m，图像地图=0.05 m，避免要求等于半个体素边长的净空导致边界退化。

“最小采样半轴”用于描述质量，不是连续最小净空证书。半轴是**机器人中心允许区域**的尺寸，机器人半径已体现在障碍膨胀中。最终输出均明确标记 FEASIBLE_GEOMETRY_SHRUNK：点模型最优管道还不能直接当作实体体素安全管道。小半轴和全局缩放仍说明几何层偏保守；它是后续质量优化重点。

回归覆盖等待地图、非法点、全程端点精确保留、反向路径、不同高度、下一轮清除、2D Pose Estimate 重设起点和 reset。原始数据 `results/engineering/final_regression.json`（ProxQP）及 `results/engineering/highs_regression.json`（HiGHS）引用每个会话目录，地图快照及 request.json 可复现，不依赖 random_device 不确定的上游种子。

```bash
python3 /root/ws/corr_gen/src/CorrGen/scripts/test_ros1_demos.py \
  --scenes sparse medium dense balls multi_gap maze
# 独立的 HiGHS 端到端回归，输出至 results/interactive_highs
python3 /root/ws/corr_gen/src/CorrGen/scripts/test_ros1_demos.py \
  --solver highs --scenes sparse medium dense balls multi_gap maze
```

## 取消与预算验收

使用已保存真实 dense 地图，在独立 master 11349 测试：

- reset 取消约 5.1 ms；地图变更取消约 16.4 ms。
- 取消后没有旧请求重新发布，节点下无遗留 worker 子进程。
- 1 ms 极短预算在约 14.5 ms 的下一次计时回调中终止；没有可行候选，明确 FAIL TIMEOUT。
- SDLP 1000 ms 预算：约 776 ms 发布已验证保底，约 1009 ms 终止 SDLP，返回 FEASIBLE_BUDGET；不是最优解，不是数学不可行。

复现 `scripts/test_worker_lifecycle.py`；结果 `results/engineering/lifecycle/report.json`。ROS 计时器周期 20 ms，加 OS 调度与 JSON/可视化开销，所以预算是受监控软截止时间。

## 构建、目录及依赖

```bash
# 宿主机
cd /home/diffbot/ros1_ws/corr_gen
bash src/CorrGen/scripts/build_ros1.sh ros1_noetic
# 可选 SDLP：catkin_make 再加
# -DCORRGEN_REAL_PLANNER_DIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner
```

`setup_solvers.sh` 把依赖放到工作空间 `.deps`，不修改系统 Python；pip 只用于获取固定版本 C++ 头文件，运行时不调用 Python 求解。ROS 适配层另需 nlohmann-json3-dev。核心库不依赖 ROS、JSON 或进程管理；求解适配、几何、worker、ROS 各自分离。

- 作者代码 commit：`5a0f48a0e02b4f0051df652e33ce21926171b701`。
- kr_param_map commit：`12dae849e05f28c9fbbcab2b3f18ba793fb536e2`，未改地图生成算法。
- HiGHS commit：`fd8665394edfd096c4f847c4a6fbc187364ef474`（v1.10.0）。
- ProxSuite：0.6.3。
- 编码约束已同步至 [coding.md](../coding.md)。

CTest 通过：解析一二阶导数、偏心椭圆、正定、投影、不可行输入、A* 无路、体素边界、连续管道拒绝、参考净空、全量/工作集目标一致、ProxQP/HiGHS 基准一致、SDLP 低维一致及预算保底。

## 剩余工程边界

- 全量投影在长 dense 路径上仍占主要时间；当前并行化，不做无证明的远点裁剪。后续应基于已认证包络和空间索引筛选，或安全减少参考段。
- 当前每轮重建后端对象，只提供 x 初值，不声称跨轮/跨请求复用基、因子分解或投影缓存。
- 管道包围使用球界和 AABB，统一缩放保守。若需要更宽的局部走廊，可研究方向相关包围或局部 B-spline 系数；那是表示/可行域变化，应另做同场景质量比较。
- 连续包围采用 long double Bernstein 转换及舍入保护，**不是严格向外舍入的形式化区间证明**。未知且无点的体素按自由空间；地图必须足够覆盖真实障碍。
- 当前只按球包围机器人，无姿态相关机体模型。世界坐标管道不承诺 C² 标架或无自交；局部 q 的导数与几何参考不能替代动力学/控制验收。
- 求解器容差内最优不等于严格对偶间隙证明；本次额外用 HiGHS 全矩阵同目标结果交叉校核。
