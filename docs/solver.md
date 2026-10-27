# LP 维度与 real_planner SDLP 选型

> 历史分析/试验记录；当前实现、默认后端与验收数据以 [工程交付报告](engineering_delivery.md) 为准。

## 是否低维

三维偏心、旋转椭圆需要 a,b,c,d,e 五组次数 n 的系数，总变量 `D=5(n+1)`：

| n | D |
|---|---:|
| 0 | 5 |
| 3 | 20 |
| 6 | 35 |
| 9 | 50 |
| 15 | 80 |

变量相对点云约束确实少，但 20～80 维不能等同于几何 LP 常见的 2～4 维。默认约束数为 `4N + N×wrapper_angles + 横截面交点数`。不是只优化单个 2×2 矩阵的 3 个数。

## real_planner 实现

检查路径：

- `real_planner/src/General-Planner/src/Planner/general_planner/include/utils/optimization/sdlp.h`
- `real_planner/src/General-Planner/src/Planner/general_planner/src/utils/sdlp.cpp`

该实现是 Seidel 随机递增、递归降维 LP。接口是编译期维度模板 `linprog<D>(c,A,b,x)`，约束形式 `Ax≤b`。现有 cpp 末尾仅显式实例化 `<1>` 和 `<4>`；直接调用 `<20>` / `<35>` 会缺少定义/链接实例。可以将定义暴露到实例化位置，或添加显式实例，但仍需考虑维度带来的递归代价。

算法背景参见 [Seidel 原论文](https://people.eecs.berkeley.edu/~jrs/meshpapers/SeidelLP.pdf) 和 [MIT 算法课程复杂度说明](https://courses.csail.mit.edu/6.856/current/Notes/n21-lp.html)。固定维度时对约束数近似线性，不意味着维度增大后仍快。

## 本次实测（2026-10-27，宿主机 Release，单次样本）

`compare_sdlp.cpp` 直接包含上述 real_planner cpp，保留原实现与随机排列机制，不修改外部文件。构造同一组 41 截面、16 个障碍方向和四侧对角占优的 LP，总计 820 条约束；两个求解器读取相同矩阵。此对比是求解器微基准，不是三张地图的总运行时间。

| n / D | GLPK | real_planner SDLP | 结果 |
|---|---:|---:|---|
| 0 / 5 | 3.49 ms | 0.045 ms | 目标值一致，最大约束违反 2.22e−16 |
| 3 / 20 | 17.48 ms | 2013.91 ms | 目标值一致，最大约束违反 8.77e−14 |
| 6 / 35 | 20.42 ms | >30 s | 外部 timeout，退出码 124，未得到解 |

数据仅代表当前单次输入/环境，不是通用性能保证。SDLP 包含随机排列，重复运行时间可能变化。

结论：可以用于极低维常数截面；本项目 3/6 阶全局参数走廊默认使用 GLPK。实验程序作为可选 target 保留，未把 SDLP 暗中设为生产后端，也没有把每截面单独求解冒充全局可微走廊优化。

## 复现

```bash
cmake -S src/CorrGen -B src/CorrGen/build \
  -DCORRGEN_REAL_PLANNER_DIR=/home/diffbot/ros1_ws/real_planner/src/General-Planner/src/Planner/general_planner
cmake --build src/CorrGen/build --target corrgen_compare_sdlp -j2
timeout 30 src/CorrGen/build/corrgen_compare_sdlp 0
timeout 30 src/CorrGen/build/corrgen_compare_sdlp 3
timeout 30 src/CorrGen/build/corrgen_compare_sdlp 6
```

若使用当前宿主解包的 GLPK，先按 README 设置构建前缀和运行时 LD_LIBRARY_PATH。日志在 `results/sdlp/degree_*.log`。
