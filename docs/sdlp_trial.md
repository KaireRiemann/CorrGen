# real_planner SDLP 真实地图试验

> 历史分析/试验记录；当前实现、默认后端与验收数据以 [工程交付报告](engineering_delivery.md) 为准。

日期：2026-10-27。按用户要求接入并试验 SDLP，未终止或替换用户正在运行的 dense 节点。

## 接入内容

- `Options::solver` 和 ROS/launch `solver:=glpk|sdlp`，保留 glpk 默认值。
- 编译时 `CORRGEN_REAL_PLANNER_DIR` 指向 real_planner/general_planner；adapter 包含原版 cpp，显式使用5、20、35维模板，对应0、3、6阶。未修改外部仓库或移除版权说明。
- 相同的35维全局系数、目标、正定/wrapper/全部点云约束，原来的 `Ax>=b` 转换为 SDLP 的 `-Ax<=-b`，不是每截面单独解LP。
- 共用原始LP装配后转换为SDLP矩阵，这是为保证第一次对比等价而保留的装配方式；不是最终内存优化方案。
- 所有返回解重新计算原始行残差，再检查障碍残差及连续正定。结果 metrics 加入 solver 字段。
- 不支持的阶数、后端未编译、非有限返回和残差失败均明确报错。
- `corrgen_replay` 与 `scripts/compare_saved_lp.py` 在独立子进程中读取保存数据并设置墙钟预算；超时只杀死该测试子进程。ROS节点仍同步调用，原版SDLP没有内部超时/取消接口。

## 真实数据结果

Docker ROS Noetic/GCC9 Release、GLPK4.65。测试时用户已有GLPK求解仍在运行，部分对比进程并行，因此这些是单次预算试验，不是隔离环境性能排名。SDLP使用上游随机排列，结果和时长可能波动。

| 数据 | 阶数/维数 | 障碍约束 / 总约束 | 后端 | 结果 |
|---|---|---|---|---|
| sparse request_1 | 6 / 35 | 37,679 / 40,507 | SDLP | 60秒未返回，TIMEOUT |
| 相同 sparse | 6 / 35 | 同上 | GLPK | 本次60秒未返回，TIMEOUT |
| 当前 dense 缓存 | 6 / 35 | 286,779 / 289,607 | SDLP | 60秒未返回，TIMEOUT |
| 相同 dense | 0 / 5 | 286,779 / 289,607 | GLPK | 20秒未返回，TIMEOUT |
| 相同 dense | 0 / 5 | 286,779 / 289,607 | SDLP | PASS，generate 147.669ms，进程全程0.868s |

0阶dense：目标2823.2001506035，最小障碍q=9.9999999998435e-6，原始全部行残差与连续正定检查通过。generate时间包含LP装配、矩阵转换、SDLP及校验，不是只计算SDLP函数用时。全进程时间还包括读取、重建参考和投影。

由于0阶GLPK超时，不能宣称这份真实dense输入上已与GLPK核对最优目标；独立的小型5维测试完成两后端目标一致验证。

结论：SDLP在5维、约29万约束上展现明显实用性；直接替换当前35维全局模型没有在60秒预算内得到结果。降低到0阶改变了模型自由度，不是6阶模型的等价加速。后续可研究更低阶参数化或工作集策略，不能直接把这次0阶耗时写成6阶性能。

## 验证

Docker编译通过，CTest 1/1通过，新增SDLP检查包含：小型5维目标与GLPK一致、原始障碍残差、不可行输入拒绝、不支持阶数拒绝。ROS `roslaunch --dump-params ... dense.launch solver:=sdlp` 确认参数传入。35维完整地图试验均超时，尚无35维成功走廊验证。

## 来源与复现

源文件：`real_planner/src/General-Planner/src/Planner/general_planner/src/utils/sdlp.cpp`，SHA256 `42721d980fc1cec3a40279350d8dd577804d416059f2f086c1373cb6fc976d7c`；头文件SHA256 `86846e094e4834f5ebdaed8bed6826cb6d3c5969a5016b6dfb66ef4ed350d739`。上游版权声明保留在外部原文件中。

启用编译与launch命令见README。限时复现：

```bash
cd /root/ws/corr_gen/src/CorrGen
python3 scripts/compare_saved_lp.py results/performance_dense \
  --binary /root/ws/corr_gen/build/CorrGen/corrgen_replay \
  --solvers sdlp --degree 6 --seconds 60 --output results/sdlp_real/dense
# 将degree改成0为常数截面诊断，不能与6阶当作相同问题比较
```

原始日志/JSON：`results/sdlp_real/sparse`、`sparse_glpk`、`dense`、`dense_constant`。数据分别为 `results/interactive/sparse/request_1` 和 `results/performance_dense`，完整真实点云和A*已保存。重放几何配置固定50×50×5m、origin(-25,-25,0)、resolution0.25、inflation0。
