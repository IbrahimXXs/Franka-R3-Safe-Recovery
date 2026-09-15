# 历史轨迹的离线力预算重评

**这不是低预算闭环实验结果。** 未修改旧轨迹或旧标签；所有阈值均为事后分析候选。

按保存的每步原始腕力合力判断 `force > budget`；力预算同时覆盖参考插入与整个恢复过程。
参考阶段先超限时，原恢复起点不可视为该低预算下可达；恢复超限后的旧轨迹即使最终退出，也不能据此判成功。
无超限但未完成退出的右删失记录仍为未知。扭矩预算沿用各来源设置，配对必须满足各自重放匹配。

## 各预算下的历史筛选

| 来源 | 预算 N | 计划条件 | 参考可接纳 | 直退完成且未超限 | 直退恢复超限 | 匹配直退超限／回正完成 |
|---|---:|---:|---:|---:|---:|---:|
| Forge-Mechanics-FixedSpeed-20260915 | 2 | 24 | 17 | 17 | 0 | 0 |
| Forge-Mechanics-FixedSpeed-20260915 | 3 | 24 | 20 | 20 | 0 | 0 |
| Forge-Mechanics-FixedSpeed-20260915 | 4 | 24 | 20 | 20 | 0 | 0 |
| Forge-Mechanics-FixedSpeed-20260915 | 5 | 24 | 20 | 20 | 0 | 0 |
| Forge-Mechanics-FixedSpeed-20260915 | 6 | 24 | 20 | 20 | 0 | 0 |
| Forge-Mechanics-FixedSpeed-20260915 | 8 | 24 | 20 | 20 | 0 | 0 |
| Forge-Mechanics-FixedSpeed-20260915 | 10 | 24 | 20 | 20 | 0 | 0 |
| Forge-Mechanics-Boundary-20260915 | 2 | 3 | 1 | 1 | 0 | 0 |
| Forge-Mechanics-Boundary-20260915 | 3 | 3 | 2 | 2 | 0 | 0 |
| Forge-Mechanics-Boundary-20260915 | 4 | 3 | 3 | 2 | 1 | 1 |
| Forge-Mechanics-Boundary-20260915 | 5 | 3 | 3 | 3 | 0 | 0 |
| Forge-Mechanics-Boundary-20260915 | 6 | 3 | 3 | 3 | 0 | 0 |
| Forge-Mechanics-Boundary-20260915 | 8 | 3 | 3 | 3 | 0 | 0 |
| Forge-Mechanics-Boundary-20260915 | 10 | 3 | 3 | 3 | 0 | 0 |
| Forge-Mechanics-RateBoundary480-20260915 | 2 | 2 | 0 | 0 | 0 | 0 |
| Forge-Mechanics-RateBoundary480-20260915 | 3 | 2 | 0 | 0 | 0 | 0 |
| Forge-Mechanics-RateBoundary480-20260915 | 4 | 2 | 0 | 0 | 0 | 0 |
| Forge-Mechanics-RateBoundary480-20260915 | 5 | 2 | 0 | 0 | 0 | 0 |
| Forge-Mechanics-RateBoundary480-20260915 | 6 | 2 | 0 | 0 | 0 | 0 |
| Forge-Mechanics-RateBoundary480-20260915 | 8 | 2 | 0 | 0 | 0 | 0 |
| Forge-Mechanics-RateBoundary480-20260915 | 10 | 2 | 0 | 0 | 0 | 0 |

## 匹配历史轨迹的策略分离窗口

下界同时覆盖参考和通过策略的全程峰值；上界为另一策略的全程峰值。区间左闭右开。

| 来源／案例 | 历史通过策略 | 历史超限策略 | 预算区间 N |
|---|---|---|---|
| Forge-Mechanics-FixedSpeed-20260915 / m_d06_fs050_fd050_a00 | straight | realign | [0.312326, 0.313001) |
| Forge-Mechanics-FixedSpeed-20260915 / m_d06_fs075_fd075_a00 | straight | realign | [0.338773, 0.409221) |
| Forge-Mechanics-FixedSpeed-20260915 / m_d12_fs050_fd050_a00 | straight | realign | [0.379849, 0.442636) |
| Forge-Mechanics-FixedSpeed-20260915 / m_d18_fs050_fd050_a00 | realign | straight | [0.434698, 0.464215) |
| Forge-Mechanics-FixedSpeed-20260915 / m_d18_fs100_fd100_a00 | straight | realign | [0.414616, 0.472439) |
| Forge-Mechanics-Boundary-20260915 / m_d18_fs100_fd100_a00 | straight | realign | [0.414616, 0.472439) |
| Forge-Mechanics-Boundary-20260915 / m_d18_fs100_fd100_a1p5 | realign | straight | [3.754587, 4.583499) |

完整逐案例、首次超限采样及 STOP/REALIGN/RETREAT 事件见 [budget_sweep.csv](budget_sweep.csv)；
未知原因、原始来源标签、输入 SHA 与解释见 [budget_sweep.json](budget_sweep.json)。

相同条件在不同来源目录中可能复用同一物理轨迹，不能将全部行数当作独立重复次数。
不同频率改变控制与采样频率，且初态可不同；本表不能证明频率稳健性。
