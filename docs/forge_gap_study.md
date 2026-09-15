# 间隙与插入中倾角扰动实验

本实验使用原有 FORGE 刚体接触环境，只改变孔径和插入过程中的倾角命令。
它不模拟塑料形变或损伤。所有间隙均指 **单边径向间隙**，不是孔销直径差。

## 固定实验定义

| 参数 | 设置 |
| --- | --- |
| 插销直径 | 7.986 mm |
| 名义径向间隙 | 0.507、0.4、0.3、0.2、0.1 mm |
| 孔径 | 9.000、8.786、8.586、8.386、8.186 mm |
| 触发位置 | 实际孔内深度首次达到 6、10、14 mm，即目标20 mm的30%、50%、70% |
| 扰动 | 1秒 smoothstep 过渡，随后保持；触发后不再受深度停滞影响 |
| 完整倾角矩阵 | pitch ±0.5°、±1°、±2°、±4° |
| 插入与恢复 | 原有8秒插入、1秒终止保持、0.25秒恢复停止、6秒退出、2秒可选对齐 |
| 力/力矩预算 | 原有20 N/1 N·m原始腕部模长；不是机械臂硬件能力上限 |

倾角触发读取已完成的物理采样，下一步开始施加命令。CSV同时保存触发时间、深度、
物理步号、命令倾角和实际倾角。未触发、未完成过渡均保留为独立状态。

孔径通过修改真实USD碰撞网格实现：只移动内孔和倒角顶点，并屏蔽原来的已烘焙
碰撞缓存。0.507 mm基准继续使用原始USD。圆周多边形及网格量化会令有效最小间隙
略小于名义值；`scene.json`记录测量值和哈希。数值重叠筛查使用有效间隙的25%。

## 案例与运行

`experiments/forge_gap_pilot.json`包含23例：五个对齐对照，加三个间隙
（0.507/0.2/0.1）×三个触发位置×两个正pitch角度（0.5°/2°）。
`experiments/forge_gap_full.json`包含125例，前23例与pilot一致。
完整矩阵不自动启动。

协调器按间隙分成五个子实验，每个仿真进程只使用一个孔径。
它先运行全部对齐对照；只对对照通过数值、插入、预算、夹持和退出检查的间隙
继续执行倾角实验。未通过的条件保留结果并标记为需要检查，不删除或替换物理失败。

```bash
# 只生成并验证计划，不启动仿真
python3 simulation/run_gap_study.py --output-dir outputs/Forge-GapTilt-Pilot --prepare-only

# 五档对齐基线；已生成的目录用 --resume
python3 simulation/run_gap_study.py --output-dir outputs/Forge-GapTilt-Pilot --resume --phase controls

# 继续23例pilot，仍受各间隙对齐对照门槛约束
python3 simulation/run_gap_study.py --output-dir outputs/Forge-GapTilt-Pilot --resume --phase all

# 小批继续：每个可运行间隙最多增加一次尝试
python3 simulation/run_gap_study.py --output-dir outputs/Forge-GapTilt-Pilot --resume --max-new-attempts 1

# 后续扩大至完整125例时使用新目录；不能覆盖或改变pilot的冻结计划
python3 simulation/run_gap_study.py --case-plan experiments/forge_gap_full.json \
  --output-dir outputs/Forge-GapTilt-Full --prepare-only
```

子仿真使用 `forge_study.sh` 和 `franka-safe-recovery` Conda环境，沿用安装版本的
原生120 Hz。`--physics-hz`是明确的数值诊断覆盖，不应悄悄混入主实验。
日志在批目录的`logs/`，`batch.json`记录子命令、耗时、退出码、进程号与状态。
子目录的`study.json`、`source/`和`geometry/`记录可恢复运行的冻结实验来源。
恢复前还会检查旧子进程的身份和输出锁，避免协调器退出后重复启动同一实验。

运行过程中可只读查看已完成案例：

```bash
python3 simulation/summarize_gap_study.py --batch-dir outputs/Forge-GapTilt-Pilot

# 将汇总表写入独立目录；使用仿真Conda环境并添加 --plots 可同时生成图表
python3 simulation/summarize_gap_study.py --batch-dir outputs/Forge-GapTilt-Pilot \
  --output-dir outputs/Forge-GapTilt-Pilot-review
```

该汇总按计划案例计数，并区分未发生的事件、仅记录的事件、未完成参考轨迹和恢复未知。
`case_summary.csv`给出每个案例的最新尝试，`attempt_summary.csv`保留所有重试，
`checkpoint_summary.csv`把触发、实际倾角与两种恢复策略的结果放在同一行。
原有采集器允许每个数值无效案例最多5次尝试，因此实际尝试次数可能超过23；
这类重试保持同一案例与种子，不能当作独立统计样本。

## 检查点与结果含义

每条参考轨迹保存`pre_tilt`、`ramp_complete`、`first_stall`和`terminal`事件。
默认仅重放`first_stall`和`terminal`，分别测试`straight`和`realign`。
没有达到的事件、仅记录未测试的事件、重放不匹配均为未知。
首停滞使用原有0.5秒窗口：命令前进至少0.5 mm，实际前进不足0.1 mm。
确认时刻不回溯到窗口起点。
这个原始检测器也可能在销尚未进孔时触发，它检测的是运动跟踪停滞，不能直接
等同于孔内卡滞。分析时应另外检查事件深度及整个检测窗口是否已进孔，并结合接触
负载判断。若原始`first_stall`发生在孔外，其恢复测试只属于该孔外状态；后来发生
的首个孔内停滞没有因此获得恢复标签。终止状态测试仍按各自的前缀和重放证据解释。

现有恢复过程先保持达到的手部位姿0.25秒，可能卸载插入负载。本实验保留该策略；
因此结论针对这套恢复策略。停止、对齐和实际退出阶段必须分开分析。
`realign`同时将销的XY位置移回孔中心并回正姿态，不能将其收益完全归因于角度修正。
恢复CSV中的`command_tilt_deg`表示该策略的目标倾角；实际逐步姿态以`tilt_deg`为准。

原有插入成功定义是连续0.2秒达到至少19.5 mm，并保持夹持；不要求姿态误差为零。
停滞表示历史窗口内运动受阻，可能与后续插入成功同时出现。数值有效性、插入预算
以及插入成功分别保留，不能用其中一项替代其他项。

| 文件 | 每行代表 |
| --- | --- |
| `cases.csv` | 一个计划案例，含名义尺寸、间隙、扰动设置 |
| `trajectories.csv` | 一次插入尝试及达到的状态、插入与数值结果 |
| `checkpoints.csv` | 一个事件状态、腕部信号、重放标签 |
| `recovery_probes.csv` | 一个检查点的一种恢复策略，包括失败和未知 |
| `comparison.csv` | 同一事件的两种策略对照，缺失策略不隐藏 |
| `insertion.csv`、`*_recovery.csv` | 原始每步记录 |

完整恢复的腕部峰值用于操作预算判断。退出阶段另报腕部合力、原始腕部世界Z分量
绝对值、向下接触阻力`max(0,-fz)`、完整100 ms窗口均值峰值、实际/命令退出位移及接触功。
世界Z等于当前固定孔的轴向；原始腕部反力不是已校准外力估计，接触力与腕部力保持区分。
未进入某阶段的指标为空而非0。超限、滑移、超时导致的未完成恢复标记为截断；
其峰值不是完整退出所需最大力。完整清孔成功才代表退出完成。

`Y_R_tested=1`表示至少一种合格策略成功；0要求两种合格匹配策略均失败；其他为未知。
插入停滞、单帧尖峰、滑移和数值无效都不能单独称为物理卡死。

## 验证目标

1. 间隙和扰动是否改变接触、实际姿态及插入停滞。
2. 停滞后，实际退出阶段的轴向阻力和持续负载是否高于同间隙对齐基线。
3. 匹配起点下，调整姿态是否改善完整恢复过程。
4. 仅在两种合格恢复均失败时，记录“既定策略库与预算内恢复失败”。

保留所有未知、失败和截断记录。重复关键案例以及邻近正常案例，必要时单独检查
时间步和碰撞精度。小批筛查不能建立普遍的卡死规律或实物安全保证。
模型训练前按案例/接触历史分组，并检查不同倾角路径的共同对齐前缀。

嵌入长度、两点接触、摩擦和柔顺支承的理论依据与后续分析变量，见
[销孔卡滞力学笔记](peg_hole_mechanics_zh.md)。

## 独立时间步诊断

`experiments/forge_gap_numeric_diagnostic.json`冻结首个候选条件：0.2 mm径向间隙、
30%实际深度触发、2°正pitch。下列命令在新目录复核该条件，不混入主批统计：

```bash
bash forge_study.sh --headless --mode collect \
  --case-plan experiments/forge_gap_numeric_diagnostic.json \
  --output-dir outputs/Forge-GapTilt-Diagnostic-240Hz \
  --physics-hz 240 --max-new-attempts 1
```

物理步进与控制命令更新在此实现中使用相同频率，诊断会同时加密两者。对比完整
插入、连续直接退出、实际位移和持续负载；重放不匹配仍按未知处理。两种频率的
一致性检查不等于已经完成SDF空间分辨率收敛或实物材料标定。
