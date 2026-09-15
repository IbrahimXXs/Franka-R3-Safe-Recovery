# 定深度、摩擦与倾角机理实验

本实验使用独立的 `Forge-mechanics-v1` 计划：先对齐到指定深度，确认实际位置稳定，再保持深度命令并施加倾角，最后测量退出过程。目标是检查实际接触位置、法向载荷、摩擦和姿态变化如何影响退出阻力。

本文说明已实现的协议、数据含义和使用方式，不预设实验结果。场景仍是 **FORGE/Panda 刚体模型**；没有塑性变形、磨损、损伤累积或新毛刺生成。机理假设及适用范围另见 [嵌入长度、摩擦与销孔卡滞](peg_hole_mechanics_zh.md)。

本实验沿用 FORGE 的销与机器人刚体禁用重力配置。记录的是该动力学设置下的仿真腕部反力，不是已标定的实物原始拔出拉力，也不等同于对真实传感器做软件重力补偿。该事实由配置继承与执行路径静态核对，原始场景快照没有记录运行时重力开关回读。销质量约 0.019 kg；启用重力可能同时改变接触与控制轨迹，不能把全部曲线简单统一加上销自重。

## 1. 24 个计划条件

冻结计划位于 [forge_mechanics_pilot.json](../experiments/forge_mechanics_pilot.json)。

| 因素 | 取值 |
| --- | --- |
| 目标实际插入深度 | 6、12、18 mm |
| 有效接触对摩擦系数 `(μs, μd)` | `(0.5, 0.5)`、`(0.75, 0.75)`、`(1.0, 1.0)`、`(1.0, 0.5)` |
| 目标 pitch 倾角 | 0°、+2° |
| 固定单边径向间隙 | 名义 0.2 mm |
| 孔型与总深度 | 25 mm 盲孔 |

共 `3 × 4 × 2 = 24` 个条件。默认完整运行先执行全部 12 个 0° 对照，再执行 12 个 2° 条件。使用 `--case-ids` 做小批检查时可以先完成指定组合；实际执行顺序保存在 `study.json` 的 `attempts`。每个倾角条件只能使用**同目标深度、同静/动摩擦对**的已通过 0° 对照。

插销直径为 7.986 mm，名义孔径为 8.386 mm。考虑孔壁多边形及运行时网格测量，当前几何的保守实际径向间隙约为 **0.198994 mm**；每次运行以 `scene.json` 与 `effective_protocol` 的测量值为准。数值重叠筛查使用实际间隙，而不是把名义 0.2 mm 当作精确碰撞间隙。

每个条件只采集一次完整尝试。数值拒绝、夹持失败和恢复失败都会保留，不触发确定性的重复试验。外部中断的未完成尝试会保留原文件，恢复运行时可在新目录重新完成该条件。通用配置中的旧 `target_valid`、`attempts_per_slot` 字段不参与本实验调度；本实验按独立计划执行。

## 2. 摩擦：保持插销和夹爪材料，只改变孔材料

静摩擦系数 `μs` 与动摩擦系数 `μd` 分别设置。插销的两项系数固定为 0.75，接触对采用显式 `average` 合并模式：

\[
\mu_{\mathrm{pair}}=\frac{\mu_{\mathrm{peg}}+\mu_{\mathrm{hole}}}{2},
\qquad \mu_{\mathrm{hole}}=2\mu_{\mathrm{pair}}-0.75.
\]

| 目标有效接触对 `(μs, μd)` | 插销材料 `(μs, μd)` | 孔材料 `(μs, μd)` |
| --- | --- | --- |
| `(0.5, 0.5)` | `(0.75, 0.75)` | `(0.25, 0.25)` |
| `(0.75, 0.75)` | `(0.75, 0.75)` | `(0.75, 0.75)` |
| `(1.0, 1.0)` | `(0.75, 0.75)` | `(1.25, 1.25)` |
| `(1.0, 0.5)` | `(0.75, 0.75)` | `(1.25, 0.25)` |

孔和插销在 PhysX 初始化前绑定独立物理材质；每个案例仅切换孔的摩擦系数，并读取 USD 绑定、合并模式和运行时 shape 材料进行核对。机器人/夹爪及插销材料必须保持原值，重置前后也要通过材料检查。

`pair_static_friction`、`pair_dynamic_friction` 是计划值；原始采样中的 `effective_pair_*` 是由运行时 shape 系数和已解析合并模式得到的值。它们不是通过独立摩擦试验测得的真实材料参数。

## 3. 一条参考轨迹的阶段

默认采用 **240 Hz 物理步长**，控制命令和测量每个物理步更新一次。该实验显式选择 240 Hz；它不是旧 FORGE 实验的默认 120 Hz。改变物理频率时使用新目录单独比较。

| 原始 `phase` | 命令与结束条件 |
| --- | --- |
| `approach` | 2 秒平滑接近孔口上方 10 mm 的对齐位置。 |
| `insert` | 保持零倾角，从深度 −10 mm 平滑插入目标深度 `d`。时长为 `(d + 10) / 3.75` 秒。 |
| `settle` | 深度命令固定为 `d`。实际深度必须在 `d ± 0.1 mm` 内连续保持 0.2 秒；离开容许带就重新计时。最多等待 2 秒。 |
| `tilt` | 深度门控通过后，在 1 秒内用 smoothstep 将目标倾角从 0° 变为计划值。深度命令保持 `d`。 |
| `hold` | 保持最终深度和倾角命令 1 秒。 |

插入时长沿用每毫米相同的时间尺度；由于使用 smoothstep，实际命令速度并非全程恒定的 3.75 mm/s。三个目标的插入阶段时长分别约为 4.267、5.867、7.467 秒。

**深度门控使用实际测量，而不是经过预定时间就默认到位。** 数值有效性与夹持保持检查覆盖全部阶段；失败会终止参考轨迹。门控超时则记录 `depth_gate_timeout`，不施加倾角。0° 对照同样经历 `tilt`、`hold` 阶段及相同时长，其目标倾角始终为零。

时序为：生成本步命令 → 执行物理步 → 读取实际状态 → 更新状态机。通过门控的那个采样仍然执行零倾角命令，下一物理步才开始倾角变化。`depth_gate_time_s`、`depth_gate_step` 和原始 `reference_step` 用来核对这个关系。

`depth_attained` 表示曾达到目标附近的下界；完整的稳定到位判断应看 `depth_gate_passed` / `aligned_entry_reached`。倾角施加后，**命令深度固定不等于实际深度固定**。应记录实际深度损失、实际倾角、位置跟踪误差与接触变化。

## 4. 两种恢复的起点与含义

### 连续直接退出：主测量

参考轨迹结束后，直接在当前仿真状态执行 `straight`，保存为 `final_retreat.csv`。中间没有重置或重放，因此保留了该次插入的接触与求解历史。

退出前先在**实际达到的手部位姿**保持 **0.25 秒**，随后维持实际姿态沿世界 +Z 方向上升，计划将销移到孔口上方约 10 mm。各深度统一使用 **5 mm/s 退出速度、0.5 秒速度平滑起步/停步**，中间匀速；退出总时长随距离调整，另有最长 0.5 秒清孔保持。满足清孔保持条件后可提前结束。

正常插入深度下，计划退出距离 `D = 实际终端深度 + 10 mm`，总时长为 `D / 5 + 0.5` 秒。这个版本不使用通用协议里继承的 `retreat_duration_s=6`：固定 6 秒会让深度较大的案例退出更快，混入速度因素。`study.json.recovery_motion` 记录实际规则，各策略结果另存具体 profile 参数；原始 CSV 增加 `command_withdrawal_mm` 和 `command_withdrawal_speed_mm_s`。更短的异常起点使用降低峰速度的短行程曲线。

0.25 秒停止阶段会改变控制目标，可能卸掉先前的位置误差载荷。因此，停止阶段的峰值与真正退出阶段的阻力必须分别报告。

接触跨度字段需要结合筛选定义阅读：原始 `wall_centroid_axial_span_mm` 和 `contact_axial_span_mm` 都只使用严格的直壁受载点，后者并不包含全部销孔接触。当前直壁筛选排除了 z=24 mm 附近的入口边缘，因此这些字段为空不能证明没有分离的接触区域。需要研究入口边缘参与的接触时，使用独立的原始接触区域分析表，并保留其有效性掩码。

### 重放后对齐退出：需要完整前缀匹配

对于完成协议、数值有效、保持夹持且未超操作预算的参考轨迹，重新准备相同材料和种子，重放到同一个终止物理步，然后测试 `realign`。

`realign` 包括：0.25 秒实际位姿保持 → 默认 2 秒将插销 **XY 重新居中并摆正** → 沿 +Z 退出。它同时改变横向位置和姿态；结果不能解释成单独调整 pitch 的效果。

两种恢复的比较要求：

- 重放终点通过姿态、关节、速度、力、接触及夹持状态容差检查；
- **整个参考前缀**逐采样通过物理状态容差、阶段、命令和时间匹配检查；
- 重放前缀自身有效，并且恢复结果具有可用的标签。

`replay_prefix_equal` 是额外的逐值相等诊断；匹配使用的是规定容差。即使所有已记录状态匹配，也不等于已经证明隐藏接触/求解器内部状态完全相同。

## 5. 有效、未知、失败和截断分别表示什么？

| 字段或状态 | 解读 |
| --- | --- |
| `numerically_valid` | 通过当前接触重叠筛查；不等于已证明物理收敛，也不等于动作成功。 |
| `reference_complete` | 完整执行了到位、稳定、倾角与保持协议；需要和数值、夹持、预算条件一起看。 |
| `safe_recovery=True` | 在该次有效测试中满足预算、夹持和清孔要求。 |
| `safe_recovery=False` | 有效测试中的恢复失败；查看具体原因及阶段。 |
| `safe_recovery=None` / 未匹配 / 未测试 | 证据不足，保持未知；不能当作机械卡死，也不能当作安全。 |
| `recovery_censored=True` | 清孔前结束，只观察到部分恢复过程。已记录峰值不是完成退出所需峰值。 |
| `paired_results_known` | 两个策略的匹配与标签条件均满足，允许作带结果语境的比较。 |
| `paired_costs_complete` | 两次恢复均已完成清孔、没有提前截断；安全标签仍需单独查看。 |

操作预算当前继承配置中的腕部合力 20 N、腕部力矩 1 N·m。这是实验预算，不是对机械臂极限或塑料安全载荷的认定。参考与恢复过程均检查预算；没有到达预算也可能因滑移、未清孔或数值问题而结束。

一个 0° 对照必须完整完成协议、数值有效、保持夹持、预算内运行并成功直接退出，才能允许对应的 2° 条件执行。失败对照对应的扰动条件记录为 `excluded`；尚未执行的对照是待完成依赖。

## 6. 力、接触和几何观测

### 不同力信号应分开看

- `wrist_force_n` 是原始腕部合力模长，受本模型的惯性、夹持和接触等影响；销和机器人重力按上述配置禁用，**不是经过标定的纯轴向拔出力**。
- `wrist_force_world_2` 是原始腕部力的世界 Z 分量；世界 +Z 是本场景退出方向。
- `normal_force_world_z_n`、`friction_force_world_z_n` 分别表示孔作用于销的接触法向、摩擦力 Z 分量；`max(0, -Fz)` 提取阻碍向上退出的分量。
- 两个分量各自的最大值可能发生在不同时刻，不能把两个独立峰值相加当成总峰值。

报告分别保留停止、对齐、退出和清孔保持阶段的峰值，以及完整的 100 ms 滑动均值峰值。不存在该阶段或不足一个完整窗口时保持空值，不记成零。

### 逐接触记录

每段运动都有压缩的 `*_contacts.jsonl.gz`。每行包含该次观测的参考/物理时间、局部记录时间、阶段、孔坐标变换，以及：

- `normal_contacts`：法向接触位置、方向、法向载荷和分离量；
- `friction_contacts`：摩擦作用位置与力向量。

**法向接触流和摩擦锚点流不能按数组索引一一配对。** 接触数量与位置可能不同，应以同次观测时间和几何区域分别聚合。CSV 的合力与逐点快照来自同次观测，但求解器内部的接触/位姿子步同步尚未被独立校准。

孔坐标原点位于孔底，孔口约在 `z=25 mm`，直壁到约 `z=24 mm`，孔口倒角另计。默认按 pitch 对应的 X 方向将直壁受载区分到两侧：

- `wall_centroid_axial_span_mm` 是两侧受载区的法向载荷加权质心间距，只在两侧均受载时有效；它不表示恰好存在两个接触点。
- `contact_axial_span_mm` 是有效直壁接触点的轴向跨度，不等于孔深或插入深度。
- `geometry_overlap_estimate_mm` 是全径圆柱包络与实际多边形孔壁的几何估算，忽略有限端面及销倒角；不能把它解释成真实材料压缩或精确网格穿透。

缺少受载区域或几何估算不适用时，相关值保持空白，并保留有效性标志。

## 7. 运行、续跑和只读汇总

以下命令在项目根目录执行。启动脚本使用 `franka-safe-recovery` Conda 环境；输出目录必须是新目录，或明确使用 `--resume`。

先运行 12 个对齐对照：

```bash
./forge_mechanics.sh --headless --physics-hz 240 --phase controls \
  --output-dir outputs/Forge-Mechanics-Pilot-240Hz
```

随后在同一冻结计划下继续符合条件的倾角案例：

```bash
./forge_mechanics.sh --headless --physics-hz 240 --phase all --resume \
  --output-dir outputs/Forge-Mechanics-Pilot-240Hz
```

也可以首次直接使用 `--phase all`；计划顺序仍然是全部对照先行。`--max-new-attempts 2` 可以在完成两条新尝试后暂停，之后沿用相同参数续跑。

`--case-ids` 可用于小批选择，例如先测 `m_d06_fs050_fd050_a00`。选择 2° 条件时，其同深度/摩擦的成功对照必须已经存在于本研究中，或也包含在本次选择中。仅包含 2° 的独立子计划不会自动补跑对照。

续跑检查计划、协议、物理频率、种子、源代码哈希、几何及设备/缓冲选项。已经完整记录的条件不会再次执行；改动冻结实验设置或源码应使用新目录。同一输出目录的并发写入会被锁拒绝。

打印当前汇总，不启动仿真：

```bash
python3 simulation/summarize_mechanics_study.py \
  --study-dir outputs/Forge-Mechanics-Pilot-240Hz
```

将汇总写到独立审阅目录：

```bash
python3 simulation/summarize_mechanics_study.py \
  --study-dir outputs/Forge-Mechanics-Pilot-240Hz \
  --output-dir outputs/Forge-Mechanics-Pilot-240Hz-review
```

审阅目录必须位于原实验目录之外。输出包括：

| 文件 | 一行或内容表示什么 |
| --- | --- |
| `summary.json` | 计划分母、完成/数值有效/到位/恢复配对计数，以及按目标深度分组的计数。 |
| `cases.csv` | 每个计划条件一行，共 24 行；包含未运行及被排除条件。 |
| `mechanics_overview.csv` | 25 列核心总览，保留全部计划条件；未匹配的策略成本为空，完整字段仍在 `cases.csv`。 |
| `mechanics_overview.schema.json` | 核心总览的列、来源哈希与空值规则。 |
| `attempts.csv` | 每条实际尝试，包括中断历史及原因。 |
| `recovery_probes.csv` | 连续直接退出与重放对齐退出的来源、匹配、阶段成本、标签和截断状态。 |
| `control_comparisons.csv` | 同深度/摩擦下 2° 与 0° 的描述性比较；不满足比较条件时差值为空。 |

原实验目录保存 `study.json`、`case_plan.json`、`config.json`、`scene.json`、冻结源码及各案例原始轨迹。每个案例通常包含 `insertion.csv`、`final_retreat.csv`、`terminal_realign_prefix.csv`、`terminal_realign_recovery.csv` 及对应接触文件；未执行的分支没有原始文件。

`complete` 表示所有计划条件已处理，不等于所有动作成功。`complete_with_exclusions` 表示包含数值无效尝试或门控排除；`paused` 表示仍有待完成条件。最终解释应结合汇总表中的细分类别。

## 8. 如何检验机理假设

先比较同深度和摩擦下的 0°/2° 条件，检查实际倾角、实际深度损失、两侧法向载荷、接触跨度，以及直接退出的持续阻力。再在满足完整前缀匹配的案例中比较 `straight` 和 `realign`。截断轨迹、未知配对和数值拒绝都要保留并单列。

**不应预设更深就一定需要更大的拔出力。** 接触间距增大可能改变力矩平衡和法向载荷；固定命令倾角时，实际角度和柔顺变形也会随接触状态调整。深度、几何重叠跨度、真实受载接触间距是三个不同变量。当前正间隙刚体模型也不提供塑料过盈配合的分布式抱紧压力。

孔内停滞、较大退出阻力、策略恢复失败，以及卸载后仍维持的摩擦自锁，应分别寻找证据。可用这些观测检查接触机理，但不能用目标深度、一个腕力峰值或数值重叠直接宣布形成了真实材料卡死。

## 9. 独立的临界角度与频率复查

主矩阵之后增加两个独立计划，使用新的输出目录。它们保留同一插入、停稳、退出速度、材料审计和安全判据，主矩阵的 24 条件定义没有变化。

| 计划 | 条件 | 用途 |
| --- | --- | --- |
| [forge_mechanics_boundary.json](../experiments/forge_mechanics_boundary.json) | 18 mm、有效摩擦 `(1,1)`、0° / 1° / 1.5°，240 Hz | 检查较小倾角是否能保持夹持并实际测量退出力。 |
| [forge_mechanics_rate_boundary_check.json](../experiments/forge_mechanics_rate_boundary_check.json) | 边界组的 18 mm、有效摩擦 `(1,1)`、0° / 1.5° 子集，480 Hz | 复查实际测到较高退出阻力的案例及其对照。 |

每个新条件使用独立仿真进程；一次只完成一条新尝试。例如临界角度组先运行：

```bash
./forge_mechanics.sh --headless --physics-hz 240 \
  --case-plan experiments/forge_mechanics_boundary.json \
  --output-dir outputs/Forge-Mechanics-Boundary-New --max-new-attempts 1
```

随后执行下列续跑命令两次，每次都会启动新的仿真进程：

```bash
./forge_mechanics.sh --headless --physics-hz 240 \
  --case-plan experiments/forge_mechanics_boundary.json \
  --output-dir outputs/Forge-Mechanics-Boundary-New --max-new-attempts 1 --resume
```

480 Hz 组改用对应计划、频率及新目录，首次运行和一次续跑分别完成两个条件。每次仍然核对完整重放前缀；独立进程本身不保证状态相同。改变 Hz 也改变控制更新频率，两个频率点只能检验敏感性，不能据此证明收敛。

480 Hz 选点在观察到 240 Hz、1.5° 的有效高退出阻力后调整，属于自适应探索。先前 0° / 2° 的 [计划](../experiments/forge_mechanics_rate_check.json) 和刚启动的中断记录保留，但没有完成案例，不作为频率比较证据；选择原因另存新频率审阅目录的 `selection.json`。

频率比较使用只读导出器，同时检查物理计划、归档源码、材质／几何、配置差异和实际初始位姿／速度；只读取已完成记录，不把未完成条件填成零：

```bash
python3 simulation/compare_mechanics_rates.py \
  --study-a outputs/Forge-Mechanics-Boundary-20260915 \
  --study-b outputs/Forge-Mechanics-RateBoundary480-20260915 \
  --output-dir outputs/Forge-Mechanics-RateBoundary480-20260915-review
```

生成 `comparison.json` 与 `comparison.csv`。相同随机种子与计划不保证重置后的状态一致；初始状态不匹配时，应解释为整条仿真流程对频率的敏感性，不能据此单独归因于接触积分精度。

临界组的 `design=boundary_angles_v1` 与独立来源哈希由计划验证器检查；1.5° 的案例标识使用 `a1p5`。已有结果保留各自冻结源码，离线分析读取其归档与数据；新实验使用新的输出目录。

## 10. 代表状态和停稳卸载分析

在汇总后，可以依次运行下列离线命令。审阅目录需位于实验原始目录之外。

```bash
python3 simulation/analyze_mechanics_states.py --study-dir outputs/Forge-Mechanics-New \
  --output-dir outputs/Forge-Mechanics-New-review
python3 simulation/analyze_mechanics_contact_regions.py --study-dir outputs/Forge-Mechanics-New \
  --output-dir outputs/Forge-Mechanics-New-review
python3 simulation/analyze_mechanics_stop_unloading.py --study-dir outputs/Forge-Mechanics-New \
  --output-dir outputs/Forge-Mechanics-New-review
python3 simulation/validate_mechanics_output.py --study-dir outputs/Forge-Mechanics-New \
  --output-dir outputs/Forge-Mechanics-New-review --deep
```

`mechanics_states.csv` 将有效观测与实际执行区分：恢复初始化 `t=0` 只是参考末端的复制快照，可能 `state_observed=True` 但 `phase_executed=False`、执行时长为 0。只有真实执行了 STOP 的记录才能讨论停稳前后的载荷变化。

`mechanics_stop_unloading.json` 同时比较该恢复分支的起止观测与两个完整的 100 ms 窗口，并保留位姿变化。载荷下降属于这个动作流程中的时序观测；没有无 STOP 对照时，不能将其全部归因于停稳动作。
