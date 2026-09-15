# 最高 6° 的命令倾角扩展实验

## 范围

根据用户补充的最大角度 6°，在[上一版任务载荷预算实验](forge_budget_study.md)后新增独立计划 `Forge-budget-angle6-v1`。旧的八组计划与原始输出保持不变。

本轮六组命令倾角为 **1.5°、2°、3°、4°、5°、6°**。1.5°是上一轮主要候选的复跑锚点。所有条件固定目标嵌入深度 18 mm、孔—销有效静／动态摩擦系数 1、名义径向间隙 0.2 mm、25 mm 盲孔、腕部合力预算 4 N、腕部力矩预算 1 N·m、240 Hz、种子 20260913。每组计划直接退出和 XY 回中＋姿态回正后退出两种策略，共 12 个计划分支。

允许角度范围在 `research/forge_mechanics_plan.py` 的参数检查中增加 3／4／5／6°，原有机械矩阵、状态转移和运动控制公式不变。接触模型、夹爪驱动和预算触发规则沿用上一轮。新的计划和源码单独归档，不能续写旧实验目录。

## 角度与角速度的解释

达到实际目标深度 ±0.1 mm 并连续保持 0.2 s 后，在固定命令深度施加原有的 **1 s smoothstep 倾角渐变**，之后保持 1 s。角度 `θ=A(3u²−2u³)`，其中 `u=t/T`，因此完整命令渐变的理论峰值角速度为 `1.5A/T`。

| 命令幅度 A | 渐变时长 T | 理论命令峰值角速度 |
|---|---|---|
| 1.5° | 1 s | 2.25°/s |
| 2° | 1 s | 3°/s |
| 3° | 1 s | 4.5°/s |
| 4° | 1 s | 6°/s |
| 5° | 1 s | 7.5°/s |
| 6° | 1 s | 9°/s |

这是固定时长的倾斜命令对照，**幅度和命令角速度同时变化**，不是严格隔离角度作用的准静态实验。提前中止的分支可能没有执行到计划峰值角速度；理论值不是测得的实际角速度。

最大命令角度是 6°，不保证机械系统实际达到 6°。输出同时保留命令目标、最后一帧命令角度、实际末端／最大角度、倾斜渐变是否完成、首次超限阶段、原始力及 100 ms 窗口均值。不能把“6°命令条件在实际 1°时中止”描述成“达到 6°后的拔出失败”。

## 执行和证据

沿用 reference 初始样本起的预算监测，包含 STOP、调姿、退出和清孔。严格大于预算时保存触发帧并结束任务推进；不裁剪力，不放宽预算来强行达到角度。超限后的实物刹停没有仿真验证。若参考阶段不合格，另一策略为 `not_tested`，保留在 12 个计划分支中。

两策略的比较需要实际完整参考前缀匹配。角度组之间只比较到倾斜开始前深度门槛的状态和命令前缀；这不表示倾斜后仍为相同接触状态。1.5°锚点也将与上一轮封存记录核对，但复跑不是独立随机样本。

## 复现

```bash
python simulation/run_budget_study.py \
  --case-plan experiments/forge_budget_angle6_v1.json \
  --output-dir outputs/Forge-Budget-Angle6-New
python simulation/summarize_budget_study.py \
  --study-dir outputs/Forge-Budget-Angle6-New \
  --output-dir outputs/Forge-Budget-Angle6-New-review --plots
python simulation/validate_budget_output.py \
  --study-dir outputs/Forge-Budget-Angle6-New \
  --output-dir outputs/Forge-Budget-Angle6-New-review --deep
python simulation/analyze_budget_comparisons.py \
  --study-dir outputs/Forge-Budget-Angle6-New
```

本轮原始数据保存于 `outputs/Forge-Budget-Angle6-20260915/`，独立审阅目录为 `outputs/Forge-Budget-Angle6-20260915-review/`。文件、单位和空值语义沿用[预算日志规范](forge_budget_study.md#4-数据与标签)。
