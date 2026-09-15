# 低拔出力预算的物理含义与观测范围

核查日期：2026-09-15。本文基于本仓库及其本地 Isaac Lab 源码的只读审计；未启动仿真、未修改物理接触或夹持模型。

## 1. 新实验实际验证什么

将允许载荷从 20 N 降到 3 / 4 / 5 N，是设置**任务级载荷预算**：观察既有插入与恢复动作能否在较小预算内完成，并记录何时、在哪个阶段首次超限。有限夹持能力或易碎零件可以解释为何任务需要较低预算，但 **4 N 不是经过标定的夹持极限、销的断裂载荷或机器人硬件极限**。

预算使用已有原始腕部三轴合力模长及三轴力矩模长；等于预算允许，仅严格大于预算时触发停止后续任务步。它不是轴向力闭环限幅，也不会把超限样本裁剪到阈值。新方案明确记录这些语义：[预算定义](../research/forge_budget.py#L19)。

## 2. 必须区分的观测量

| 量 | 现有来源和含义 | 不能解释为 |
| --- | --- | --- |
| `wrist_force_n` | 原始腕部关节反力的三个力分量之模长，含横向分量；未使用平滑后的信号 | 纯轴向拔出力、夹指夹紧力 |
| `wrist_force_world_2` | 将原始腕力旋转到世界系后的 Z 分量；应同时注明世界 +Z 为退出方向与传感器符号 | 与合力模长相同的预算信号 |
| `normal_load_n` | 孔作用于销的受载接触点法向载荷标量和 ΣN | 左右夹指法向力之和、腕部合力、轴向拔出阻力 |
| `normal_force_world_z_n` / `friction_force_world_z_n` | 孔—销法向与摩擦接触合力各自的世界 Z 分量 | 夹指对销的接触力 |
| `grasp_slip_mm/deg` | 销相对夹手位姿偏离准备末端基准的程度 | 直接测得的夹持力或已经掉落 |
| 驱动参数、位置/力矩命令 | 模型限值、PD 参数或控制器目标 | 实际接触法向力或零件强度 |

腕部原始量来自 `get_link_incoming_joint_force()`；平滑值另存：[FORGE 传感器读取](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/forge/forge_env.py#L94)。后端使用原始值，转换坐标并记录模长：[力转换](../simulation/forge_backend.py#L248)、[预算用模长](../simulation/forge_backend.py#L272)。这是当前仿真动力学下的腕部关节反力观测，不是实物负载传感器的校准结果。

接触视图只选择销及孔过滤器：[视图构建](../simulation/forge_backend.py#L189)、[接触读取](../simulation/contact.py#L43)。法向与摩擦是独立数据流，不能按索引强行配对。该视图没有夹指接触，因此现有数据不能直接给出左右夹指法向力。已有日志只额外记录夹指位置、速度：[夹指日志](../simulation/forge_backend.py#L284)。

## 3. 夹爪的“40”是什么

配置使用隐式 PD 驱动，手指关节 `effort_limit_sim=40`、位置刚度 7500、阻尼 173、速度上限 0.04；这是关节驱动参数：[手指驱动配置](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_env_cfg.py#L178)。其中 `friction=0.1` 是关节静摩擦参数，不能代替夹指—销表面摩擦系数。物体表面摩擦由另一条材质设置路径更新：[表面材质摩擦](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_utils.py#L31)。

每步仍给夹爪位置目标 0：[后端命令](../simulation/forge_backend.py#L195)。上游将其写入手指位置目标，并将显式手指力矩命令置 0：[命令写入](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_env.py#L326)。显式命令为 0 不代表实际夹紧力为 0，因为物理引擎仍执行隐式位置 PD。反过来，驱动上限 40 也不代表每个夹指始终施加 40 N 法向力，更不能直接推出 80 N 总夹力或某个确定拔出能力。

隐式驱动的 `computed_effort/applied_effort` 是根据位置误差、速度误差与前馈项形成的**近似估计**，不是接触力传感器：[本地实现](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab/isaaclab/actuators/actuator_pd.py#L118)、[Isaac Lab v2.3 官方源码说明](https://isaac-sim.github.io/IsaacLab/v2.3.0/_modules/isaaclab/actuators/actuator_pd.html)。只读回读 effort limit、刚度、阻尼可以确认参数，却不能补足夹指法向力测量。

同样，环境中的 `applied_wrench` 来自任务空间 PD 控制器返回的命令：[调用处](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_env.py#L308)、[任务力到关节力矩](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_control.py#L74)、[返回值](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_control.py#L100)。它不是腕部实测力。机械臂关节驱动限值 87 / 12 也属于关节空间参数，不能直接换成统一的腕部 20 N 上限：[机械臂驱动配置](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_env_cfg.py#L159)。

## 4. 有限夹持能力：合理动机，尚未标定

库仑摩擦模型把切向承载能力与法向载荷联系起来；静止接触允许的切向力处于摩擦锥内。[Modern Robotics 第 12.2.1 节](https://modernrobotics.northwestern.edu/nu-gm-book-resource/12-2-1-friction/)

据此，在两侧夹指摩擦系数相同、准静态、轴向载荷主要由两处切向摩擦承担、忽略复杂力矩与接触分布的简化条件下，可以写出：

```text
F_t ≤ μ_g × (N_left + N_right)
```

这里 μ_g 是夹指—销的摩擦系数，N_left/right 是实际夹指接触法向载荷。该关系是由接触摩擦规律得到的简化受力启发，不是本实验已验证的夹持能力模型。不能把孔—销的 ΣN、孔的摩擦系数或驱动配置 40 代入后宣称已经得到 4 N 安全界限。涉及偏心力矩、转动、动态冲击或柔性夹垫时，还需考虑接触力矩与载荷分配。

## 5. 动力学、损伤和预算时间范围

当前销仍为刚体 articulation，质量配置 0.019 kg，没有断裂、塑性损伤或易碎销失效状态：[销尺寸与质量](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_tasks_cfg.py#L88)、[刚体销配置](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_tasks_cfg.py#L159)。降低预算只能验证预算内动作与超限检测，不能验证真实损伤的发生或避免。

机器人与销配置均禁用局部重力：[机器人](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_env_cfg.py#L127)、[销](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_tasks_cfg.py#L165)。reset 末尾恢复全局重力不会撤销这些局部配置：[全局重力恢复](https://github.com/isaac-sim/IsaacLab/blob/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_env.py#L820)。这不能表述为实物传感器已经完成软件重力补偿。0.019 kg 在地球重力下约为 0.186 N，只是重量量级参考，不能标定 4 N 或解释全部 20 N 差距。

`prepare()` 先 reset，再稳定抓持 3 秒，最后设置记录时间零点并返回初始观测：[准备顺序](../simulation/forge_backend.py#L210)。因此新版预算从 **recorded t=0 初始观测**生效，覆盖其后的记录插入、STOP、姿态调整、退出及清空保持；不覆盖此前 reset、抓取与稳定过程。初始观测本身也要检查预算，但它不是新执行的任务步。

每个物理步结束后保存观测、检查超限，可以保证检测后不再发出下一任务步，不能保证第一次超限没有过冲。结束命令流或关闭仿真也不是经过验证的实际制动动作。

## 6. 最小日志与术语

建议保留以下信息，避免后续把预算中断误读为夹持或材料实验：

- **条件与参数**：预算值、比较符号、激活时刻和排除的准备阶段；物理频率、原始配置与 scene、材质回读、源码哈希；关节名称与 effort limit / stiffness / damping 回读。回读失败保留 null 与原因。
- **逐步观测**：记录时间与物理时间、阶段、命令与实际深度/姿态、原始腕部六维量及世界系轴向分量、孔—销法向/摩擦量、夹持相对滑移、夹指位置速度、数值重叠筛查。
- **首次超限事件**：样本索引、阶段、腕合力/力矩、阈值与过冲、是否初始状态已超限、检测后实际执行步数；完整 100 ms 窗口不足时不补 0。
- **结果分别记录**：预算是否超限、是否执行恢复/退出、是否清空、数值有效性、夹持是否满足位姿门槛。未执行阶段为 null 或明确未执行，不能当作“0 N 成功退出”。

推荐使用“预算超限/预算中断”“满足夹持位姿门槛”“已清空”等术语；不要将这些标签直接写成“销已断裂”“已经掉落”或“不可恢复卡死”。不同策略来自独立准备时，单策略实际结果与满足轨迹匹配要求后的配对比较也应分开。

