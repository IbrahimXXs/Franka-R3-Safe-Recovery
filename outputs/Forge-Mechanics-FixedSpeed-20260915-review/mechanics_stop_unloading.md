# STOP 前后载荷离线审计

本次读取时主矩阵有 24 个完成案例；本表只分析其中 12 个倾斜案例。运行中案例不读取。重新运行同一命令即可更新。

```bash
python3 simulation/analyze_mechanics_stop_unloading.py --study-dir '/home/zephyr/Downloads/Franka-R3-Safe-Recovery-main (1)/Franka-R3-Safe-Recovery-main/outputs/Forge-Mechanics-FixedSpeed-20260915' --output-dir '/home/zephyr/Downloads/Franka-R3-Safe-Recovery-main (1)/Franka-R3-Safe-Recovery-main/outputs/Forge-Mechanics-FixedSpeed-20260915-review'
```

CSV 每行是一项恢复策略。JSON 保存相应端点、固定 100 ms 窗口、原始接触分区和输入哈希。空值表示没有可用观测，不能读作零载荷。

`stop_executed=false` 表示没有新的 STOP 物理步，通常只保存了恢复 t=0 的参考末端复制行。`full_stop_observed=true` 才说明记录到设定的 0.25 s STOP。

`stop_endpoint_change` 比较同一恢复分支的 t=0 与真实 STOP 末端；`stop_tail_mean_change` 比较恢复前最后 100 ms 与 STOP 最后 100 ms。负值表示后者降低。这些是时序变化，不是相隔时刻峰值的比值。短时零接触可能伴随脉冲，应同时看窗口均值与载荷占空比。

STOP 将控制目标切换为当时测得的手部位姿，可能释放此前的跟踪误差。即使销角度几乎不变，接触载荷也可以改变。这里没有无 STOP 对照，不能将下降单独归因为保持动作或声称统计显著。

同摩擦不同深度的参考末端和法向载荷峰位置见 JSON 的 `cross_depth_reference_states`。需要同时比较实际角度、偏移、夹持滑移和终止原因；上缘与反侧内壁的受载区跨度不是唯一识别出的“两点接触距离”。
