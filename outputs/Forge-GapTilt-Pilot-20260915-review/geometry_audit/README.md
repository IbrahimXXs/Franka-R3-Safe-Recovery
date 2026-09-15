# 候选案例的独立几何核查归档

案例：`gap_0200/g0200_d30_pitch_pos20_try00`，名义单边间隙 0.2 mm、实际插入到 30% 深度时开始施加 +2° pitch 扰动。

本目录是离线诊断归档。复制的是已经关闭的原始插入和连续直接退出记录；没有修改主批次、实时输出或物理源码，也不启动 Isaac Sim 或 GPU 仿真。

## 文件

- `crosscheck.py`：可直接运行的归档诊断脚本。支持 `--phase retreat`，仅从真实退出阶段选择力峰，避免把停止阶段残余插入力当成拔出力。
- `crosscheck_original.py`：最初 `/tmp/forge_geometry_crosscheck.py` 的原样副本，仅用于来源追溯。
- `forge_geometry.py`：几何修改与哈希函数的冻结副本；主脚本使用这个本地副本，不依赖工作区后续修改。
- `assets/`：原始 NVIDIA 孔/销 USD，共约 9 MB。脚本在内存中重建 0.2 mm 孔径，不修改这些文件。
- `inputs/`：已关闭的 `insertion.csv`、`final_retreat.csv` 及运行时 `scene.json` 副本。
- `results/candidate_insertion.json`：原始插入中最大报告重叠与最大腕力所在帧，以及前后各两帧的核查。
- `results/candidate_retreat.json`：仅在 `retreat` 阶段选择最大报告重叠与最大腕力，再核查前后各两帧。
- `manifest.json`：全部归档文件的 SHA-256、原始 USD URL 和输入记录来源。

## 已验证的结果

- 重建孔网格 SHA-256 与 `scene.json` 的 `socket_mesh_sha256` 相同：`8241aad76384b8c880b75008758ac2eb2c34334472ae70db41ca31428d3145c9`。
- 实际退出腕力峰值位于 CSV 第 226 个样本（从零计数）、恢复时间 1.883333 s，腕力 2.543674 N。
- 该帧的向下接触力为 2.549269 N；PhysX 报告重叠为 17.844 μm，独立直孔壁越界约 16.277 μm。
- 没有误选恢复零时刻 `stop` 阶段的 3.332919 N 残余插入力。

## 方法和局限

脚本把原始销三角网格按记录位姿变换到固定孔坐标系，将三角面裁剪到直孔段（z = 0–24 mm）和倒角段（z = 24–25 mm），计算其相对于各孔壁平面的最大有符号越界。正值表示进入对应孔壁半空间，负值表示仍在该半空间的孔内侧；`null` 表示销表面与该轴向区域没有交集。

**这是局部孔壁平面越界，不是完整网格穿透深度，也不是塑料变形、材料损伤或接触力估计。** 假设固定孔为当前实验使用的单位旋转，几何为已审计的米制 Factory 网格。不能直接用于任意形状或旋转的孔。

独立核查使用网格和日志位姿，不调用 PhysX SDF 距离，但日志位姿与接触缓存未必代表物理步内同一个求解时刻。旧样本的相邻帧比较曾支持时序差异的推断，**尚未通过 PhysX 接触 API 的直接时序保证确认**；本脚本不做全局帧平移修正。

当前 25% 间隙重叠筛查是实验规则，不是已证实的物理精度门槛。通过该规则和本核查不能代替时间步、SDF 分辨率收敛或实物验证。

## 运行

从本目录执行。使用当前机器已经安装的 USD Python 包，无需网络或启动仿真：

```bash
forge_env=/home/zephyr/miniforge3/envs/franka-safe-recovery
forge_usd="$forge_env/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311"

# 插入：选择最大重叠和腕力，保留前后各两帧。
env PYTHONPATH="$forge_usd" \
  LD_LIBRARY_PATH="$forge_env/lib:$forge_usd/bin" \
  "$forge_env/bin/python" crosscheck.py \
  --profile inputs/insertion.csv --radial-clearance-mm 0.2 \
  --neighbors 2 --scene-json inputs/scene.json \
  > /tmp/candidate_insertion_reproduced.json

# 拔出：必须按 retreat 阶段选峰，排除 stop 阶段。
env PYTHONPATH="$forge_usd" \
  LD_LIBRARY_PATH="$forge_env/lib:$forge_usd/bin" \
  "$forge_env/bin/python" crosscheck.py \
  --profile inputs/final_retreat.csv --radial-clearance-mm 0.2 \
  --phase retreat --neighbors 2 --scene-json inputs/scene.json \
  > /tmp/candidate_retreat_reproduced.json
```

在已经能够 `import pxr` 的其他 Python 环境中，可直接调用 `python crosscheck.py ...`。脚本及输入在本目录内自包含；移动目录后 JSON 中的绝对输入路径会改变，输入哈希与数值结果应保持一致。
