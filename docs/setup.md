# Simulation setup

The existing Miniconda environment is `franka-safe-recovery`, located at
`/home/ibrahim/miniconda3/envs/franka-safe-recovery`.
Other Conda environments and the system Python are unchanged.

| Component | Pinned version |
| --- | --- |
| Python | 3.11.16 |
| Isaac Sim | 5.1.0.0 |
| Isaac Lab source | v2.3.0 |
| Isaac Lab commit | `3c6e67bb5c7ada942a6d1884ab69338f57596f77` |
| PyTorch / Torchvision | 2.7.0+cu128 / 0.22.0+cu128 |
| CUDA runtime | 12.8, included in the Python packages |

Isaac Lab is installed in editable mode from `.deps/IsaacLab`. Moving or deleting
that checkout breaks the environment. Its upstream source is kept unchanged.
`environment/requirements.txt` records the complete installed Python dependency
set, including the simulator extensions.

## Use the installed environment

Run commands from the project root:

```bash
./run.sh --keep-open
./run.sh --headless --steps 10
```

For direct Python use:

```bash
conda activate franka-safe-recovery
python simulation/launch.py --keep-open
python -m unittest discover -s tests -v
python -m pip check
```

`run.sh` clears inherited Python and simulator library paths before selecting the
project environment. `PYTHONNOUSERSITE=1` disables packages from `~/.local` while
the environment is active. The operating system and GPU driver remain shared.

The project launcher selects DLSS Performance and enables Eco mode on every run.
It reapplies Eco mode after the initial playback/reset event, which otherwise
disables it. These settings are applied by the project script; no system-wide
graphics preferences are changed.

VS Code uses this environment as its workspace default. If it has remembered an
older selection, use **Python: Select Interpreter** to choose
`franka-safe-recovery`.

## Recreate on Linux x86_64

Use these commands in a fresh copy of this project, from its root. If the Conda
environment or `.deps/IsaacLab` already exists, reuse it instead of recreating it.

```bash
conda env create -f environment/conda.yml
conda activate franka-safe-recovery
mkdir -p .deps
git clone --branch v2.3.0 --depth 1 https://github.com/isaac-sim/IsaacLab.git .deps/IsaacLab
git -C .deps/IsaacLab rev-parse HEAD
python -m pip install --upgrade pip
python -m pip install --build-constraint environment/build-constraints.txt -r environment/requirements.txt
python -m pip check
python -m unittest discover -s tests -v
./run.sh --headless --steps 10
```

Check the printed commit against the table above. Run pip from the project root:
the editable dependency paths are relative to that directory. The build
constraints preserve `pkg_resources` for `flatdict` and a `wheel` version
compatible with Isaac Sim's `packaging==23.0` requirement.

The first launch can request NVIDIA license acceptance and download scene assets.
Internet access is required for assets that are not already cached. No separate
system CUDA toolkit installation is required for the supplied scripts.

## Hardware and validated scope

Validated on Ubuntu 22.04.5, Intel i5-14600KF, 32 GB RAM, and an RTX 3080 (10 GB),
with NVIDIA driver `580.178.04`. The initial driver/library mismatch was resolved
by rebooting after Ubuntu's automatic driver update; this project did not change
the system driver.

The custom FR3 scene uses the official native USD asset and CUDA physics.
See [scene details and validation](scene.md). The original stock Panda baseline
is preserved in `outputs/baseline/`. Larger batches and sensor-camera workloads
have not been validated.

The RTX 3080 is below the published Isaac Sim 5.1 GPU minimum. Start with one
environment. If `nvidia-smi` reports a driver/library mismatch after a future
system update, save work, reboot, and check it again before launching.

Sources: [Isaac Lab installation](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/setup/installation/pip_installation.html),
[Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html).
