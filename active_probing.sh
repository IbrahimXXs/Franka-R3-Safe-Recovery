#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
conda_exe="${CONDA_EXE:-/home/ibrahim/miniconda3/bin/conda}"
if [[ ! -x "$conda_exe" ]]; then conda_exe="$(command -v conda)"; fi
cd "$project_dir"
exec env -u PYTHONPATH -u PYTHONHOME -u ISAAC_PATH -u EXP_PATH \
-u CARB_APP_PATH -u LD_LIBRARY_PATH -u LD_PRELOAD \
"$conda_exe" run --no-capture-output -n franka-safe-recovery \
python -u -s "$project_dir/simulation/launch_active_probing.py" "$@"
