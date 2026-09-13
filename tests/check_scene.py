"""End-to-end GPU check: python tests/check_scene.py (requires Isaac Sim environment)."""

import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    with tempfile.TemporaryDirectory(prefix="fr3-scene-check-") as directory:
        log = Path(directory) / "diagnostics.jsonl"
        subprocess.run(
            [str(ROOT / "run.sh"), "--headless", "--demo", "--steps", "300", "--log", str(log)],
            cwd=ROOT, check=True, timeout=180,
        )
        records = [json.loads(line) for line in log.read_text().splitlines()]
        metadata, frames = records[0], records[1:]
        assert metadata["task"] == "FR3-Custom-PegInsert-v1"
        assert metadata["arm_joints"] == [f"fr3_joint{i}" for i in range(1, 8)]
        assert metadata["dlss_mode"] == 0 and metadata["eco_mode"] is True
        assert len(frames) == 300
        assert abs(frames[0]["insertion_depth_m"][0] + 0.025) < 0.001
        assert not frames[0]["geometrically_seated"][0]
        assert frames[-1]["geometrically_seated"][0]
        assert abs(frames[-1]["insertion_depth_m"][0] - 0.020) < 0.0005
        assert all(frame["lateral_error_m"][0] < 0.0005 for frame in frames)
        assert all(abs(frame["simulation_time_s"] - frame["nominal_time_s"]) < 0.001 for frame in frames)
        print("FR3 scene check passed: prepared hold, 20 mm insertion, alignment, timing, and render defaults.")


if __name__ == "__main__":
    main()
