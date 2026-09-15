# Safe recovery research presentation

English 16:9 scientific briefing:12 main slides paced for10 minutes, plus evidence coverage and two reference pages.

Final PDF: `output/pdf/safe_recovery_motion_planning_control.pdf` (repository-relative).

## Retained evidence

- `talk_manifest.json`: page titles, planned durations and concise speaker notes.
- `final_qa.json` and `pdf_text_audit.json`: latest PDF checks.
- `presentation_sources.zip`: exact authoring sources, six vector/PNG data figures, four recorded-state scene renderings, literature notes, source hashes, scene audit, and all15 final page renders.

The bundle preserves the original repository-relative paths for reproducibility. It references the existing `outputs/Forge-Budget-V1-20260915`, its review folder, and `outputs/Forge-Budget-Angle6-20260915` data. It does not duplicate or alter these studies.

## Rebuild

From the repository root, extract `presentation_sources.zip` into the repository. This restores `tmp/pdfs/safe_recovery_talk/`. Then run:

```bash
/home/zephyr/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python tmp/pdfs/safe_recovery_talk/build_talk.py
```

Optional figure regeneration (uses existing recorded data, no dynamics run):

```bash
/home/zephyr/miniforge3/envs/franka-safe-recovery/bin/python tmp/pdfs/safe_recovery_talk/figures/build_figures.py
```

Optional recorded-state scene replay needs the repository Isaac Sim environment and GPU, using `scene/render_recorded_scene.py --headless --enable_cameras`. The saved PNGs already suffice for rebuilding the PDF.

## Interpretation

The main4N matched comparison establishes strategy-dependent recovery in the sampled rigid-body model. Peaks are raw wrist-force norms, not calibrated axial pull or grip capacity. The straight branch is censored at its first exceedance. Proposed MPC and safety filters have not been implemented or validated by this presentation. See the source logs for detailed scope.
