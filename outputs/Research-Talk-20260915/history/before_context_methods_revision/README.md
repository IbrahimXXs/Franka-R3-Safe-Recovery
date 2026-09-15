# Preventing insertion-induced jamming

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

Research priority: first develop insertion planning and control that avoids insertion-induced jamming; then restore withdrawability under a bounded operation force. The main 4 N matched comparison supplies an adverse scenario and preliminary force-limited recovery evidence. It does not validate a preventive insertion policy. Peaks are raw wrist-force norms, not calibrated axial pull or grip capacity. The straight branch is censored at its first exceedance. Proposed MPC and safety filters have not been implemented or validated by this presentation. See the source logs for detailed scope.

## Prevention-first revision

The cover, objectives, interpretation, dataset design, proposed control loop and conclusion now reflect prevention as the primary objective. The next validation compares baseline and risk-aware insertion using insertion completion, peak load and subsequent exit cost. Earlier presentation and provenance are retained in `history/before_prevention_first_revision/`.
