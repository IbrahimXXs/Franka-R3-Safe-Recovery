# Preventing insertion-induced jamming

English16:9 scientific briefing:12 main slides paced for10 minutes, plus4 appendix pages (depth comparison, budget-study coverage, two reference pages).

Final PDF: `output/pdf/safe_recovery_motion_planning_control.pdf` (repository-relative).

## Current revision

Expanded the metal-peg/tight-plastic-socket scenario and explained finite friction grip, part/fixture damage and sudden-release concerns as physical motivations. Removed the6-degree test page. Added detailed proposed methods for insertion-risk/exit-cost prediction, insertion planning/control, and force-limited recovery. The primary objective remains preventing insertion-induced jamming. Restoration of withdrawability is secondary. Quantitative simulation results are unchanged.

## Retained evidence

- `talk_manifest.json`: page titles, durations and concise speaker notes.
- `final_qa.json` and `pdf_text_audit.json`: final layout/text/source checks.
- `presentation_sources.zip`: final builder, generated figures, scene replay assets, literature/source verification records, and16 final page renders. Full third-party source PDFs are excluded.
- `history/`: previous PDFs, source bundles and audits.

## Rebuild

Extract `presentation_sources.zip` into the repository root. It restores `tmp/pdfs/safe_recovery_talk/` with its original relative paths. Run:

```bash
/home/zephyr/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python tmp/pdfs/safe_recovery_talk/build_talk.py
```

The existing PNG/PDF assets suffice. Optional data-figure regeneration uses:

```bash
/home/zephyr/miniforge3/envs/franka-safe-recovery/bin/python tmp/pdfs/safe_recovery_talk/figures/build_figures.py
```

Raw data stay in the existing Forge-Budget output and review folders. Figure generation does not rerun dynamics. Scene replay uses `scene/render_recorded_scene.py --headless --enable_cameras` in the repository Isaac environment and needs a GPU.

## Interpretation

Existing4N matched trials supply an adverse scenario and preliminary force-limited recovery evidence. They do not validate a preventive insertion policy. Wrist-force norm differs from axial pull and squeeze. Censored branches do not reveal their unobserved future. Physical damage, release dynamics, learned predictors, MPC and predictive filters require separate validation.
