# Interactive study viewer

From the project directory:

```bash
./view_study.sh outputs/full_suite_guard_fix
# Any other study folder, or its study.json path:
./view_study.sh outputs/MY_STUDY/study.json
# Generate without opening the browser:
./view_study.sh outputs/MY_STUDY --no-open
# Choose a portable output file:
./view_study.sh outputs/MY_STUDY --output /tmp/my-study.html
```

The equivalent command is `python3 visualization/view_study.py STUDY_DIRECTORY`.
Python's standard library is sufficient; activating the simulation environment
is optional. The output defaults to `STUDY_DIRECTORY/interactive_viewer.html`.
Open it in Chrome or Firefox. It is an offline snapshot with bundled Plotly,
raw samples, manifest, report and original figures. No server or Isaac Sim
process is started. Regenerating replaces the HTML snapshot only.

## Controls

- Select trials to compare. Scenario colors are consistent; realignment curves
  are dashed. The initial selection compares the loaded-deep policies when present.
- Plot against experiment time, time relative to each trial's recovery start,
  or actual insertion depth. Negative recovery-relative time is before recovery.
- Filter insertion, retreat, full recovery, or another recorded phase. Partial
  trials that never reached recovery have no recovery-relative time curve.
- Select signed axial force, axial resistance, net force, normal/friction axial
  components, normal contact load, or minimum separation. Resistance is defined
  only during insertion and retreat; other phases are gaps in that signal.
- Hover for recorded sample details; drag or scroll to zoom. Time-axis zoom is
  shared across force, torque and motion charts. Double-click or use Reset zoom.
  A single selected trial adds phase shading to time plots.
- Compare saved recovery metrics, optionally restricted to trials passing the
  overlap screen. Sort the full outcome table using its column headings.
- Use each chart's camera icon to export a PNG. Export selected summary CSV from
  the profile toolbar; source-file downloads are in Report & original files.

All raw samples are plotted without smoothing or downsampling. Filtering and
zooming do not recalculate summary costs. The table lists every trial, while
cost bars use selected completed trials with the requested metric available.

## Data and validity

`study.json` is required. `summary.csv`, `report.md`, `force_profiles.png` and
`force_vs_depth.png` are loaded when present. The detailed interactive profiles
require the per-trial `SCENARIO__RECOVERY.csv` files. If those CSVs are missing,
the viewer still provides summary comparisons, settings, and original figures;
it does not attempt to reconstruct data from PNGs. When no raw samples are
available, it opens on the cost comparison tab.

Numerically invalid, incomplete, or invalid-sensor trials are excluded from cost
comparisons. Their partial traces can be enabled explicitly for diagnosis and
are labeled accordingly. Missing costs remain blank, never zero. Raw CSVs with
no manifest/summary outcome are treated as incomplete. A passed overlap screen
is not evidence of timestep convergence.

Reports are rendered with a small safe Markdown subset (headings, tables, inline
emphasis, code and bundled-file links/images); the exact Markdown is also shown
and downloadable. The source study files are not modified.

## Bundled dependency

`vendor/plotly-basic-3.1.0.min.js` is Plotly.js basic v3.1.0 from
<https://cdn.plot.ly/plotly-basic-3.1.0.min.js>, under the MIT license in
`vendor/PLOTLY-LICENSE`. It supplies interactive SVG scatter/bar charts and
image export. The Python environment has no additional dependencies.

## Verification

Four data-loading tests cover stale summaries overridden by invalid manifest
status, partial trials, missing/nonfinite samples, invalid sensor configurations,
and safe HTML embedding. Run `python3 -m unittest discover -s tests -p test_study_viewer.py -v`.
The generated full-study and five-file-only dashboards were also tested in Chrome:
trial selection, phase and depth views, linked time zoom, invalid-cost exclusion,
overlap-screen filtering, sortable outcome table, and original-file/report access.

## Phase 2 datasets

Use the same launcher: `./view_study.sh outputs/Phase2-Pilot-480G`.
Phase 2 is detected from `study.json` and opens on the checkpoint-recoverability
scatter and table. You can filter insertion families and inspect the tested
policy outcomes. The insertion-characterization tab exposes success, stalls,
peak loads, penetration, budget flags, and numerical validity. The profile tab
plots each reference insertion. Per-policy replay and recovery samples remain
available in the attempt folders for detailed analysis.

Invalid or incomplete parents are shown as unknown in the central plot, even
when an earlier prefix has a recorded positive witness. The original prefix
labels are preserved in the dataset for separately qualified analysis.
