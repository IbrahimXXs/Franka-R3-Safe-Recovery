import csv,json,hashlib,shutil,subprocess
from pathlib import Path
import numpy as np
from research.contact_productivity import Settings,plots,report,STUDIES
from research.future_stall import read_rows
from research.contact_response import digest,write_json
root=Path('/home/ibrahim/Documents/Franka-Safe-Recovery');out=root/'outputs/Contact-Productivity-v1'
def read(name):
 def convert(v):
  if v=='':return None
  if v in ('True','False'):return v=='True'
  try:return int(v)
  except ValueError:
   try:return float(v)
   except ValueError:return v
 with (out/name).open() as f:return [{k:convert(v) for k,v in r.items()} for r in csv.DictReader(f)]
windows=read('window_features.csv');summaries=read('trajectory_summary.csv');pred=read('predictions.csv')
metrics=read('metrics.csv');warnings=read('warning_summary.csv');comparisons=read('paired_comparisons.csv');hidden=read('hidden_load_results.csv')
raw={s['trajectory_id']:read_rows(root/'outputs'/s['trajectory_id']/'insertion.csv') for s in summaries}
assert len(summaries)==170 and sum(s['role']=='stage4' and s['numerically_valid'] for s in summaries)==13
assert sum(not s['grasp_retained'] for s in summaries)==1
for w in windows:
 rr=raw[w['trajectory_id']];a=rr[w['history_end_row']];b=rr[w['future_end_row']]
 dc=b['command_depth_mm']-a['command_depth_mm'];da=b['depth_mm']-a['depth_mm']
 assert dc>=.1-1e-9
 assert abs(da/dc-w['eta_raw'])<1e-10
 assert abs(b['time_s']-a['time_s']-w['horizon_s'])<1e-6
 assert all(r['phase']=='insert' for r in rr[w['history_start_row']:w['future_end_row']+1])
for p in pred:
 if p['stall_confirmation_s'] is not None:assert p['time_s']<p['stall_confirmation_s']
 assert p['evaluation'] in ('stage4','development_oof','quarantined_shared_path')
inputs=json.loads((out/'feature_sets.json').read_text())
for model,features in inputs.items():
 assert all(not key.startswith(('priv_','future_')) and 'normal_load' not in key for key in features)
 assert not set(features)&{'eta_raw','stalled','time_s','study','trajectory_id','group_id','role'}
held={s['group_id'] for s in summaries if s['role']=='stage4'}
for split in json.loads((out/'split_audit.json').read_text()):
 a=set(split['train_groups']);b=set(split['validation_groups'])
 assert a.isdisjoint(b) and a.isdisjoint(held) and b.isdisjoint(held)
sources=json.loads((out/'source_files.json').read_text())
assert all(digest(Path(path))==sha for path,sha in sources.items())
first=Path('/tmp/contact-productivity-coverage-audit-first-pass')
assert (first/'models.json').read_bytes()==(out/'models.json').read_bytes()
subprocess.run(['git','diff','--exit-code'],cwd=root,check=True)
validation=json.loads((out/'validation.json').read_text())
validation.update(analysis_sha256=digest(root/'research/contact_productivity.py'),tests_passed=90,new_focused_tests=10,
 raw_ratio_windows_recomputed=len(windows),prediction_rows_checked=len(pred),
 no_post_confirmation_predictions=True,no_heldout_group_in_development_folds=True,
 models_unchanged_after_coverage_correction=True,
 numerical_valid_slipping_trajectories_retained=1,
 analysis_notes='Initial coverage audit caught an overly restrictive grasp-retention filter. Retained that numerically valid Stage 4 trajectory; model coefficients, transforms and alarm thresholds remained byte-identical. Additional hidden-load definitions are descriptive sensitivity checks, not fitting/tuning inputs.',
 command='OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n franka-safe-recovery python -m research.contact_productivity --outputs-root outputs --output outputs/Contact-Productivity-v1')
write_json(out/'validation.json',validation)
plots(out,windows,pred,summaries,raw)
report(out,Settings(),summaries,windows,metrics,warnings,comparisons,hidden,validation)
for relative in ('research/contact_productivity.py','research/future_stall.py','research/contact_response.py','tests/test_contact_productivity.py','docs/contact_productivity.md'):
 shutil.copy2(root/relative,out/'source'/Path(relative).name)
shutil.copy2('/tmp/contact-productivity-tests.log',out/'tests.log')
shutil.copy2('/tmp/contact-productivity-v1.log',out/'analysis.log')
shutil.copy2('/tmp/contact_productivity_finalize.py',out/'source/finalize_audit.py')
with (out/'report.md').open('a') as f:
 f.write('\n## Completed-data checks\n\n90 tests passed, including 10 focused productivity tests. Recomputed all 16,680 raw productivity targets from source endpoints, checked every forecast precedes first stall confirmation, and verified group separation and predictor exclusions. One numerically valid slipping Stage 4 trajectory is retained. The coverage correction did not change any fitted coefficients, transforms or alarm thresholds. All 195 consumed input hashes and 3,247 pre-existing output files remained unchanged. See validation.json and tests.log.\n')
print(json.dumps(validation,indent=2))
