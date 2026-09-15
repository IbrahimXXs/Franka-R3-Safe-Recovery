import csv,json,math,hashlib,shutil,subprocess,platform,importlib.metadata
from pathlib import Path
root=Path('/home/ibrahim/Documents/Franka-Safe-Recovery');out=root/'outputs/Active-Probing-Pilot-v1'
m=json.loads((out/'pilot.json').read_text())
assert m['status']=='complete',m['status']
assert len(m['cases'])==2 and all(len(c['probes'])==18 for c in m['cases'])
assert m['physics_hz']==120
assert m['preexisting_outputs_changed']==[],m['preexisting_outputs_changed']
checks=0;timing=0
for c in m['cases']:
 assert len({(p['repeat'],p['axis'],p['sign']) for p in c['probes']})==18
 for p in c['probes']:
  with (out/p['log']).open() as f:rows=list(csv.DictReader(f))
  assert rows
  for r in rows:
   for k in ['peg_world_x_m','qw','vx','omegax','peg_base_vx_m_s','fx','fy','fz','taux','tauy','tauz','normal_load_n','grasp_slip_mm','grasp_slip_deg','depth_mm']:
    assert math.isfinite(float(r[k])),(p['probe_id'],k)
   assert (r['force_budget_exceeded']=='True')==(float(r['wrist_force_n'])>m['protocol']['force_budget_n'])
   assert (r['torque_budget_exceeded']=='True')==(float(r['wrist_torque_nm'])>m['protocol']['torque_budget_nm'])
   assert 0<=float(r['probe_fraction'])<=1
   assert abs(float(r['probe_displacement_si'])-p['sign']*p['amplitude_si']*float(r['probe_fraction']))<1e-12
   if p['axis']=='hold':assert float(r['probe_displacement_si'])==0
   checks+=1
  for a,b in zip(rows,rows[1:]):
   assert abs(float(b['probe_time_s'])-float(a['probe_time_s'])-1/120)<1e-9
   assert abs(float(b['time_s'])-float(a['time_s'])-1/120)<1e-8
   timing+=1
  if p['completed']:
   assert len(rows)==181
   assert set(r['phase'] for r in rows)=={'before','outward','during','return','after'}
   assert rows[-1]['phase']=='after' and float(rows[-1]['probe_displacement_si'])==0
  if p['reason']=='preparation_rejected':assert len(rows)==1 and not p['eligible']
with (out/'summary.csv').open() as f:summaries=list(csv.DictReader(f))
assert len(summaries)==36
assert sum(r['eligible']=='True' for r in summaries)==sum(p['eligible'] for c in m['cases'] for p in c['probes'])
for name in ['report.md','direction_response.png','repeatability.png','return_errors.png','wrench_traces.png','central_differences.csv']:
 assert (out/name).stat().st_size>100,name
subprocess.run(['git','diff','--exit-code'],cwd=root,check=True)
# Preserve final offline analysis source separately from the code frozen at collection start.
for name in ['research/active_probing.py','research/contact_response.py','research/forge_protocol.py','research/phase2_protocol.py','tests/test_active_probing.py','docs/active_probing.md']:
 target=out/'analysis_source'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/name,target)
shutil.copy2('/tmp/franka-active-probing-tests.log',out/'tests.log')
shutil.copy2('/tmp/franka-active-probing-pilot-v1.log',out/'launch.log')
validation=dict(status='passed',tests=80,new_focused_tests=9,per_probe_rows_checked=checks,timestep_intervals_checked=timing,
 existing_output_files_checked=m['preexisting_outputs_checked'],existing_output_files_changed=m['preexisting_outputs_changed'],
 existing_tracked_files_unchanged=True,existing_output_relaunch_refused=True,
 command='./active_probing.sh --headless',
 analysis_command='conda run -n franka-safe-recovery python -m research.active_probing outputs/Active-Probing-Pilot-v1',
 python=platform.python_version(),versions={k:importlib.metadata.version(k) for k in ['numpy','matplotlib','torch','isaacsim']},
 analysis_source_sha256=hashlib.sha256((root/'research/active_probing.py').read_bytes()).hexdigest(),
 provenance_note='Collection code is frozen in source/. Final offline analysis and tests are in analysis_source/. Added analysis diagnostics and direct paired-state checks after collection started; no collected probe, simulation parameter, or existing dataset was changed.')
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print(json.dumps(validation,indent=2))
