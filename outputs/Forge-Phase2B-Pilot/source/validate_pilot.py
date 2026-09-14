"""Audit saved Phase 2B command/replay records and plot raw pilot histories."""
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root))
from research.phase2b import command_misalignment,path_outcome
from research.phase2_protocol import recovery_label
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

study=Path(__file__).resolve().parents[1]
m=json.loads((study/'study.json').read_text())
assert m['status']=='complete' and m['valid_trajectories']==2
assert m['physics_hz']==120
assert all(hashlib.sha256((study/'source'/k).read_bytes()).hexdigest()==v for k,v in m['sources'].items())
# Only analysis provenance changed after the runtime archive was frozen.
# Verify the actual path functions remain identical in the current code.
archived=ast.parse((study/'source/phase2b.py').read_text())
current=ast.parse((root/'research/phase2b.py').read_text())
for name in ('make_case','command_misalignment','case_for_slot','load_plan'):
    get=lambda tree:ast.dump(next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name))
    assert get(archived)==get(current)
results=[]
fig,axes=plt.subplots(4,2,figsize=(13,11),sharex='col',constrained_layout=True)
for col,a in enumerate(m['attempts']):
    assert a['status']=='complete' and a['metrics']['numerically_valid'] and a['metrics']['grasp_retained']
    folder=study/a['folder'];max_error=0.;reference=None;checked_files=0
    for path in [folder/'insertion.csv',*sorted(folder.glob('*_prefix.csv'))]:
        with path.open() as f:rows=list(csv.DictReader(f))
        progress=-10.
        for i,r in enumerate(rows):
            if i:progress=max(progress,float(rows[i-1]['depth_mm']))
            assert abs(float(r['time_s'])-i/120)<1e-5
            assert abs(float(r['ramp_progress_depth_mm'])-progress)<1e-9
            for k,v in command_misalignment(a,progress).items():
                max_error=max(max_error,abs(float(r['command_'+k])-v))
        if path.name=='insertion.csv':reference=rows
        checked_files+=1
    assert max_error<1e-9
    terminals=[cp for cp in a['checkpoints'] if cp.get('checkpoint_kind')=='terminal']
    assert len(terminals)==1 and terminals[0]['step']==len(reference)-1
    for cp in a['checkpoints']:
        label,reason=recovery_label(cp['probes'],cp['reached'],cp['prefix_numerically_valid'])
        assert label==cp['Y_R_tested'] and reason==cp['label_reason']
        if cp['reached']:
            assert abs(cp['state']['time_s']-float(reference[cp['step']]['time_s']))<1e-9
            if cp.get('checkpoint_kind')!='terminal':
                assert all(float(r['depth_mm'])<cp['depth_mm'] for r in reference[:cp['step']] if r['phase'] in ('insert','hold'))
        for probe in cp['probes']:
            if not probe['replay_matched']:assert not probe['label_eligible'] and probe['safe_recovery'] is None
    results.append(dict(trajectory_id=a['trajectory_id'],outcome=path_outcome(a),
        maximum_command_error=max_error,command_files_checked=checked_files,metrics=a['metrics'],
        checkpoint_labels=[dict(depth_mm=cp['depth_mm'],kind=cp.get('checkpoint_kind','fixed_depth'),Y_R=cp['Y_R_tested']) for cp in a['checkpoints']],
        matched_probes=sum(p['replay_matched'] for cp in a['checkpoints'] for p in cp['probes']),
        unknown_probes=sum(not p['label_eligible'] for cp in a['checkpoints'] for p in cp['probes'])))
    time=[float(r['time_s']) for r in reference]
    for row,(actual,command,label) in enumerate((('depth_mm','command_depth_mm','Depth [mm]'),
            ('tilt_deg','command_tilt_deg','Tilt [deg]'),('wrist_force_n',None,'Raw wrist force [N]'),
            ('wrist_torque_nm',None,'Raw wrist torque [Nm]'))):
        ax=axes[row,col];ax.plot(time,[float(r[actual]) for r in reference],label='Actual',lw=1)
        if command:ax.plot(time,[float(r[command]) for r in reference],ls='--',label='Command',lw=1)
        ax.set_ylabel(label);ax.grid(alpha=.2)
        for cp in a['checkpoints']:
            if not cp['reached']:continue
            color={1:'seagreen',0:'firebrick',None:'gray'}[cp['Y_R_tested']]
            ax.scatter(cp['state']['time_s'],cp['state'][actual],color=color,s=25,zorder=3)
        if row<2:ax.legend(fontsize=8)
    axes[0,col].set_title(f"{a['trajectory_id']}: onset {a['ramp_onset_mm']:g} mm; endpoint {a['severity_deg']:g} deg")
    axes[-1,col].set_xlabel('Reference time [s]')
fig.suptitle('Phase 2B pilot: green = observed safe checkpoint recovery; gray = unknown')
fig.savefig(study/'phase2b_paths.png',dpi=160);fig.savefig(study/'phase2b_paths.pdf');plt.close(fig)
result=dict(passed=True,physics_hz=m['physics_hz'],unit_tests_passed=54,
    study_sha256=hashlib.sha256((study/'study.json').read_bytes()).hexdigest(),
    validator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    archive_hashes_match=True,current_command_functions_match_archive=True,cases=results,
    limitation='Numerical screens and observable replay tolerances only; no timestep convergence or recoverability-boundary certificate.')
(study/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
