"""Reproduce the bounded native-rate repeatability assessment (not convergence)."""
import csv
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2] if Path(__file__).parent.name=='Forge-Native-Stability' else Path.cwd()
OUT=ROOT/'outputs/Forge-Native-Stability'
PAIRS=[('Forge-Pilot-120','Forge-Native-Repeat',2),('Forge-Pilot-120','Forge-Native-Repeat',4),
       ('Forge-Challenge-120','Forge-Native-Loaded-Recovery',4)]

def read(name):
    m=json.loads((ROOT/'outputs'/name/'study.json').read_text())
    if m['status']!='pilot_complete':raise ValueError(f'Incomplete study: {name}')
    assert m['physics_hz']==120
    return m


def main():
    comparisons=[]
    for original,repeated,slot in PAIRS:
        a=next(a for a in read(original)['attempts'] if a['slot']==slot)
        b=next(a for a in read(repeated)['attempts'] if a['slot']==slot)
        for field in ('offset_x_mm','offset_y_mm','roll_deg','pitch_deg','insertion_duration_s'):
            assert a[field]==b[field]
        checks={};differences={}
        for section in ('metrics','final_retreat'):
            x,y=a[section],b[section]
            limits=[('max_force',.5,.15),('max_torque',.02,.15),('max_normal_load',1.,.2),
                    ('max_penetration',.005,0.),('max_wrist_force_n',.5,.15),('max_wrist_torque_nm',.02,.15)]
            limits += [('max_depth',.1,0.)] if section=='metrics' else [('recovery_work_j',.0001,.2)]
            for field,absolute,relative in limits:
                error=abs(x[field]-y[field]);tolerance=max(absolute,relative*max(abs(x[field]),abs(y[field])))
                key=f'{section}.{field}'
                checks[key]=error<=tolerance;differences[key]={'absolute_difference':error,'tolerance':tolerance}
            for field in ('numerically_valid','grasp_retained'):
                checks[f'{section}.{field}']=x[field] is True and y[field] is True
            flags=['force_budget_exceeded','torque_budget_exceeded']
            flags += ['insertion_success','stalled'] if section=='metrics' else ['safe_recovery']
            for field in flags:checks[f'{section}.{field}']=x[field]==y[field] and x[field] is not None
        path=ROOT/'outputs'/repeated/b['folder']/'insertion.csv'
        rows=list(csv.DictReader(path.open()))
        checks['continuous_120_hz_grid']=all(abs(float(v['time_s'])-float(u['time_s'])-1/120)<1e-6 for u,v in zip(rows,rows[1:]))
        checks['finite_samples']=all(math.isfinite(float(v)) for r in rows for k,v in r.items() if k!='phase')
        comparisons.append({'original':original,'repeat':repeated,'slot':slot,'family':b['family'],
                            'roll_deg':b['roll_deg'],'passed':all(checks.values()),'checks':checks,'differences':differences})
    loaded=read('Forge-Native-Loaded-Recovery')['attempts'][0]
    verdict={'physics_hz':120,'source':'installed FORGE default','repeatability_passed':all(c['passed'] for c in comparisons),
             'comparisons':comparisons,'loaded_checkpoints':loaded['checkpoints'],
             'scope':'Repeatability and operational screening for three seeded cases at 120 Hz. Not a guarantee for every randomized trajectory or a cross-timestep convergence certificate. Unknown recovery probes remain unknown.'}
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'validation.json').write_text(json.dumps(verdict,indent=2)+'\n')
    (OUT/'check_repeatability.py').write_bytes(Path(__file__).read_bytes())
    lines=['# Native FORGE rate check','',f'Working physics rate: **120 Hz**, inherited from the installed scene. The earlier 240/480 Hz runs were explicit comparisons.','',
           f'Three-case same-rate repeatability check passed: **{verdict["repeatability_passed"]}**.','',
           '| Case | Tilt | Passed | Failed checks |','|---|---:|---|---|']
    lines += [f'| {c["family"]} | {c["roll_deg"]:.3f}° | {c["passed"]} | '+', '.join(k for k,v in c['checks'].items() if not v)+' |' for c in comparisons]
    lines += ['', 'The comparisons check force/torque peaks, reported overlap, depth, grasp retention, insertion/stall/budget labels, final recovery outcome and contact work. Thresholds are retained in validation.json.','',
              '## Loaded checkpoint','']
    for cp in loaded['checkpoints']:
        lines.append(f'At requested depth {cp["depth_mm"]:g} mm: reached={cp["reached"]}, Y_R_tested={cp.get("Y_R_tested")}.')
        for probe in cp['probes']:
            lines.append(f'- {probe["policy"]}: {probe["reason"]}; replay matched={probe["replay_matched"]}; safe within configured limits={probe.get("safe_recovery")}.')
    lines += ['', verdict['scope'],'', 'Normal launch (no rate override):','', '```bash\n./forge_study.sh --headless\n```','',
              '[Contact repetitions](../Forge-Native-Repeat/report.md) · [Loaded recovery](../Forge-Native-Loaded-Recovery/report.md) · [Earlier rate comparison](../Forge-Validation/report.md)']
    (OUT/'report.md').write_text('\n'.join(lines)+'\n')
    print('Native repeatability passed:',verdict['repeatability_passed'])

if __name__=='__main__':main()
