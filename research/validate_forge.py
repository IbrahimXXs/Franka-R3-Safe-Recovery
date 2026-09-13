"""Summarize FORGE pilot readiness and compare matching cases across timesteps."""
import argparse
import csv
import json
import math
from pathlib import Path


def read(path):return json.loads(path.read_text())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--studies',type=Path,nargs='+',required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    (args.output_dir/'validate_forge.py').write_bytes(Path(__file__).read_bytes())
    base=read(args.baseline/'study.json');cal=read(args.calibration/'calibration.json')
    if base['status']!='pilot_complete':raise ValueError('Baseline must be complete')
    data={}
    for directory in args.studies:
        m=read(directory/'study.json')
        if m['status']!='pilot_complete':raise ValueError(f'Incomplete comparison input: {directory}')
        if m['physics_hz'] in data:raise ValueError('Supply one comparison study per physics rate')
        data[m['physics_hz']]={'directory':directory,'manifest':m,'cases':{a['slot']:a for a in m['attempts'] if a['status']=='complete'}}
    rates=sorted(data)
    if len(rates)<2:raise ValueError('At least two physics rates are required')
    common=sorted(set.intersection(*(set(v['cases']) for v in data.values())))
    integrity=[]
    for rate,d in data.items():
        paths=sorted(d['directory'].glob('*/insertion.csv'))+sorted(d['directory'].glob('*/*retreat.csv'))
        for path in paths:
            with path.open() as stream:samples=list(csv.DictReader(stream))
            numeric=[[float(v) for k,v in r.items() if k!='phase'] for r in samples]
            finite=bool(samples) and all(math.isfinite(v) for r in numeric for v in r)
            times=[float(r['time_s']) for r in samples]
            grid=all(abs(b-a-1/rate)<1e-6 for a,b in zip(times,times[1:]))
            clock_present=bool(samples) and 'physics_time_s' in samples[0]
            clock_matches=all(math.isclose(float(r['physics_time_s']),float(r['time_s']),rel_tol=2**-23,abs_tol=1e-7) for r in samples) if clock_present else None
            integrity.append({'file':str(path),'rows':len(samples),'finite':finite,'logged_time_grid_valid':grid,
                              'independent_clock_recorded':clock_present,'independent_clock_matches':clock_matches})
    rows=[]
    for rate,d in data.items():
        for slot,a in d['cases'].items():
            rows.append({'physics_hz':rate,'slot':slot,'family':a['family'],**a['metrics'],
                         'safe_final_retreat':a['final_retreat']['safe_recovery'],
                         'final_retreat_work_j':a['final_retreat']['recovery_work_j']})
    with (args.output_dir/'rate_metrics.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    comparisons=[]
    for lo,hi in zip(rates,rates[1:]):
        for slot in common:
            a=data[lo]['cases'][slot];b=data[hi]['cases'][slot]
            for key in ('family','offset_x_mm','offset_y_mm','roll_deg','pitch_deg','insertion_duration_s'):
                if a[key]!=b[key]:raise ValueError(f'Comparison cases differ: slot {slot}, {key}')
            ma,mb=a['metrics'],b['metrics'];checks={};differences={}
            for field,absolute,relative in (('max_force',.5,.15),('max_normal_load',1.,.2),
                                           ('max_torque',.02,.15),('max_wrist_force_n',.5,.15),
                                           ('max_wrist_torque_nm',.02,.15),('max_depth',.1,0.),('max_penetration',.005,0.)):
                difference=abs(ma[field]-mb[field]);threshold=max(absolute,relative*max(abs(ma[field]),abs(mb[field])))
                checks[field]=difference<=threshold;differences[field]={'difference':difference,'tolerance':threshold}
            checks['both_numerical_screens_passed']=ma['numerically_valid'] and mb['numerically_valid']
            checks['same_insertion_outcome']=ma['insertion_success']==mb['insertion_success']
            for flag in ('stalled','force_budget_exceeded','torque_budget_exceeded'):
                checks['same_'+flag]=ma[flag]==mb[flag]
            checks['same_grasp_outcome']=ma['grasp_retained']==mb['grasp_retained']
            ra,rb=a['final_retreat'],b['final_retreat']
            checks['same_known_recovery_outcome']=ra['safe_recovery'] is not None and ra['safe_recovery']==rb['safe_recovery']
            for field,absolute in (('max_wrist_force_n',.5),('max_wrist_torque_nm',.02)):
                difference=abs(ra[field]-rb[field]);threshold=max(absolute,.15*max(abs(ra[field]),abs(rb[field])))
                checks['retreat_'+field]=difference<=threshold
                differences['retreat_'+field]={'difference':difference,'tolerance':threshold}
            difference=abs(ra['recovery_work_j']-rb['recovery_work_j'])
            tolerance=max(.0001,.2*max(ra['recovery_work_j'],rb['recovery_work_j']))
            checks['recovery_work']=difference<=tolerance
            differences['recovery_work_j']={'difference':difference,'tolerance':tolerance}
            comparisons.append({'lower_hz':lo,'higher_hz':hi,'slot':slot,'family':a['family'],
                                'passed':all(checks.values()),'checks':checks,'differences':differences})
    baseline_probes=[probe for a in base['attempts'] for cp in a['checkpoints'] for probe in cp['probes']]
    baseline_pass=(len(baseline_probes)>=8 and all(p['replay_matched'] and p['safe_recovery'] is True for p in baseline_probes)
                   and all(a['metrics']['insertion_success'] for a in base['attempts']))
    families={r['family'] for r in rows}
    contact_cases=[r for r in rows if r['max_force']>.1 and r['numerically_valid']]
    finest=[c for c in comparisons if c['higher_hz']==max(rates)]
    gates={'centered_checkpoint_baseline':baseline_pass,'wrist_calibration':cal['passed'],
           'reference_and_final_retreat_csv_integrity':bool(integrity) and all(r['finite'] and r['logged_time_grid_valid'] and r['independent_clock_matches'] is not False for r in integrity),
           'six_families_piloted':len(families)==6,'screened_contact_cases_present':bool(contact_cases),
           'finest_pair_converges_on_sampled_cases':len(finest)>=3 and all(c['passed'] for c in finest)}
    verdict={'ready_for_100':all(gates.values()),'gates':gates,'rates_hz':rates,'common_slots':common,
             'comparisons':comparisons,'calibration_directory':str(args.calibration),
             'csv_integrity':integrity,
             'baseline_directory':str(args.baseline),'study_directories':[str(d) for d in args.studies],
             'scope':'Pilot evidence for this synthetic model and tested policies; not real-world material validation or a global recoverability certificate. Rate comparisons include integration, per-step controller sampling and seeded preparation; they do not isolate the contact solver alone.'}
    (args.output_dir/'validation.json').write_text(json.dumps(verdict,indent=2)+'\n')
    lines=['# FORGE validation assessment','',f'Full collection ready under the pilot gates: **{verdict["ready_for_100"]}**.','',
           '| Check | Passed |','|---|---|']
    lines += [f'| {k} | {v} |' for k,v in gates.items()]
    lines += ['', '| Rate pair | Slot | Family | Passed | Failed checks |','|---|---:|---|---|---|']
    lines += [f'| {c["lower_hz"]} → {c["higher_hz"]} Hz | {c["slot"]} | {c["family"]} | {c["passed"]} | '+', '.join(k for k,v in c['checks'].items() if not v)+' |' for c in comparisons]
    lines += ['', 'Thresholds and individual differences are recorded in [validation.json](validation.json). '
              'The finest available pair controls the convergence gate; the native-rate comparison remains visible. '
              'A failed gate is not hidden by collecting more easy trajectories.', '',
              '[Per-rate metrics](rate_metrics.csv)', '', '![Rate comparison](rate_comparison.png)', '', verdict['scope']]
    (args.output_dir/'report.md').write_text('\n'.join(lines)+'\n')
    if common:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(3,len(common),figsize=(5*len(common),9),squeeze=False)
        for col,slot in enumerate(common):
            for rate,d in data.items():
                case=d['cases'][slot]
                with (d['directory']/case['folder']/'insertion.csv').open() as stream:
                    samples=list(csv.DictReader(stream))
                times=[float(r['time_s']) for r in samples]
                for row,(key,label) in enumerate((('force_norm_n','Peg–socket force (N)'),('wrist_force_n','Raw wrist force (N)'),('depth_mm','Actual depth (mm)'))):
                    axes[row,col].plot(times,[float(r[key]) for r in samples],label=f'{rate} Hz',linewidth=.8)
                    axes[row,col].set(xlabel='Time (s)',ylabel=label);axes[row,col].grid(alpha=.2)
            axes[0,col].set_title(f'Slot {slot}: {case["family"]}')
            axes[0,col].legend()
        fig.suptitle('FORGE pilot: identical commanded cases across physics rates')
        fig.tight_layout();fig.savefig(args.output_dir/'rate_comparison.png',dpi=160);plt.close(fig)
    print(json.dumps(gates,indent=2));print('Ready for 100:',verdict['ready_for_100'])


if __name__=='__main__':main()
