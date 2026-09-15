"""Phase 2 dataset tables and checkpoint characterization plots."""
import csv
import copy
import json
import os
from pathlib import Path
from research.forge_gap_metrics import FACTOR_FIELDS


def table(path,rows):
    if not rows:return
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)


def write_gap_batch_report(directory,case_plan,run_directories,status='running'):
    """Merge immutable child ledgers into one reviewable factor-study report.

    Children can be incomplete or absent while a batch is running. Canonical
    case IDs prevent local slot zero in each geometry from collapsing counts.
    Raw CSV folder links remain relative to the aggregate report directory.
    """
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True)
    slots={case['case_id']:i for i,case in enumerate(case_plan['cases'])}
    attempts=[];sources=[];protocol=None
    for run_directory in run_directories:
        run_directory=Path(run_directory)
        if not run_directory.is_absolute():run_directory=directory/run_directory
        ledger=run_directory/'study.json'
        if not ledger.exists():continue
        child=json.loads(ledger.read_text())
        protocol=copy.deepcopy(child['protocol']) if protocol is None else protocol
        sources.append(dict(folder=os.path.relpath(run_directory,directory),status=child['status'],
                            physics=child.get('physics'),elapsed_wall_seconds=child.get('elapsed_wall_seconds')))
        for original in child['attempts']:
            attempt=copy.deepcopy(original)
            if attempt.get('case_id') not in slots:
                raise ValueError(f"Child case is absent from aggregate plan: {attempt.get('case_id')}")
            attempt['slot']=slots[attempt['case_id']]
            attempt['folder']=os.path.relpath(run_directory/attempt['folder'],directory)
            attempts.append(attempt)
    if protocol is None:
        # No measured data yet: export the plan without inventing a protocol or
        # pretending that absent children are measured unknown checkpoints.
        table(directory/'cases.csv',case_plan['cases'])
        return None
    protocol['target_valid']=len(case_plan['cases'])
    manifest=dict(study='Forge-Controlled-Phase2-v1',mode='collect',status=status,
                  case_plan=case_plan,protocol=protocol,attempts=attempts,
                  pilot_candidate_count=len(case_plan['cases']),batch_sources=sources,
                  validation_status='provisional',budget_signal='raw wrist force and torque norms')
    target=directory/'aggregate_study.json';temporary=target.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n');temporary.replace(target)
    write_phase2_report(directory,manifest)
    report=directory/'report.md'
    report.write_text(report.read_text().replace('(study.json)','(aggregate_study.json)')+
                      '\n## Independent geometry runs\n\n'+
                      '\n'.join(f"- [{source['folder']}]({source['folder']}/report.md): {source['status']}" for source in sources)+'\n')
    return manifest


def write_phase2_report(directory,manifest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory)
    gap_study=bool(manifest.get('case_plan',{}).get('schema')=='Forge-gap-tilt-v1') if manifest.get('case_plan') else False
    gap_study |= any('radial_clearance_mm' in a for a in manifest['attempts'])
    trajectories=[];checkpoints=[];probes=[]
    for a in manifest['attempts']:
        if 'metrics' not in a:continue
        base={k:a[k] for k in ('trajectory_id','slot','retry','family','offset_x_mm','offset_y_mm','roll_deg','pitch_deg','insertion_duration_s','folder','status')}
        grouping={k:a[k] for k in ('sampling_slot','sample_role','split_group_id','path_group_id','severity_deg',
            'ramp_onset_mm','ramp_full_depth_mm','final_offset_x_mm','final_offset_y_mm','final_roll_deg','final_pitch_deg',
            *FACTOR_FIELDS) if k in a}
        base.update(grouping)
        continuation={}
        if gap_study:
            final_retreat=a.get('final_retreat') or {}
            continuation={'continuation_straight_present':bool(final_retreat),
                          'continuation_straight_origin':'original_reference_continuation_without_replay' if final_retreat else None}
            continuation.update({f'continuation_straight_{k}':v for k,v in final_retreat.items()
                                 if not isinstance(v,(dict,list)) and k not in ('replay_matched','label_eligible')})
        trajectories.append({**base,**a['metrics'],**continuation})
        for cp_index,cp in enumerate(a['checkpoints']):
            state=cp.get('state',{})
            checkpoint_id=cp.get('checkpoint_id',f"{a['trajectory_id']}:{cp.get('checkpoint_kind','fixed_depth')}:{cp.get('step',cp_index)}")
            row=dict(trajectory_id=a['trajectory_id'],folder=a['folder'],family=a['family'],slot=a['slot'],
                checkpoint_id=checkpoint_id,
                parent_numerically_valid=a['metrics']['numerically_valid'],parent_complete=a['status']=='complete',
                parent_insertion_success=a['metrics'].get('insertion_success'),parent_stalled=a['metrics'].get('stalled'),
                checkpoint_depth_mm=cp['depth_mm'],reached=cp['reached'],reference_time_s=state.get('time_s'),
                checkpoint_kind=cp.get('checkpoint_kind','fixed_depth'),
                prefix_numerically_valid=cp['prefix_numerically_valid'],Y_R_tested=cp.get('Y_R_tested'),label_reason=cp.get('label_reason'),
                depth_mm=state.get('depth_mm'),force_n=state.get('force_norm_n'),torque_nm=state.get('torque_norm_nm'),
                normal_load_n=state.get('normal_load_n'),tilt_deg=state.get('tilt_deg'),
                tip_x_mm=state.get('tip_x_mm'),tip_y_mm=state.get('tip_y_mm'),
                prefix_max_force_n=cp.get('prefix_max_force_n'),prefix_max_torque_nm=cp.get('prefix_max_torque_nm'),
                prefix_max_normal_load_n=cp.get('prefix_max_normal_load_n'),prefix_max_penetration_mm=cp.get('prefix_max_penetration_mm'))
            row.update({k:state.get(k) for k in ('wrist_force_n','wrist_torque_nm',
                'wrist_force_world_0','wrist_force_world_1','wrist_force_world_2',
                'command_tilt_deg','command_roll_deg','command_pitch_deg','tilt_triggered','tilt_ramp_fraction')})
            row.update({k:cp.get(k) for k in ('prefix_max_wrist_force_n','prefix_max_wrist_torque_nm')})
            row.update({k:state.get(k) for k in ('fx','fy','fz','taux','tauy','tauz','vx','vy','vz','omegax','omegay','omegaz','qw','qx','qy','qz')})
            for i in range(1,8):
                for k in (f'joint{i}_rad',f'joint_velocity{i}_rad_s'):row[k]=state.get(k)
            row.update(grouping)
            checkpoints.append(row)
            for p in cp['probes']:
                flat={k:v for k,v in p.items() if not isinstance(v,dict)}
                flat.update({f'replay_error_{k}':v for k,v in p.get('replay_errors',{}).items()})
                flat.setdefault('checkpoint_id',checkpoint_id)
                flat.setdefault('checkpoint_kind',cp.get('checkpoint_kind','fixed_depth'))
                probes.append(dict(trajectory_id=a['trajectory_id'],folder=a['folder'],family=a['family'],**flat,**grouping))
    table(directory/'trajectories.csv',trajectories)
    table(directory/'checkpoints.csv',checkpoints)
    table(directory/'recovery_probes.csv',probes)
    fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
    # No inference or interpolation between probed depths. Invalid parents and
    # uncertain probes remain explicitly unlabelled in the central plot.
    for value,color,label in [(1,'#13866b','Safe policy observed'),(0,'#b23f4c','Tested policies failed'),(None,'#8b98a1','Unknown / numerical review')]:
        selected=[r for r in checkpoints if r['reached'] and
            (r['Y_R_tested'] if r['parent_numerically_valid'] and r['parent_complete'] else None)==value]
        axes[0].scatter([r['depth_mm'] for r in selected],[r['force_n'] for r in selected],
            c=color,label=label,marker='x' if value is None else 'o',alpha=.8,s=40)
    axes[0].set(xlabel='Actual checkpoint depth [mm]',ylabel='Insertion-state net force [N]',title='Recovery labels at observed insertion states')
    if manifest.get('study')!='Forge-Controlled-Phase2-v1':
        axes[0].axhline(manifest['protocol']['force_budget_n'],color='#b87c27',ls=':',lw=1)
    axes[0].legend(fontsize=8)
    for policy,marker in [('straight','o'),('realign','^')]:
        if gap_study:
            for value,color,label in [(True,'#13866b','safe'),(False,'#b23f4c','failed'),(None,'#8b98a1','unknown')]:
                selected=[p for p in probes if p['policy']==policy and
                    (p.get('safe_recovery') if p.get('label_eligible') else None)==value and p.get('max_wrist_force_n') is not None]
                axes[1].scatter([p['depth_mm'] for p in selected],[p['max_wrist_force_n'] for p in selected],
                    marker=marker,c=color,label=f'{policy}: {label}',alpha=.8)
        else:
            selected=[p for p in probes if p['policy']==policy and p['label_eligible'] and p.get('safe_recovery')]
            axes[1].scatter([p['depth_mm'] for p in selected],[p['max_force'] for p in selected],marker=marker,label=policy,alpha=.8)
    axes[1].set(xlabel='Checkpoint depth [mm]',ylabel='Recorded whole-recovery wrist peak [N]' if gap_study else 'Peak force over successful recovery [N]',
                title='All measured probes (including failures)' if gap_study else 'Observed safe-policy force costs')
    axes[1].legend(fontsize=8)
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(directory/'recovery_characterization.png',dpi=160)
    fig.savefig(directory/'recovery_characterization.pdf');plt.close(fig)
    accepted={r['slot'] for r in trajectories if r['status']=='complete' and r['numerically_valid']}
    heading='# Phase 2 randomized insertion characterization'
    status=f"Status: **{manifest['status']}**. {len(accepted)} / {manifest['protocol']['target_valid']} numerically valid trajectory slots filled."
    if manifest.get('study')=='Forge-Controlled-Phase2-v1':
        heading='# FORGE randomized insertion collection' if manifest.get('mode')=='collect' else '# Controlled FORGE pilot'
        if manifest.get('case_plan'):heading='# FORGE Phase 2B depth-dependent boundary search'
        if manifest.get('mode')!='collect':
            status=f"Status: **{manifest['status']}**. {len(accepted)} / {manifest['pilot_candidate_count']} pilot cases passed the initial numerical screen. Full collection has not started."
    lines=[heading,'',status,'',
        'A valid trajectory need not insert successfully or remain within force budgets. Numerical rejects stay in the dataset; retries keep their family and direction.','',
        '| Family | Completed attempts | Numerically valid slots |', '| --- | ---: | ---: |']
    for family in ('centered','x_offset','y_offset','diagonal_offset','tilt_only','offset_tilt'):
        rows=[r for r in trajectories if r['family']==family and r['status']=='complete']
        lines.append(f"| {family} | {len(rows)} | {len({r['slot'] for r in rows if r['numerically_valid']})} |")
    if gap_study:
        heading='# FORGE clearance and insertion-triggered tilt study'
        lines[0]=heading
        lines += ['', 'Only hole geometry and tilt timing/amplitude vary. Tilt starts after an actual-depth crossing, then ramps with elapsed time; a stall does not freeze the command ramp. All force costs are recorded-motion costs, including failed probes.', '']
    elif manifest.get('case_plan'):
        lines += ['', 'Misalignment ramps with the maximum actual depth reached on the previous physics step, starting at 5 or 10 mm. Terminal probes test the final insertion/hold state as well as fixed-depth checkpoints. A terminal safe recovery after a stall does not establish safe further insertion progress.', '',
            'Group paired amplitudes and all branches by `split_group_id`. Shared aligned prefixes across paths also need duplicate-history handling before ML evaluation. Unreached checkpoints are unknown; zero requires valid failures of both tested recovery policies.', '']
    elif manifest.get('planned_family_quotas'):
        lines += ['', 'Planned valid-slot quotas: '+', '.join(f'{k}: {v}' for k,v in manifest['planned_family_quotas'].items())+'.', '',
            'Centered trajectories are repeated controls, not independent configurations. Split by `split_group_id` across trajectories, checkpoints and recovery probes: all centered copies share one group, and each contact-rich slot retains its group across retries and branches. These fields do not automatically enforce a downstream ML split. Report controls separately or deduplicate them when evaluating performance.', '']
    eligible_labels=[r for r in checkpoints if r['parent_numerically_valid'] and r['parent_complete']]
    lines += ['', f"Eligible-parent checkpoint labels: {sum(r['Y_R_tested']==1 for r in eligible_labels)} safe witnesses, {sum(r['Y_R_tested']==0 for r in eligible_labels)} tested-policy failures, {sum(r['Y_R_tested'] is None for r in eligible_labels)} unknown. {len(checkpoints)-len(eligible_labels)} checkpoint records belong to invalid or incomplete parents.", '', '## Labels and numerical limits','',
        '`Y_R_tested = 1` is a safe-policy witness. Zero means both tested policies failed valid matched-state tests, not that all recovery motions are impossible. Null means unknown. Labels are recorded only at the tested checkpoints; no continuous recoverability boundary is inferred.','',
        'Branches replay the insertion prefix and compare observable state, velocity and wrench against the original checkpoint. Hidden contact/solver memory is not restored or certified equivalent. Unmatched probes cannot provide labels.','',
        'The central plot shows invalid-parent or unfinished-parent points as unknown. A passed overlap screen is a first numerical acceptance criterion, not proof of convergence.','',
        '[Trajectory labels](trajectories.csv) · [Checkpoint features and labels](checkpoints.csv) · [Policy tests](recovery_probes.csv) · [Protocol and ledger](study.json)','',
        '![Recovery characterization](recovery_characterization.png)','']
    (directory/'report.md').write_text('\n'.join(lines))
    if gap_study:
        _write_gap_report(directory,manifest,trajectories,checkpoints,probes,plt)


def _write_gap_report(directory,manifest,trajectories,checkpoints,probes,plt):
    """One row per planned case and per checkpoint; unknown pairs stay visible."""
    planned=(manifest.get('case_plan') or {}).get('cases') or manifest['attempts']
    cases=[];seen=set()
    for case in planned:
        key=case.get('case_id',case.get('slot',case.get('trajectory_id')))
        if key in seen:continue
        seen.add(key)
        cases.append({k:case[k] for k in ('trajectory_id','slot','family','insertion_duration_s',*FACTOR_FIELDS) if k in case})
    table(directory/'cases.csv',cases)
    by_checkpoint={}
    for probe in probes:
        by_checkpoint.setdefault((probe['trajectory_id'],probe['checkpoint_id']),{})[probe['policy']]=probe
    comparisons=[]
    keys=('label_eligible','replay_matched','safe_recovery','reason','termination_reason','recovery_censored','termination_phase',
          'numerically_valid','grasp_retained','cleared','force_budget_exceeded','torque_budget_exceeded',
          'max_wrist_force_n','max_wrist_torque_nm','duration_s','clear_time_s',
          'stop_peak_wrist_force_n','realign_peak_wrist_force_n','retreat_peak_wrist_force_n',
          'retreat_peak_wrist_force_mean100ms_n','retreat_peak_wrist_world_z_abs_n',
          'retreat_peak_wrist_world_z_abs_mean100ms_n','retreat_peak_contact_downward_resistance_n',
          'retreat_actual_withdrawal_mm','retreat_commanded_withdrawal_mm','retreat_contact_resistive_work_j')
    for cp in checkpoints:
        row={k:cp.get(k) for k in ('trajectory_id','checkpoint_id','checkpoint_kind','checkpoint_depth_mm',
             'depth_mm','reached','parent_numerically_valid','parent_complete','Y_R_tested','label_reason',
             'parent_insertion_success','parent_stalled',
             'tilt_deg','command_tilt_deg','wrist_force_n',*FACTOR_FIELDS)}
        pair=by_checkpoint.get((cp['trajectory_id'],cp['checkpoint_id']),{})
        for policy in ('straight','realign'):
            probe=pair.get(policy,{})
            row[f'{policy}_probe_present']=bool(probe)
            row.update({f'{policy}_{key}':probe.get(key) for key in keys})
        row['paired_label_eligible']=all(pair.get(policy,{}).get('label_eligible',False) for policy in ('straight','realign'))
        row['paired_measured']=all(pair.get(policy,{}).get('max_wrist_force_n') is not None for policy in ('straight','realign'))
        comparisons.append(row)
    table(directory/'comparison.csv',comparisons)
    fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
    colors={1:'#13866b',0:'#b23f4c',None:'#8b98a1'}
    plotted=0
    for cp in comparisons:
        x=cp.get('straight_max_wrist_force_n');y=cp.get('realign_max_wrist_force_n')
        if x is None or y is None:continue
        eligible=cp['paired_label_eligible'] and cp['parent_numerically_valid'] and cp['parent_complete']
        label=cp['Y_R_tested'] if eligible else None
        axes[0].scatter(x,y,c=colors[label],marker='o' if eligible else 'x',alpha=.75)
        plotted+=1
    axes[0].set(xlabel='Straight: whole-recovery wrist peak [N]',ylabel='Realign: whole-recovery wrist peak [N]',
                title=f'Paired recorded probes ({plotted}/{len(comparisons)} checkpoints)')
    upper=max([1.,*[p.get('max_wrist_force_n') or 0. for p in probes]])
    axes[0].plot([0,upper],[0,upper],':',color='#8b98a1',lw=1)
    for policy,marker in [('straight','o'),('realign','^')]:
        for value,label in [(1,'safe witness'),(0,'tested failures'),(None,'unknown')]:
            selected=[p for p in probes if p.get('policy')==policy and p.get('retreat_peak_contact_downward_resistance_n') is not None and
                (1 if p.get('safe_recovery') else 0 if p.get('safe_recovery') is False else None)==value]
            axes[1].scatter([p.get('radial_clearance_mm') for p in selected],
                [p['retreat_peak_contact_downward_resistance_n'] for p in selected],
                marker=marker,c=colors[value],alpha=.7,label=f'{policy}: {label}')
    axes[1].set(xlabel='Nominal radial clearance [mm]',ylabel='Recorded retreat downward contact resistance [N]',
                title='Retreat phase only; absent phases have no cost')
    axes[1].legend(fontsize=7)
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(directory/'gap_recovery_comparison.png',dpi=160);plt.close(fig)
    heatmap_written=_write_gap_heatmap(directory,cases,comparisons,plt)
    counts={}
    for row in trajectories:
        counts[row.get('radial_clearance_mm')]=counts.get(row.get('radial_clearance_mm'),[])+[row]
    lines=['','## Clearance × tilt experiment','',
        '[Planned cases](cases.csv) · [Paired checkpoint comparison](comparison.csv)','',
        '| Nominal radial clearance [mm] | Measured references | Numerically valid | Insertion success | Triggered / applicable | Ramp completed |',
        '| ---: | ---: | ---: | ---: | ---: | ---: |']
    for gap,rows in sorted(counts.items(),key=lambda item:float(item[0] or 0)):
        lines.append(f"| {gap} | {len(rows)} | {sum(bool(r.get('numerically_valid')) for r in rows)} | {sum(bool(r.get('insertion_success')) for r in rows)} | {sum(bool(r.get('tilt_triggered')) for r in rows)} / {sum(bool(r.get('tilt_applicable')) for r in rows)} | {sum(bool(r.get('tilt_ramp_completed')) for r in rows)} |")
    lines += ['', 'The comparison table contains every checkpoint, including unreached, unmatched, failed, invalid and incomplete records. A missing cost is not zero force. A failed/censored probe reports only the observed part of motion, not the force required to complete withdrawal.', '',
        'Force definitions: operational safety uses whole-recovery raw wrist force/torque norms. `wrist_world_z_abs` is the absolute world-Z component of the raw wrist reaction; it is not gravity-compensated or calibrated as external extraction force. World +Z is the fixed-hole withdrawal axis in this setup. `contact_downward_resistance` is max(0, −contact Fz), the peg/socket force opposing upward withdrawal. These signals must not be substituted for one another.', '',
        'The 100 ms moving means use complete windows within each phase, never stop-to-retreat mixtures. Short phases have no full-window value. Retreat displacement is signed progress from the state immediately before retreat; force without progress remains visible. Root-level recovery labels retain full-process budget, grasp, clearance and numerical checks.', '',
        '`trajectories.csv` also preserves `continuation_straight_*`: the original reference followed directly by straight withdrawal, without reset/replay. This evidence remains available when later checkpoint replays mismatch. It is a separate observation and never replaces a missing matched policy in `comparison.csv` or changes its recovery label.', '',
        f'{len(comparisons)-plotted} checkpoints lack two measured whole-recovery costs and are absent from the paired scatter; all remain in `comparison.csv`. Green: safe witness; red: both tested policies fail; grey: unknown/ineligible pair. No safe-only filter is applied.', '',
        '![Gap-study recovery comparison](gap_recovery_comparison.png)','']
    if heatmap_written:
        lines += ['### Terminal-state direct withdrawal by factors','',
            'Each cell is an observed terminal replay straight-withdrawal wrist peak, restricted to the retreat phase; continuous final withdrawals are not included. Cells do not interpolate across configurations. S = safe complete recovery; F = valid tested failure; U = unknown/ineligible; ? = checkpoint exists but no retreat cost; dash = planned combination with no recorded terminal checkpoint; N/A = combination absent from the plan. Failed/censored values are partial-motion costs. Aligned controls remain in the tables and are excluded from the angle grid, including clearance rows with controls only. If a case has retries, the latest recorded attempt is shown; all attempts remain in the tables.', '',
            '![Terminal direct-withdrawal factor matrix](gap_terminal_force_matrix.png)','']
    with (directory/'report.md').open('a') as stream:stream.write('\n'.join(lines))


def _write_gap_heatmap(directory,cases,comparisons,plt):
    import numpy as np
    tilted=[c for c in cases if c.get('tilt_amplitude_deg',0)>0]
    onsets=sorted({c['tilt_onset_fraction'] for c in tilted if c.get('tilt_onset_fraction') is not None})
    gaps=sorted({c['radial_clearance_mm'] for c in tilted if c.get('radial_clearance_mm') is not None},reverse=True)
    angles=sorted({c['tilt_amplitude_deg']*c.get('tilt_sign',1) for c in tilted})
    planned={(c.get('radial_clearance_mm'),c.get('tilt_onset_fraction'),
              c['tilt_amplitude_deg']*c.get('tilt_sign',1)) for c in tilted}
    if not onsets or not gaps or not angles:return False
    terminal={}
    for cp in comparisons:
        if cp['checkpoint_kind']!='terminal' or not cp.get('tilt_amplitude_deg'):continue
        terminal[(cp.get('radial_clearance_mm'),cp.get('tilt_onset_fraction'),
                  cp['tilt_amplitude_deg']*cp.get('tilt_sign',1))]=cp
    maximum=max([1.,*[cp.get('straight_retreat_peak_wrist_force_n') or 0. for cp in terminal.values()]])
    fig,axes=plt.subplots(1,len(onsets),figsize=(max(6,5*len(onsets)),5),squeeze=False,constrained_layout=True)
    for onset,ax in zip(onsets,axes[0]):
        values=np.full((len(gaps),len(angles)),np.nan)
        annotations=[]
        for i,gap in enumerate(gaps):
            for j,angle in enumerate(angles):
                if (gap,onset,angle) not in planned:annotations.append((i,j,'N/A'));continue
                cp=terminal.get((gap,onset,angle))
                if cp is None:annotations.append((i,j,'–'));continue
                value=cp.get('straight_retreat_peak_wrist_force_n')
                eligible=(cp.get('straight_label_eligible') and cp.get('parent_numerically_valid') and cp.get('parent_complete'))
                outcome=cp.get('straight_safe_recovery') if eligible else None
                tag='S' if outcome is True else 'F' if outcome is False else 'U'
                if value is None:annotations.append((i,j,f'? {tag}'));continue
                values[i,j]=value;annotations.append((i,j,f'{value:.2f}\n{tag}'))
        cmap=plt.get_cmap('YlOrRd').copy();cmap.set_bad('#edf0f2')
        im=ax.imshow(values,vmin=0,vmax=maximum,cmap=cmap,aspect='auto',interpolation='nearest')
        for i,j,label in annotations:
            value=values[i,j]
            ax.text(j,i,label,ha='center',va='center',fontsize=8,color='white' if np.isfinite(value) and value>.65*maximum else '#26333a')
        ax.set_xticks(range(len(angles)),[f'{angle:g}' for angle in angles])
        ax.set_yticks(range(len(gaps)),[f'{gap:g}' for gap in gaps])
        ax.set(xlabel='Signed target pitch [deg]',ylabel='Nominal radial clearance [mm]',title=f'Tilt starts at {onset:.0%} actual depth')
    fig.colorbar(im,ax=list(axes[0]),label='Recorded terminal replay straight-retreat wrist peak [N]',shrink=.85)
    fig.savefig(directory/'gap_terminal_force_matrix.png',dpi=160);plt.close(fig)
    return True
