"""Phase 2 dataset tables and checkpoint characterization plots."""
import csv
from pathlib import Path


def table(path,rows):
    if not rows:return
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)


def write_phase2_report(directory,manifest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory)
    trajectories=[];checkpoints=[];probes=[]
    for a in manifest['attempts']:
        if 'metrics' not in a:continue
        base={k:a[k] for k in ('trajectory_id','slot','retry','family','offset_x_mm','offset_y_mm','roll_deg','pitch_deg','insertion_duration_s','folder','status')}
        grouping={k:a[k] for k in ('sampling_slot','sample_role','split_group_id') if k in a}
        base.update(grouping)
        trajectories.append({**base,**a['metrics']})
        for cp in a['checkpoints']:
            state=cp.get('state',{})
            row=dict(trajectory_id=a['trajectory_id'],folder=a['folder'],family=a['family'],slot=a['slot'],
                parent_numerically_valid=a['metrics']['numerically_valid'],parent_complete=a['status']=='complete',
                checkpoint_depth_mm=cp['depth_mm'],reached=cp['reached'],reference_time_s=state.get('time_s'),
                prefix_numerically_valid=cp['prefix_numerically_valid'],Y_R_tested=cp.get('Y_R_tested'),label_reason=cp.get('label_reason'),
                depth_mm=state.get('depth_mm'),force_n=state.get('force_norm_n'),torque_nm=state.get('torque_norm_nm'),
                normal_load_n=state.get('normal_load_n'),tilt_deg=state.get('tilt_deg'),
                tip_x_mm=state.get('tip_x_mm'),tip_y_mm=state.get('tip_y_mm'),
                prefix_max_force_n=cp.get('prefix_max_force_n'),prefix_max_torque_nm=cp.get('prefix_max_torque_nm'),
                prefix_max_normal_load_n=cp.get('prefix_max_normal_load_n'),prefix_max_penetration_mm=cp.get('prefix_max_penetration_mm'))
            row.update({k:state.get(k) for k in ('fx','fy','fz','taux','tauy','tauz','vx','vy','vz','omegax','omegay','omegaz','qw','qx','qy','qz')})
            for i in range(1,8):
                for k in (f'joint{i}_rad',f'joint_velocity{i}_rad_s'):row[k]=state.get(k)
            row.update(grouping)
            checkpoints.append(row)
            for p in cp['probes']:
                flat={k:v for k,v in p.items() if not isinstance(v,dict)}
                flat.update({f'replay_error_{k}':v for k,v in p.get('replay_errors',{}).items()})
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
        selected=[p for p in probes if p['policy']==policy and p['label_eligible'] and p.get('safe_recovery')]
        axes[1].scatter([p['depth_mm'] for p in selected],[p['max_force'] for p in selected],marker=marker,label=policy,alpha=.8)
    axes[1].set(xlabel='Checkpoint depth [mm]',ylabel='Peak force over successful recovery [N]',title='Observed safe-policy force costs')
    axes[1].legend(fontsize=8)
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(directory/'recovery_characterization.png',dpi=160)
    fig.savefig(directory/'recovery_characterization.pdf');plt.close(fig)
    accepted={r['slot'] for r in trajectories if r['status']=='complete' and r['numerically_valid']}
    heading='# Phase 2 randomized insertion characterization'
    status=f"Status: **{manifest['status']}**. {len(accepted)} / {manifest['protocol']['target_valid']} numerically valid trajectory slots filled."
    if manifest.get('study')=='Forge-Controlled-Phase2-v1':
        heading='# FORGE randomized insertion collection' if manifest.get('mode')=='collect' else '# Controlled FORGE pilot'
        if manifest.get('mode')!='collect':
            status=f"Status: **{manifest['status']}**. {len(accepted)} / {manifest['pilot_candidate_count']} pilot cases passed the initial numerical screen. Full collection has not started."
    lines=[heading,'',status,'',
        'A valid trajectory need not insert successfully or remain within force budgets. Numerical rejects stay in the dataset; retries keep their family and direction.','',
        '| Family | Completed attempts | Numerically valid slots |', '| --- | ---: | ---: |']
    for family in ('centered','x_offset','y_offset','diagonal_offset','tilt_only','offset_tilt'):
        rows=[r for r in trajectories if r['family']==family and r['status']=='complete']
        lines.append(f"| {family} | {len(rows)} | {len({r['slot'] for r in rows if r['numerically_valid']})} |")
    if manifest.get('planned_family_quotas'):
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
