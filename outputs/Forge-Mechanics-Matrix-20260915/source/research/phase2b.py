"""Depth-dependent FORGE paths and staged, bounded recovery-boundary search."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path

SCHEMA='Forge-Phase2B-depth-drift-v1'
COMMAND_KEYS=('offset_x_mm','offset_y_mm','roll_deg','pitch_deg')


def make_case(onset,axis,sign,mode,severity):
    # Severity is the tilt endpoint in degrees; drift endpoint scales to 1 mm
    # at severity 6. These are separate controlled paths, not a random box.
    x=y=roll=pitch=0.
    if mode in ('drift','combined'):
        if axis=='x_pitch':x=sign*severity/6
        else:y=sign*severity/6
    if mode in ('tilt','combined'):
        if axis=='x_pitch':pitch=sign*severity
        else:roll=sign*severity
    family='offset_tilt' if mode=='combined' else 'tilt_only' if mode=='tilt' else 'x_offset' if axis=='x_pitch' else 'y_offset'
    group=f'b_d{onset:g}_{axis}_{sign:+d}_{mode}'
    return dict(family=family,offset_x_mm=0.,offset_y_mm=0.,roll_deg=0.,pitch_deg=0.,
        insertion_duration_s=8.,ramp_onset_mm=float(onset),ramp_full_depth_mm=20.,
        final_offset_x_mm=x,final_offset_y_mm=y,final_roll_deg=roll,final_pitch_deg=pitch,
        severity_deg=float(severity),path_axis=axis,path_sign=sign,path_mode=mode,
        sample_role='repeatability_control' if severity==0 else 'boundary_characterization',
        split_group_id='phase2b_centered_controls' if severity==0 else group,path_group_id=group)


def plan(cases,provenance=None):
    return dict(schema=SCHEMA,depth_feedback='maximum_reached_depth_previous_step',probe_terminal=True,
        provenance=provenance or [],cases=[dict(c,slot=i,retry=0,trajectory_id=f'b{i:03d}_try00') for i,c in enumerate(cases)])


def initial_plan(pilot=False):
    if pilot:
        cases=[make_case(5,'x_pitch',1,'combined',3),make_case(10,'y_roll',-1,'combined',6)]
    else:
        cases=[make_case(d,axis,s,mode,3) for d in (5,10) for axis in ('x_pitch','y_roll')
               for s in (1,-1) for mode in ('tilt','drift','combined')]
    return plan(cases)


def load_plan(path):
    value=json.loads(Path(path).read_text())
    if value.get('schema')!=SCHEMA or value.get('depth_feedback')!='maximum_reached_depth_previous_step':
        raise ValueError('Unsupported Phase 2B plan or depth feedback')
    if value.get('probe_terminal') is not True or not value.get('cases'):
        raise ValueError('Phase 2B needs cases and terminal recovery probes')
    for i,c in enumerate(value['cases']):
        if c.get('slot')!=i or c.get('trajectory_id')!=f'b{i:03d}_try00' or c.get('retry')!=0:
            raise ValueError('Phase 2B plan must have consecutive unique slots')
        if (c.get('ramp_onset_mm') not in (5.,10.) or c.get('path_axis') not in ('x_pitch','y_roll')
                or c.get('path_sign') not in (-1,1) or c.get('path_mode') not in ('tilt','drift','combined')
                or not isinstance(c.get('severity_deg'),(float,int))
                or not math.isfinite(c['severity_deg']) or not 0<=c['severity_deg']<=6):
            raise ValueError('Invalid bounded Phase 2B path')
        expected=make_case(c['ramp_onset_mm'],c['path_axis'],c['path_sign'],c['path_mode'],c['severity_deg'])
        if any(c.get(k)!=v for k,v in expected.items()):
            raise ValueError('Phase 2B case fields disagree with its path parameters')
    return value


def command_misalignment(case,reached_depth):
    if 'ramp_onset_mm' not in case:return {k:case[k] for k in COMMAND_KEYS}
    fraction=max(0.,min(1.,(reached_depth-case['ramp_onset_mm'])/
                           (case['ramp_full_depth_mm']-case['ramp_onset_mm'])))
    return {k:case[k]+fraction*(case['final_'+k]-case[k]) for k in COMMAND_KEYS}


def case_for_slot(case_plan,slot,retry):
    return dict(case_plan['cases'][slot],retry=retry,trajectory_id=f'b{slot:03d}_try{retry:02d}')


def checkpoint_label(a,cp):
    if a.get('status')!='complete' or not a.get('metrics',{}).get('numerically_valid'):
        return None
    if not cp.get('reached') or not cp.get('prefix_numerically_valid'):return None
    return cp.get('Y_R_tested')


def path_outcome(a):
    """Unknown/unreached endpoints never become a negative recovery label."""
    cps=a.get('checkpoints',[])
    if any(checkpoint_label(a,c)==0 for c in cps):return 'tested_failure'
    terminals=[c for c in cps if c.get('checkpoint_kind')=='terminal']
    return 'safe_terminal' if terminals and checkpoint_label(a,terminals[-1])==1 else 'unknown'


def recommend(attempts,resolution=.25):
    groups=defaultdict(list)
    for a in attempts:groups[a['path_group_id']].append(a)
    candidates=[];notes=[]
    for group,rows in sorted(groups.items()):
        # Combine repeats conservatively. Contradictory labels require review.
        levels=defaultdict(set)
        for a in rows:levels[a['severity_deg']].add(path_outcome(a))
        safe=sorted(x for x,s in levels.items() if 'safe_terminal' in s and 'tested_failure' not in s)
        failed=sorted(x for x,s in levels.items() if 'tested_failure' in s)
        chosen=None;reason=''
        if any('tested_failure' in s and 'safe_terminal' in s for s in levels.values()):
            reason='Conflicting repeats: review; no automatic escalation.'
        elif failed and safe and max(safe)>min(failed):
            reason='Nonmonotone observed path: review; do not assume a single boundary.'
        elif failed:
            high=min(failed);lower=[x for x in safe if x<high]
            if lower:
                low=max(lower)
                if high-low>resolution:chosen=(low+high)/2;reason=f'Refine local candidate bracket [{low:g}, {high:g}].'
                else:reason=f'Local candidate bracket [{low:g}, {high:g}] reached resolution; not a global guarantee.'
            elif high>0:
                chosen=0.;reason='Test an aligned endpoint control before claiming a bracket.'
            else:reason='Failure even at zero amplitude: review preparation and policy.'
        elif safe:
            high=max(safe)
            if high<6:chosen=min(6.,high+1);reason='Advance one bounded ladder step after an observed safe terminal recovery.'
            else:reason='Safe at configured ceiling; no negative found. Do not widen automatically.'
        else:reason='Unknown only: inspect numerical/replay failures; do not escalate.'
        if chosen is not None and chosen in levels:
            chosen=None;reason='Candidate already tested but unresolved; review it instead of skipping uncertainty.'
        notes.append(dict(path_group_id=group,reason=reason,next_severity_deg=chosen))
        if chosen is not None:
            a=rows[0];c=make_case(a['ramp_onset_mm'],a['path_axis'],a['path_sign'],a['path_mode'],chosen)
            candidates.append(c)
    return candidates,notes


def comparison_sources(manifest,directory):
    """Allow report-only edits across stages, while checking all archived runtime code."""
    import ast
    sources=dict(manifest.get('sources',{}))
    if 'phase2b.py' not in sources:return sources
    archived=Path(directory)/'source/phase2b.py'
    if hashlib.sha256(archived.read_bytes()).hexdigest()!=sources['phase2b.py']:
        raise ValueError('Archived Phase 2B source hash mismatch')
    tree=ast.parse(archived.read_text())
    analysis_only={'checkpoint_label','path_outcome','recommend','comparison_sources','analyze','main'}
    # Retain imports, constants, path functions, and any new runtime helpers.
    tree.body=[n for n in tree.body if not (isinstance(n,ast.FunctionDef) and n.name in analysis_only)
               and not (isinstance(n,ast.If) and ast.unparse(n.test)=="__name__ == '__main__'")]
    sources['phase2b.py']=hashlib.sha256(ast.dump(tree).encode()).hexdigest()
    return sources


def analyze(studies,output):
    attempts=[];sources=[];settings=None
    for directory in studies:
        path=Path(directory)/'study.json';m=json.loads(path.read_text())
        if (m.get('case_plan') or {}).get('schema')!=SCHEMA:raise ValueError(f'{directory} is not Phase 2B')
        fingerprint={k:m.get(k) for k in ('protocol','physics_hz','sources','stock_buffers','requested_checkpoints_mm')}
        fingerprint['sources']=comparison_sources(m,directory)
        # The number of candidates is allowed to vary between search stages.
        fingerprint['protocol']=dict(fingerprint['protocol']);fingerprint['protocol'].pop('target_valid',None)
        if settings is not None and settings!=fingerprint:raise ValueError('Cannot pool changed physics/protocol/source versions')
        settings=fingerprint
        attempts.extend(dict(a,source_study=str(Path(directory).resolve())) for a in m['attempts'])
        sources.append(dict(study=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    candidates,notes=recommend(attempts)
    transitions=[]
    for a in attempts:
        observed=sorted((c for c in a['checkpoints'] if checkpoint_label(a,c) is not None),
                        key=lambda c:c['state']['time_s'])
        for i,c in enumerate(observed):
            if checkpoint_label(a,c)!=0:continue
            earlier=[x for x in observed[:i] if checkpoint_label(a,x)==1 and x['state']['depth_mm']<c['state']['depth_mm']]
            if earlier:
                prev=earlier[-1]
                transitions.append(dict(source_study=a['source_study'],trajectory_id=a['trajectory_id'],folder=a['folder'],path_group_id=a['path_group_id'],
                    safe_depth_mm=prev['state']['depth_mm'],failed_depth_mm=c['state']['depth_mm'],
                    safe_time_s=prev['state']['time_s'],failed_time_s=c['state']['time_s'],
                    failure_already_over_budget=c['state']['wrist_force_n']>settings['protocol']['force_budget_n']
                        or c['state']['wrist_torque_nm']>settings['protocol']['torque_budget_nm']))
                break
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    result=dict(sources=sources,analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                transitions=transitions,recommendations=notes,
                outcomes={k:sum(path_outcome(a)==k for a in attempts) for k in ('safe_terminal','tested_failure','unknown')})
    (output/'boundary.json').write_text(json.dumps(result,indent=2)+'\n')
    if candidates:(output/'next_plan.json').write_text(json.dumps(plan(candidates,sources),indent=2)+'\n')
    lines=['# Phase 2B observed recovery boundary','',f'{len(transitions)} within-trajectory safe-to-failed depth transitions observed.',
           '',f"Outcomes: {result['outcomes']}",'',
           'Only reached, numerically screened, matched-policy tests supply labels. Unreached depths and unknown tests are not failures. Candidate brackets compare paths with different contact histories and do not certify monotonicity or a continuous boundary.', '',
           '| Path | Next step |','| --- | --- |']
    lines.extend(f"| {n['path_group_id']} | {n['reason']} |" for n in notes)
    lines+=['','Run the next plan as a separate study, then analyze all stages together. No simulation is started by this analysis.','']
    (output/'report.md').write_text('\n'.join(lines))
    print(json.dumps(result['outcomes']),f'; transitions: {len(transitions)}; next cases: {len(candidates)}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pilot',action='store_true')
    parser.add_argument('--from-study',type=Path,nargs='+')
    args=parser.parse_args()
    if args.from_study:analyze(args.from_study,args.output)
    else:
        with args.output.open('x') as f:json.dump(initial_plan(args.pilot),f,indent=2);f.write('\n')
        load_plan(args.output)

if __name__=='__main__':main()
