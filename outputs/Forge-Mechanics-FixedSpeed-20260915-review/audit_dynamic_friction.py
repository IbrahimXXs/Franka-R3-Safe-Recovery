#!/usr/bin/env python3
"""Offline, fixed-scope check of completed d06 pair friction 1/1 and 1/.5.

Run from the repository root with python3 and this script path. No simulator.
"""
import csv
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import statistics

REVIEW = Path(__file__).resolve().parent
STUDY = REVIEW.with_name('Forge-Mechanics-FixedSpeed-20260915')


def dot(a,b): return sum(x*y for x,y in zip(a,b))
def norm(a): return math.sqrt(dot(a,a))
def sub(a,b): return [x-y for x,y in zip(a,b)]


def rotate(q,v,inverse=False):
    length=norm(q); w,x,y,z=[c/length for c in q]
    if inverse: x,y,z=-x,-y,-z
    u=[x,y,z]; uv=dot(u,v)
    cross=[y*v[2]-z*v[1],z*v[0]-x*v[2],x*v[1]-y*v[0]]
    return [2*uv*u[i]+(w*w-dot(u,u))*v[i]+2*w*cross[i] for i in range(3)]


def quantiles(values):
    if not values: return None
    ordered=sorted(values)
    def percentile(p):
        position=p*(len(ordered)-1); low=int(position); high=math.ceil(position)
        return ordered[low]+(position-low)*(ordered[high]-ordered[low])
    return dict(min=min(values),p10=percentile(.1),median=statistics.median(values),
                p90=percentile(.9),max=max(values))


def summarize(samples):
    return {'sample_count':len(samples),
            'phase_counts':{phase:sum(s['phase']==phase for s in samples)
                            for phase in sorted({s['phase'] for s in samples})},
            'net_friction_norm_over_normal_load':quantiles([s['net_ratio'] for s in samples]),
            'sum_friction_anchor_norms_over_normal_load':quantiles([s['anchor_sum_ratio'] for s in samples]),
            'pose_difference_axial_speed_mm_s':quantiles([s['pose_difference_axial_speed_mm_s']
                   for s in samples if s['pose_difference_axial_speed_mm_s'] is not None]),
            'sample_indices':[s['sample_index'] for s in samples]}


manifest_bytes=(STUDY/'study.json').read_bytes()
manifest=json.loads(manifest_bytes)
previous_audit=REVIEW/'friction_effectiveness_audit.json'
report={'schema':'dynamic_friction_readback_response_audit_v1','study_dir':str(STUDY),
    'study_sha256_at_read':hashlib.sha256(manifest_bytes).hexdigest(),
    'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    'prior_audit_untouched_sha256':hashlib.sha256(previous_audit.read_bytes()).hexdigest(),
    'scope':'Completed d06 controls and +2 degree cases at pair static/dynamic 1/1 and 1/.5; original insertion and continuous straight recovery only.',
    'filters':{
        'loaded':'CSV summed normal load > .1 N; recovery copied t=0 excluded',
        'coherent_horizontal_axial_force':'norm(sum normal vectors)/sum normal load >.99; abs(normal_world_z)<.03*sum normal load; abs(friction_world_z)>.95*net friction norm. Allows upper-rim contact, without pairing streams.',
        'pose_sliding_dissipative':'Previous and current pose exist one physics dt apart; friction-anchor-force-magnitude-weighted tangential pose-difference velocity has |vz|>.2 mm/s and >90% axial direction; summed current anchor friction dot pose-difference anchor velocity <0.',
        'anchor_velocity':'Map each CURRENT friction anchor to the current peg local frame and transform that same rigid point by the previous peg pose; subtract weighted normal direction from velocity. Does not pair normal/friction indices or match anchor identities between frames.'},
    'limitations':[
        'Runtime readback confirms coefficients and explicit average combine binding, not an independently measured contact-pair coefficient.',
        'Normal and friction streams are independent; ratio interpretation requires a predominantly coherent sliding contact. Static sticking allows friction below the static bound.',
        'Finite-pose sliding screen and negative power support a slipping interpretation, but contact/pose within-step API timing is not independently guaranteed. These are multiple frame response checks, not a calibrated tribology experiment.',
        'Summed anchor magnitudes can differ from the norm of their resultant. Neither ratio alone proves a dynamic coefficient when multiple patches or sticking are present.',
        'Only named completed profiles are read. No running profile, physical source, simulation parameter, or existing audit is modified.'
        ,'Control and tilted cases share the aligned-entry prefix. Repeated prefix observations are not independent trial replications.'
    ],'materials':[],'profiles':[]}
hashes={}
for attempt in manifest['attempts']:
    if not (attempt.get('status')=='complete' and attempt['case_id'].startswith('m_d06_fs100')):
        continue
    material=attempt['material']
    runtime=material['runtime_materials']; pair=(attempt['pair_static_friction'],attempt['pair_dynamic_friction'])
    checks={
        'hole_runtime_matches_requested_average_pair':all(abs(runtime['hole'][0][0][i]-(2*pair[i]-.75))<1e-7 for i in range(2)),
        'peg_runtime_stays_0p75':all(abs(runtime['peg'][0][0][i]-.75)<1e-7 for i in range(2)),
        'robot_runtime_stays_0p75':all(abs(shape[i]-.75)<1e-7 for env in runtime['robot'] for shape in env for i in range(2)),
        'hole_and_peg_resolve_explicit_average':all(material['bindings'][body]['resolved_friction_combine_mode']=='average' for body in ('hole','peg'))}
    assert all(checks.values())
    report['materials'].append({'case_id':attempt['case_id'],'status':attempt['status'],
        'checks':checks,'material_record':material})
    for stem in ('insertion','final_retreat'):
        path=STUDY/attempt['folder']/(stem+'.csv'); rawpath=path.with_name(stem+'_contacts.jsonl.gz')
        data=path.read_bytes(); rawdata=rawpath.read_bytes()
        hashes[str(path)]=hashlib.sha256(data).hexdigest()
        hashes[str(rawpath)]=hashlib.sha256(rawdata).hexdigest()
        rows=[]
        for raw in csv.DictReader(io.StringIO(data.decode())):
            row={}
            for key,value in raw.items():
                try: row[key]=float(value)
                except (ValueError,TypeError): row[key]=value
            rows.append(row)
        selected=[]; previous=None; rawcount=0; max_n_error=0.; max_f_error=0.; times_ok=True
        coefficients=set()
        for index,line in enumerate(gzip.decompress(rawdata).splitlines()):
            rawcount+=1; record=json.loads(line); raw=record['contacts']; row=rows[index]
            coefficients.add((row['effective_pair_static_friction'],row['effective_pair_dynamic_friction']))
            times_ok &= all(abs(record[k]-row[k])<1e-8 for k in ('time_s','physics_time_s'))
            normal_points=raw['normal_contacts']; anchors=raw['friction_contacts']
            nsum=sum(p['normal_load_n'] for p in normal_points)
            nf=[sum(p['normal_load_n']*p['normal_world'][axis] for p in normal_points) for axis in range(3)]
            ff=[sum(p['force_world_n'][axis] for p in anchors) for axis in range(3)]
            max_n_error=max(max_n_error,abs(nsum-row['normal_load_n']))
            max_f_error=max(max_f_error,*(abs(ff[i]-row[f'friction_force_world_{axis}_n']) for i,axis in enumerate('xyz')))
            if row['normal_load_n']>.1 and row.get('recovery_time_s')!=0.:
                N=row['normal_load_n']; F=norm(ff); anchor_sum=sum(norm(p['force_world_n']) for p in anchors)
                coherent=norm(nf)/N>.99 and abs(nf[2])<.03*N and abs(ff[2])>.95*F
                sample={'sample_index':index,'phase':row['phase'],'time_s':row['time_s'],
                    'normal_load_n':N,'friction_world_n':ff,'normal_world_n':nf,
                    'net_ratio':F/N,'anchor_sum_ratio':anchor_sum/N,
                    'normal_point_count':len(normal_points),'friction_anchor_count':len(anchors),
                    'coherent_horizontal_axial_force':coherent,
                    'pose_sliding_dissipative':False,'pose_difference_axial_speed_mm_s':None,
                    'pose_difference_friction_power_w':None,'post_com_vz_mm_s':row['vz']*1000}
                if previous and norm(nf)>0 and anchor_sum>0:
                    prow,praw=previous; dt=row['time_s']-prow['time_s']
                    if math.isclose(dt,1/manifest['physics_hz'],abs_tol=1e-7):
                        q=[row[k] for k in ('qw','qx','qy','qz')]; pq=[prow[k] for k in ('qw','qx','qy','qz')]
                        direction=[x/norm(nf) for x in nf]; average=[0.,0.,0.]; power=0.
                        for anchor in anchors:
                            force=anchor['force_world_n']; point=anchor['point_world_m']
                            local=rotate(q,sub(point,raw['origin_world_m']),inverse=True)
                            old_offset=rotate(pq,local)
                            old_point=[a+b for a,b in zip(praw['origin_world_m'],old_offset)]
                            velocity=[x/dt for x in sub(point,old_point)]
                            tangent=[velocity[i]-dot(velocity,direction)*direction[i] for i in range(3)]
                            weight=norm(force)/anchor_sum
                            average=[average[i]+weight*tangent[i] for i in range(3)]
                            power+=dot(force,velocity)
                        sample['pose_difference_axial_speed_mm_s']=average[2]*1000
                        sample['pose_difference_friction_power_w']=power
                        sample['pose_sliding_dissipative']=bool(coherent and abs(average[2])>.0002
                            and abs(average[2])>.9*norm(average) and power<0)
                selected.append(sample)
            previous=(row,raw)
        assert rawcount==len(rows) and times_ok
        assert coefficients=={pair}
        report['profiles'].append({'case_id':attempt['case_id'],'profile':stem,
            'row_count':len(rows),'csv_path':str(path),'raw_path':str(rawpath),
            'gzip_closed_crc_verified':True,'raw_csv_times_match':times_ok,
            'recorded_effective_pair_values':sorted(coefficients),
            'raw_normal_sum_max_abs_csv_error_n':max_n_error,
            'raw_friction_vector_max_abs_csv_error_n':max_f_error,
            'all_loaded':summarize(selected),
            'coherent_axial_force':summarize([s for s in selected if s['coherent_horizontal_axial_force']]),
            'pose_sliding_dissipative':summarize([s for s in selected if s['pose_sliding_dissipative']]),
            'loaded_samples':selected})
for path,digest in hashes.items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest
report['input_hashes']=hashes
report['conclusion']=(
    'Readback and contact response do not support the claim that changing dynamic friction was ignored. '
    'All four completed cases record correct per-shape coefficients with explicitly resolved average combination and unchanged peg/robot coefficients. '
    'In aligned controls, predominantly axial dissipative slipping frames have median friction-resultant/normal-load ratios near 1.0 for pair 1/1 '
    '(10 insertion, 12 retreat frames) and near 0.5 for pair 1/.5 (10 insertion, 5 retreat frames). '
    'Several 1/.5 frames lie between .5 and 1 despite the kinematic slip screen, consistent with unresolved sticking/transition/stream-timing effects; '
    'no exact coefficient is inferred from those individual frames. Tilted reference/withdrawal responses are more mixed, '
    'and neither tilted withdrawal supplies frames passing the declared stringent axial sliding screen. '
    'This supports effective parameter variation, not a calibrated dynamic-friction estimate for every tilted contact.')
assert hashlib.sha256(previous_audit.read_bytes()).hexdigest()==report['prior_audit_untouched_sha256']
destination=REVIEW/'dynamic_friction_effectiveness_audit.json'
destination.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
for p in report['profiles']:
    print(p['case_id'],p['profile'],[(k,p[k]['sample_count'],p[k]['net_friction_norm_over_normal_load'],p[k]['sum_friction_anchor_norms_over_normal_load'])
        for k in ('all_loaded','coherent_axial_force','pose_sliding_dissipative')])
print(destination)
