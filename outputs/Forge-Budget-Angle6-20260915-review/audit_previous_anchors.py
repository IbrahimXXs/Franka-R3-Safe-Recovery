"""Compare only sealed Angle6 anchor branches with the preceding budget study.

This script never starts a simulator, changes archived sources, or reads a
running branch's run.json/CSV. Unclosed branches remain pending. Source checks
use the archived bytes of both studies, not the current workspace versions.
"""
import argparse
import ast
import copy
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from research.forge_mechanics_protocol import compare_prefixes

EXPECTED_CHANGED = {
    'research/forge_mechanics_plan.py', 'research/forge_budget.py',
    'simulation/forge_budget.py', 'simulation/run_budget_study.py',
}
BUDGET_SOURCE_FILES = {
    'research/forge_budget.py', 'simulation/forge_budget.py',
    'simulation/launch_forge_budget.py', 'simulation/run_budget_study.py', 'forge_budget.sh',
}
TARGETS = (('m_d18_fs100_fd100_a1p5__b04','straight'),
           ('m_d18_fs100_fd100_a1p5__b04','realign'),
           ('m_d18_fs100_fd100_a02__b04','straight'))
TIME_COLUMNS = {'time_s','physics_time_s'}
TERMINAL_FIELDS = ('reference_step','phase','time_s','recovery_time_s','command_depth_mm',
    'command_pitch_deg','command_tilt_deg','command_withdrawal_mm','command_withdrawal_speed_mm_s',
    'depth_mm','tilt_deg','wrist_force_n','wrist_torque_nm','normal_load_n',
    'grasp_slip_mm','grasp_slip_deg','min_separation_mm')


def sha(data):return hashlib.sha256(data).hexdigest()


def read(path, files, expected=None):
    data=path.read_bytes();digest=sha(data)
    if expected is not None and digest != expected:raise ValueError(f'Input SHA mismatch: {path}')
    files[str(path)]=digest
    return data


def inside(parent, relative):
    path=(parent/relative).resolve()
    if parent not in path.parents:raise ValueError(f'Archive path escapes its directory: {path}')
    return path


def archived_sources(directory, manifest, files):
    return {name:read(inside(directory/'source',name),files,digest)
            for name,digest in manifest['sources'].items()}


def source_comparison(old_bytes,new_bytes):
    old_keys,new_keys=set(old_bytes),set(new_bytes)
    changed=sorted(name for name in old_keys&new_keys if old_bytes[name]!=new_bytes[name])
    mechanical=old_keys-BUDGET_SOURCE_FILES
    remaining=mechanical-{'research/forge_mechanics_plan.py'}
    rows=[dict(path=name,old_sha256=sha(old_bytes[name]),new_sha256=sha(new_bytes[name]),
               bytes_equal=old_bytes[name]==new_bytes[name]) for name in sorted(old_keys&new_keys)]
    old_ast=ast.parse(old_bytes['research/forge_mechanics_plan.py'])
    new_ast=ast.parse(new_bytes['research/forge_mechanics_plan.py'])
    dump=lambda node:ast.dump(node,include_attributes=False)
    get=lambda tree,name:next(node for node in tree.body if getattr(node,'name',None)==name)
    state_equal=dump(get(old_ast,'MechanicsState'))==dump(get(new_ast,'MechanicsState'))
    extensions=[node for node in new_ast.body if isinstance(node,ast.Assign)
                and any(isinstance(target,ast.Name) and target.id=='EXTENDED_TILT_AMPLITUDES_DEG'
                        for target in node.targets)]
    values=ast.literal_eval(extensions[0].value) if len(extensions)==1 else None

    class RemoveAllowlist(ast.NodeTransformer):
        def __init__(self):self.removed=0
        def visit_Tuple(self,node):
            node=self.generic_visit(node)
            kept=[]
            for element in node.elts:
                if (isinstance(element,ast.Starred) and isinstance(element.value,ast.Name)
                        and element.value.id=='EXTENDED_TILT_AMPLITUDES_DEG'):
                    self.removed+=1
                else:kept.append(element)
            node.elts=kept
            return node

    stripped=copy.deepcopy(new_ast)
    stripped.body=[node for node in stripped.body if not (isinstance(node,ast.Assign)
        and any(isinstance(target,ast.Name) and target.id=='EXTENDED_TILT_AMPLITUDES_DEG' for target in node.targets))]
    remover=RemoveAllowlist();stripped=remover.visit(stripped)
    case_equal=dump(get(old_ast,'make_case'))==dump(get(stripped,'make_case'))
    module_equal=dump(old_ast)==dump(stripped)
    expected_only=(old_keys==new_keys and set(changed)==EXPECTED_CHANGED)
    unchanged_twenty=(len(mechanical)==21 and len(remaining)==20
                      and all(name in new_bytes and old_bytes[name]==new_bytes[name] for name in remaining))
    ast_pass=(state_equal and values==(3.,4.,5.,6.) and remover.removed==1 and case_equal and module_equal)
    return dict(expected_changed_paths=sorted(EXPECTED_CHANGED),actual_changed_paths=changed,
        added_paths=sorted(new_keys-old_keys),removed_paths=sorted(old_keys-new_keys),
        expected_source_changes_only=expected_only,source_comparisons=rows,
        original_mechanical_source_count=len(mechanical),unchanged_other_mechanical_source_count=sum(
            name in new_bytes and old_bytes[name]==new_bytes[name] for name in remaining),
        all_other_twenty_mechanical_sources_byte_identical=unchanged_twenty,
        mechanics_ast=dict(state_class_ast_identical=state_equal,extended_allowlist_values=values,
            removed_allowlist_uses=remover.removed,make_case_identical_after_removing_allowlist=case_equal,
            entire_module_identical_after_removing_allowlist=module_equal,
            interpretation='AST excludes comments/locations. Only the explicit accepted-angle set changes; no state/control formula is changed.'),
        passed=bool(expected_only and unchanged_twenty and ast_pass))


def parse_csv(data):
    if not data.endswith(b'\n'):raise ValueError('Closed CSV lacks final newline')
    raw=list(csv.DictReader(io.StringIO(data.decode())))
    if not raw:raise ValueError('Empty closed trajectory')
    typed=[]
    for source in raw:
        if None in source or any(v is None for v in source.values()):raise ValueError('Malformed CSV')
        row={}
        for key,value in source.items():
            if value=='':row[key]=None
            elif value in ('True','False'):row[key]=value=='True'
            else:
                try:number=float(value)
                except ValueError:row[key]=value
                else:
                    if not math.isfinite(number):raise ValueError(f'Nonfinite CSV field {key}')
                    row[key]=number
        typed.append(row)
    return dict(raw=raw,typed=typed)


def branch(directory, manifest, identity, policy, files):
    attempts=[a for a in manifest['attempts'] if a['condition_id']==identity and a['policy']==policy]
    if len(attempts)>1:raise ValueError('Duplicate branch invocation')
    attempt=attempts[0] if attempts else {};status=attempt.get('status','not_started')
    result=dict(status=status,record=None,segments={})
    if status!='complete':return result  # No branch files are opened here.
    folder=inside(directory,attempt['folder'])
    record=json.loads(read(folder/'run.json',files,attempt['run_sha256']))
    condition=next(c for c in manifest['case_plan']['conditions'] if c['condition_id']==identity)
    if (record['status']!='complete' or record['policy']!=policy or record['condition']!=condition
            or record['sources']!=manifest['sources']):raise ValueError('Closed branch/manifest disagreement')
    archived_sources(folder,record,files)
    result['record']=record
    for segment in ('reference','recovery'):
        if record.get(segment) is not None:
            name=record[segment]['trajectory']
            result['segments'][segment]=parse_csv(read(inside(folder,name),files,record['artifact_sha256'][name]))
    return result


def exact_rows(old,new):
    differences=[]
    for index,(a,b) in enumerate(zip(old,new)):
        fields={k:dict(old=a.get(k),new=b.get(k)) for k in sorted(set(a)|set(b))
                if k not in TIME_COLUMNS and a.get(k)!=b.get(k)}
        if fields:differences.append(dict(index=index,fields=fields))
    return dict(equal=len(old)==len(new) and not differences,
                old_sample_count=len(old),new_sample_count=len(new),
                differing_observed_row_count=len(differences),
                first_difference=differences[0] if differences else None,
                first_unpaired_index=min(len(old),len(new)) if len(old)!=len(new) else None)


def compare_segment(old,new,protocol):
    if old is None or new is None:
        return dict(status='unobserved_both' if old is None and new is None else 'unobserved_one',
                    old_observed=old is not None,new_observed=new is not None,
                    all_observed_samples_match=None,unobserved_data_imputed=False)
    count=min(len(old['typed']),len(new['typed']))
    whole=compare_prefixes(old['typed'],new['typed'],protocol)
    common=compare_prefixes(old['typed'][:count],new['typed'][:count],protocol)
    exact=exact_rows(old['raw'],new['raw'])
    return dict(status='compared',full_stored_rows_excluding_clocks=exact,
        full_physical_prefix=whole,common_observed_sample_count=count,common_physical_prefix=common,
        common_stored_rows_excluding_clocks=exact_rows(old['raw'][:count],new['raw'][:count]),
        terminal_old={k:old['typed'][-1].get(k) for k in TERMINAL_FIELDS},
        terminal_new={k:new['typed'][-1].get(k) for k in TERMINAL_FIELDS},
        terminal_stored_values_equal_excluding_clocks=exact_rows(old['raw'][-1:],new['raw'][-1:])['equal'],
        all_observed_samples_match=bool(exact['equal'] and whole['replay_prefix_matched']),
        unobserved_data_imputed=False,
        interpretation='Exact stored-field comparison also includes withdrawal commands. Equality covers recorded samples only, not unobserved continuation after a guard stop.')


def compare_branch(old,new,identity,policy):
    result=dict(condition_id=identity,policy=policy,old_status=old['status'],new_status=new['status'],
                status='pending',passed=None,segments={})
    if old['record'] is None or new['record'] is None:return result
    a,b=old['record'],new['record'];protocol=SimpleNamespace(**a['effective_protocol'])
    keys=('physics_hz','seed','protocol','effective_protocol','budget_definition','recovery_motion',
          'preparation','geometry','material','material_after_run','actuator_readback')
    differences=[k for k in keys if a.get(k)!=b.get(k)]
    condition_without_role=lambda record:{k:v for k,v in record['condition'].items() if k!='role'}
    if condition_without_role(a)!=condition_without_role(b):differences.append('condition_excluding_selection_role')
    result.update(status='compared',configuration_differences=differences,
                  selection_role_old=a['condition']['role'],selection_role_new=b['condition']['role'],
                  outcome_old=a['outcome'],outcome_new=b['outcome'],outcomes_equal=a['outcome']==b['outcome'])
    for segment in ('reference','recovery'):
        result['segments'][segment]=compare_segment(old['segments'].get(segment),new['segments'].get(segment),protocol)
    result['passed']=bool(not differences and result['outcomes_equal'] and all(
        segment['status']=='unobserved_both' or segment.get('all_observed_samples_match') is True
        for segment in result['segments'].values()))
    return result


def analyze(old_dir,new_dir):
    old_dir,new_dir=Path(old_dir).resolve(),Path(new_dir).resolve();files={};manifests=[]
    for directory in (old_dir,new_dir):
        manifest=json.loads(read(directory/'study.json',files))
        if manifest['schema']!='Forge-budget-v1':raise ValueError('Unexpected study schema')
        manifests.append(manifest)
    old,new=manifests
    matching_sources=('research/forge_mechanics_protocol.py','research/forge_protocol.py','research/phase2_protocol.py')
    for name in matching_sources:
        if old['sources'][name]!=new['sources'][name]:raise ValueError('Matching code differs across studies')
        read(ROOT/name,files,old['sources'][name])
    source_audit=source_comparison(archived_sources(old_dir,old,files),archived_sources(new_dir,new,files))
    comparisons=[compare_branch(branch(old_dir,old,identity,policy,files),
                                branch(new_dir,new,identity,policy,files),identity,policy)
                 for identity,policy in TARGETS]
    manifest_paths={str(directory/'study.json') for directory in (old_dir,new_dir)}
    for name,digest in files.items():
        if name not in manifest_paths and sha(Path(name).read_bytes())!=digest:
            raise ValueError(f'Closed input changed while auditing: {name}')
    complete=all(c['status']=='compared' for c in comparisons)
    return dict(schema='Forge-budget-angle6-previous-anchor-audit-v1',
        old_study=str(old_dir),new_study=str(new_dir),old_study_status=old['status'],new_study_status=new['status'],
        expected_comparisons=len(TARGETS),closed_comparisons=sum(c['status']=='compared' for c in comparisons),
        status='complete' if complete else 'pending',
        passed=bool(source_audit['passed'] and all(c['passed'] for c in comparisons)) if complete else None,
        source_audit=source_audit,comparisons=comparisons,
        input_files=[dict(path=k,sha256=v) for k,v in files.items()],
        matching_sources_verified_against_both_archives=list(matching_sources),
        script_sha256=sha(Path(__file__).read_bytes()),
        matching_helper_sha256=sha((ROOT/'research/forge_mechanics_protocol.py').read_bytes()),
        interpretation=[
            'Only manifest-complete branches are opened. Pending does not imply a failed or mismatched experiment.',
            'The 1.5 degree branches and 2 degree straight branch are reproducibility anchors; they are not new independent random replications.',
            'All stored CSV fields except time_s/physics_time_s are checked exactly; the production matcher also checks physical states, commands and relative task time.',
            'Source/AST equality proves this bounded code change, not equality of unobserved solver memory or real-world physical accuracy.',
            'Both absent recoveries mean recovery was unobserved in both references, never two successful recovery trials.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--old-study',type=Path,default=ROOT/'outputs/Forge-Budget-V1-20260915')
    parser.add_argument('--new-study',type=Path,default=ROOT/'outputs/Forge-Budget-Angle6-20260915')
    parser.add_argument('--output',type=Path,default=HERE/'previous_anchor_audit.json')
    args=parser.parse_args();output=args.output.resolve()
    if any(source.resolve() in output.parents for source in (args.old_study,args.new_study)):
        raise ValueError('Audit output must be outside both source studies')
    result=analyze(args.old_study,args.new_study)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),status=result['status'],passed=result['passed'],
                         closed_comparisons=result['closed_comparisons']),indent=2))


if __name__=='__main__':main()
