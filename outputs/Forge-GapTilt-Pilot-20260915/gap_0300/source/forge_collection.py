"""Quota scheduling and resume checks for the FORGE characterization collection."""
import json
from research.phase2_protocol import FAMILIES, sample_slot


def family_quotas(protocol):
    """Exact integer allocation, scaling the configured weights for other targets."""
    protocol.validate()
    total=sum(protocol.family_weights.values())
    counts={f:protocol.target_valid*protocol.family_weights[f]//total for f in FAMILIES}
    remainder=protocol.target_valid-sum(counts.values())
    ranked=sorted(FAMILIES,key=lambda f:-(protocol.target_valid*protocol.family_weights[f]%total))
    for family in ranked[:remainder]:counts[family]+=1
    return counts


def sample_collection_slot(protocol,slot,retry=0):
    """Interleave families until their quotas fill; preserve signed strata on retry."""
    if not 0<=slot<protocol.target_valid:raise ValueError('Collection slot outside target')
    quotas=family_quotas(protocol)
    schedule=[(family,ordinal) for ordinal in range(max(quotas.values()))
              for family in FAMILIES if ordinal<quotas[family]]
    family,ordinal=schedule[slot]
    # Reuse the signed-stratum sampler without letting reduced centered quotas
    # shift a contact family's sign/axis sequence or its retry random stream.
    sampling_slot=ordinal*len(FAMILIES)+FAMILIES.index(family)
    case=sample_slot(protocol,sampling_slot,retry)
    case.update(slot=slot,trajectory_id=f'slot{slot:03d}_try{retry:02d}',
                sampling_slot=sampling_slot,
                sample_role='repeatability_control' if family=='centered' else 'characterization',
                split_group_id='centered_controls' if family=='centered' else f'slot{slot:03d}')
    return case


def accepted_slots(attempts):
    return {a['slot'] for a in attempts if a['status']=='complete'
            and a.get('metrics',{}).get('numerically_valid') is True}


def next_case(protocol,attempts,case_plan=None):
    accepted=accepted_slots(attempts)
    for slot in range(protocol.target_valid):
        if slot in accepted:
            continue
        finished={a['retry'] for a in attempts if a['slot']==slot and a['status']=='complete'}
        for retry in range(protocol.attempts_per_slot):
            if retry not in finished:
                if case_plan is not None:
                    if case_plan.get('schema') == 'Forge-gap-tilt-v1':
                        from research.forge_gap_study import case_for_slot
                    else:
                        from research.phase2b import case_for_slot
                    return case_for_slot(case_plan,slot,retry)
                return sample_collection_slot(protocol,slot,retry)
    return None


def collection_cases(protocol,attempts,case_plan=None):
    while (case:=next_case(protocol,attempts,case_plan)) is not None:
        yield case


def validate_resume(saved,current):
    if saved.get('study')!='Forge-Controlled-Phase2-v1' or saved.get('mode')!='collect':
        raise ValueError('Only FORGE collect-mode runs can be resumed')
    for key in ('protocol','physics_hz','requested_checkpoints_mm','sources','stock_buffers','case_plan',
                'radial_clearance_mm','gap_probe_events'):
        if json.dumps(saved.get(key),sort_keys=True)!=json.dumps(current.get(key),sort_keys=True):
            raise ValueError(f'Cannot resume with changed {key}; use the original settings and source files')


def collection_status(protocol,attempts):
    if len(accepted_slots(attempts))==protocol.target_valid:
        return 'complete'
    return 'paused' if next_case(protocol,attempts) is not None else 'target_not_met'
