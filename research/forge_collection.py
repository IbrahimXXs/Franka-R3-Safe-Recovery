"""Quota scheduling and resume checks for the FORGE characterization collection."""
import json
from research.phase2_protocol import sample_slot


def accepted_slots(attempts):
    return {a['slot'] for a in attempts if a['status']=='complete'
            and a.get('metrics',{}).get('numerically_valid') is True}


def next_case(protocol,attempts):
    accepted=accepted_slots(attempts)
    for slot in range(protocol.target_valid):
        if slot in accepted:
            continue
        finished={a['retry'] for a in attempts if a['slot']==slot and a['status']=='complete'}
        for retry in range(protocol.attempts_per_slot):
            if retry not in finished:
                return sample_slot(protocol,slot,retry)
    return None


def collection_cases(protocol,attempts):
    while (case:=next_case(protocol,attempts)) is not None:
        yield case


def validate_resume(saved,current):
    if saved.get('study')!='Forge-Controlled-Phase2-v1' or saved.get('mode')!='collect':
        raise ValueError('Only FORGE collect-mode runs can be resumed')
    for key in ('protocol','physics_hz','requested_checkpoints_mm','sources','stock_buffers'):
        if json.dumps(saved.get(key),sort_keys=True)!=json.dumps(current.get(key),sort_keys=True):
            raise ValueError(f'Cannot resume with changed {key}; use the original settings and source files')


def collection_status(protocol,attempts):
    if len(accepted_slots(attempts))==protocol.target_valid:
        return 'complete'
    return 'paused' if next_case(protocol,attempts) is not None else 'target_not_met'
