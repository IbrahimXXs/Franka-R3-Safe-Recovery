"""Reproduce the illustrative guide-span comparison from closed references."""
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
STUDY = ROOT/'outputs/Forge-Budget-V1-20260915'
rows = []
for identity in ('m_d12_fs100_fd100_a1p5__b04', 'm_d18_fs100_fd100_a1p5__b04'):
    directory = STUDY/'runs'/identity/'straight'
    record_path = directory/'run.json'
    record = json.loads(record_path.read_text())
    if record['status'] != 'complete':
        raise ValueError('Only closed branches may be analyzed')
    path = directory/'insertion.csv'
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != record['artifact_sha256']['insertion.csv']:
        raise ValueError('Reference artifact hash mismatch')
    last = list(csv.DictReader(data.decode().splitlines()))[-1]
    valid = last['geometry_overlap_estimate_valid'] == 'True'
    if not valid: raise ValueError('Guide-span estimate unavailable')
    length = float(last['effective_guide_span_estimate_mm'])
    angle = float(last['tilt_deg'])
    gap = record['effective_protocol']['effective_radial_clearance_mm']
    spread = length*math.tan(math.radians(angle))
    rows.append(dict(condition_id=identity, target_depth_mm=record['condition']['case']['target_depth_mm'],
        actual_terminal_depth_mm=float(last['depth_mm']), actual_terminal_tilt_deg=angle,
        effective_guide_span_estimate_mm=length, diametral_clearance_mm=2*gap,
        illustrative_transverse_spread_mm=spread, spread_over_diametral_clearance=spread/(2*gap),
        geometry_overlap_estimate_mm=float(last['geometry_overlap_estimate_mm']),
        source=str(path), sha256=hashlib.sha256(data).hexdigest(),
        run_sha256=hashlib.sha256(record_path.read_bytes()).hexdigest()))
value = dict(schema='budget-guide-span-illustration-v1', rows=rows,
    expression='transverse_spread = Lg * tan(actual tilt); ratio = transverse_spread / (2 * effective radial clearance)',
    interpretation='Approximate geometric illustration only. Lg is an analytic guide overlap, not the distance between actual loaded contacts. The ratio is not a jamming, self-locking, fracture or force prediction criterion.',
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
Path(__file__).with_name('guide_span_audit.json').write_text(json.dumps(value,indent=2)+'\n')
print(json.dumps(rows,indent=2))
