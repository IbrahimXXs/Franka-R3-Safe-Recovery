"""Postcollection audit erratum: replay runtime SLERP, keeping original tolerances.

No frozen source, detector, metric, trajectory or collected artifact is changed.
The original all-episode audit result remains in frozen_audit_diagnostic.json.
"""
import ast
from datetime import datetime, timezone
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import sys
from types import FunctionType

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from research import force_productivity_analysis as analysis
from research import force_productivity_report as report
from research import productivity_detector_v2_report as v2_report
from research.productivity_dewedge_report import audit as frozen_audit

DIRECTORY = Path(__file__).resolve().parents[1]
VALIDATION = DIRECTORY / 'validation'
spec = importlib.util.spec_from_file_location('runtime_math_snapshot', VALIDATION / 'runtime_math_snapshot.py')
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)
errors = []


def runtime_hand(event, fraction, command):
    """Exact float32 expressions used by execute_dewedge during recovery.

Inputs are captured event transforms and the independently replayed recovery
schedule, never the logged command being checked. No Isaac Sim is launched.
"""
    def tensor(values):
        return torch.tensor(values, dtype=torch.float32, device='cuda:0').unsqueeze(0)
    with torch.inference_mode():
        anchor = tensor(event['trigger_peg_position'])
        neutral = tensor(event['neutral_peg_position'])
        pp = (anchor + fraction * (neutral - anchor)).clone()
        pp[:, 2] += command / 1000.
        pq = runtime.quat_slerp(tensor(event['trigger_peg_quaternion'])[0],
                               tensor(event['neutral_peg_quaternion'])[0].clone(), fraction).unsqueeze(0)
        hq = runtime.quat_mul(pq, runtime.quat_conjugate(tensor(event['captured_grasp_quaternion'])))
        hp = pp - runtime.quat_apply(hq, tensor(event['captured_grasp_position']))
        return hp[0].cpu().numpy(), hq[0].cpu().numpy()


def correction():
    """Replace only the recovery hand-transform reconstruction, not any assert."""
    tree = ast.parse(inspect.getsource(frozen_audit))
    class RuntimeTransform(ast.NodeTransformer):
        replacements = 0
        transforms = 0
        def visit_Assign(self, node):
            if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'hp':
                self.transforms += 1
                if self.transforms == 1:
                    self.replacements += 1
                    return ast.copy_location(ast.parse('hp, hpq = runtime_hand(event, frac, command)').body[0], node)
            return node
    transform = RuntimeTransform()
    modified = transform.visit(tree)
    assert transform.replacements == 1 and transform.transforms == 2
    ast.fix_missing_locations(modified)
    before_asserts = [ast.dump(n) for n in ast.walk(ast.parse(inspect.getsource(frozen_audit))) if isinstance(n, ast.Assert)]
    after_asserts = [ast.dump(n) for n in ast.walk(modified) if isinstance(n, ast.Assert)]
    assert before_asserts == after_asserts
    # The assert_allclose calls and their 2e-7/3e-7 tolerances also remain exact.
    comparisons = lambda t: [ast.dump(n) for n in ast.walk(t) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'assert_allclose']
    assert comparisons(ast.parse(inspect.getsource(frozen_audit))) == comparisons(modified)
    namespace = dict(frozen_audit.__globals__, runtime_hand=runtime_hand)
    exec(compile(modified, str(__file__) + ':runtime_reconstruction', 'exec'), namespace)
    fixed = namespace['audit']
    fixed_v2 = FunctionType(v2_report.audit_run.__code__, dict(v2_report.audit_run.__globals__, original_audit=fixed))
    fixed_run = FunctionType(analysis.audit_run.__code__, dict(analysis.audit_run.__globals__, original_audit=fixed, v2_audit=fixed_v2))
    return FunctionType(report.analyze.__code__, dict(report.analyze.__globals__, audit_run=fixed_run))


def main():
    original = json.loads((VALIDATION / 'frozen_audit_diagnostic.json').read_text())
    assert sum(original['counts'].values()) == 384
    diagnostic = json.loads((VALIDATION / 'runtime_math_diagnostic.json').read_text())
    math_source = ROOT / '.deps/IsaacLab/source/isaaclab/isaaclab/utils/math.py'
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    assert digest(math_source) == diagnostic['isaaclab_math_sha256']
    # Verify the retained math snapshot contains the exact installed function bodies.
    current = ast.parse(math_source.read_text())
    snapshot = ast.parse((VALIDATION / 'runtime_math_snapshot.py').read_text())
    names = {'quat_slerp', 'quat_mul', 'quat_apply', 'quat_conjugate'}
    assert {n.name: ast.dump(n) for n in current.body if isinstance(n, ast.FunctionDef) and n.name in names} == {
        n.name: ast.dump(n) for n in snapshot.body if isinstance(n, ast.FunctionDef)}
    # Demonstrate unchanged tolerance still rejects a materially different command.
    np.testing.assert_allclose(diagnostic['logged'], diagnostic['runtime'], atol=3e-7, rtol=0)
    try:
        np.testing.assert_allclose(np.asarray(diagnostic['logged']) + 1e-5, diagnostic['runtime'], atol=3e-7, rtol=0)
    except AssertionError:
        pass
    else:
        raise AssertionError('Original command tolerance did not reject a 10 micrometre perturbation')
    result = correction()(DIRECTORY)
    validation = json.loads((DIRECTORY / 'validation.json').read_text())
    validation.update(status='passed_runtime_math_replay_with_frozen_audit_exception',
        frozen_audit_status='383_passed_1_failed', original_frozen_audit=original,
        postcollection_audit_erratum=True, original_audit_tolerances_unchanged=True,
        detector_metrics_and_paired_matching_unchanged=True,
        explanation='Offline SciPy SLERP differs from the existing Isaac Lab float32 near-identity branch. Runtime reconstruction replaces only expected recovery hand transform; every original assertion and tolerance remains unchanged.')
    (DIRECTORY / 'validation.json').write_text(json.dumps(validation, indent=2) + '\n')
    notice = ('> Audit erratum: the original frozen SciPy reconstruction passed 383/384 episodes and failed one '
              'near-identity rotation case. A separately archived postcollection replay using the actual Isaac Lab '
              'float32 math passed all 384 with every original assertion and tolerance unchanged. The original '
              'failure is retained in `validation/frozen_audit_diagnostic.json`; see '
              '[audit erratum](validation/audit_erratum.md). No detector, recovery, budget, physics, outcome, '
              'selection rule, metric, or paired-matching criterion changed.\n\n')
    path = DIRECTORY / 'report.md'
    path.write_text(notice + path.read_text())
    provenance = json.loads((DIRECTORY / 'provenance_audit.json').read_text())
    provenance['postcollection_audit_erratum'] = dict(
        utc=datetime.now(timezone.utc).isoformat(),
        command='python outputs/Force-vs-Productivity-Dewedge-v1/validation/finish_with_runtime_audit.py',
        original_frozen_source_files_unchanged=True, all_384_episodes_included=True,
        original_audit_tolerances_unchanged=True, source_isaaclab_math_sha256=digest(math_source),
        files_sha256={str(p.relative_to(DIRECTORY)): digest(p) for p in (
            Path(__file__), VALIDATION / 'runtime_math_snapshot.py', VALIDATION / 'runtime_math_diagnostic.json',
            VALIDATION / 'frozen_audit_diagnostic.json')})
    (DIRECTORY / 'provenance_audit.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
