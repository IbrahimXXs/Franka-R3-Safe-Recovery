"""Read-only summaries distinguish missing events from unresolved tests."""
import json
from pathlib import Path
import tempfile
import unittest

from simulation.summarize_gap_study import summarize_batch, export_review, _event_status


class GapSummaryTests(unittest.TestCase):
    def make_batch(self, directory):
        control=dict(case_id='aligned',radial_clearance_mm=.1,tilt_amplitude_deg=0.)
        tilted=dict(case_id='tilted',radial_clearance_mm=.1,tilt_amplitude_deg=2.,
                    tilt_onset_fraction=.5,tilt_sign=1)
        (directory/'plan.json').write_text(json.dumps({'schema':'Forge-gap-tilt-v1','cases':[control,tilted]}))
        (directory/'batch.json').write_text(json.dumps({'status':'running','groups':[{'folder':'tight'}]}))
        child=directory/'tight';child.mkdir()
        checkpoints=[dict(checkpoint_kind='pre_tilt',checkpoint_id='pre_tilt',reached=False,probes=[]),
                     dict(checkpoint_kind='first_stall',checkpoint_id='first_stall',reached=False,probes=[]),
                     dict(checkpoint_kind='terminal',checkpoint_id='terminal',reached=True,
                          state={'depth_mm':20.,'tilt_deg':.1},prefix_numerically_valid=True,
                          Y_R_tested=None,probes=[dict(policy='straight',label_eligible=True,
                                                     safe_recovery=False,recovery_censored=True,
                                                     termination_phase='retreat',max_wrist_force_n=21.,
                                                     retreat_peak_wrist_force_n=21.)])]
        attempt=dict(control,trajectory_id='aligned_try00',status='complete',folder='aligned_try00',
                     metrics=dict(numerically_valid=True,insertion_success=True,stalled=False,
                                  terminal_actual_tilt_deg=.1),checkpoints=checkpoints)
        (child/'study.json').write_text(json.dumps({'attempts':[attempt]}))
        return child

    def test_unstarted_case_and_event_types_do_not_become_recovery_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp);self.make_batch(directory)
            result=summarize_batch(directory)
            self.assertEqual(result['summary']['counts']['planned_cases'],2)
            self.assertEqual(result['summary']['counts']['complete_cases'],1)
            self.assertEqual(result['cases'][1]['attempt_status'],'not_started')
            self.assertIsNone(result['cases'][0]['terminal_actual_fraction_of_target'])
            events=result['summary']['counts']['checkpoint_event_status_counts']
            self.assertEqual(events['not_applicable_aligned_control'],1)
            self.assertEqual(events['event_not_observed'],1)
            self.assertEqual(events['recovery_evidence_unresolved'],1)
            self.assertEqual(result['summary']['counts']['direct_failed_realign_safe'],0)
            self.assertEqual(result['summary']['counts']['censored_probes'],1)
            terminal=result['checkpoints'][-1]
            self.assertEqual(terminal['straight_retreat_peak_wrist_force_n'],21.)
            self.assertIsNone(terminal['realign_retreat_peak_wrist_force_n'])

    def test_export_requires_destination_outside_source(self):
        with tempfile.TemporaryDirectory() as temp, tempfile.TemporaryDirectory() as out:
            directory=Path(temp);self.make_batch(directory)
            before={str(p.relative_to(directory)):p.read_bytes() for p in directory.rglob('*') if p.is_file()}
            result=summarize_batch(directory)
            with self.assertRaises(ValueError):export_review(result,directory)
            with self.assertRaises(ValueError):export_review(result,directory/'review')
            export_review(result,out)
            self.assertTrue((Path(out)/'summary.json').exists())
            self.assertTrue((Path(out)/'checkpoint_summary.csv').exists())
            after={str(p.relative_to(directory)):p.read_bytes() for p in directory.rglob('*') if p.is_file()}
            self.assertEqual(before,after)

    def test_event_only_classification_does_not_hide_invalid_prefix(self):
        case={'tilt_amplitude_deg':2.}
        cp=dict(checkpoint_kind='ramp_complete',reached=True,probes=[],probe_requested=False,
                prefix_numerically_valid=False)
        self.assertEqual(_event_status(case,cp,True),'numerically_invalid_prefix')
        self.assertEqual(_event_status(case,cp,False),'reference_incomplete')
        cp['prefix_numerically_valid']=True
        self.assertEqual(_event_status(case,cp,True),'event_only_no_probe_requested')
        cp['probe_requested']=True
        self.assertEqual(_event_status(case,cp,True),'recovery_evidence_unresolved')

    def test_early_valid_prefix_evidence_survives_later_reference_invalidity(self):
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp);child=self.make_batch(directory)
            path=child/'study.json';manifest=json.loads(path.read_text())
            attempt=manifest['attempts'][0]
            attempt['metrics'].update(numerically_valid=False,insertion_success=True,stalled=True)
            attempt['checkpoints'][1]=dict(checkpoint_kind='first_stall',checkpoint_id='first_stall',
                reached=True,probe_requested=True,prefix_numerically_valid=True,Y_R_tested=1,
                label_reason='safe_policy_witness',probes=[
                    dict(policy='straight',label_eligible=True,safe_recovery=False,retreat_peak_wrist_force_n=10.),
                    dict(policy='realign',label_eligible=True,safe_recovery=True,retreat_peak_wrist_force_n=2.)])
            attempt['checkpoints'][2].update(prefix_numerically_valid=False,probes=[],Y_R_tested=None)
            path.write_text(json.dumps(manifest))
            result=summarize_batch(directory);counts=result['summary']['counts']
            self.assertEqual(counts['successful_insertions'],1)
            self.assertEqual(counts['stalled_insertions'],1)
            self.assertEqual(counts['trusted_successful_insertions'],0)
            self.assertEqual(counts['trusted_stalled_insertions'],0)
            self.assertEqual(counts['prefix_valid_reached_checkpoints'],1)
            self.assertEqual(counts['whole_reference_valid_reached_checkpoints'],0)
            self.assertEqual(counts['both_policy_pairs_eligible'],1)
            self.assertEqual(counts['whole_reference_valid_both_policy_pairs_eligible'],0)
            self.assertEqual(counts['direct_failed_realign_safe'],1)
            self.assertEqual(counts['whole_reference_valid_direct_failed_realign_safe'],0)
            self.assertEqual(counts['maximum_recorded_direct_retreat_wrist_n'],10.)
            self.assertIsNone(counts['whole_reference_valid_maximum_recorded_direct_retreat_wrist_n'])
            self.assertEqual(result['checkpoints'][1]['Y_R_tested'],1)
            self.assertEqual(result['checkpoints'][1]['label_reason'],'safe_policy_witness')
            self.assertEqual(result['checkpoints'][2]['event_status'],'numerically_invalid_prefix')

    def test_continuous_withdrawal_survives_replay_unknown_without_filling_pair(self):
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp);child=self.make_batch(directory)
            path=child/'study.json';manifest=json.loads(path.read_text())
            attempt=manifest['attempts'][0]
            attempt['final_retreat']=dict(safe_recovery=True,numerically_valid=True,grasp_retained=True,
                cleared=True,recovery_censored=False,termination_reason='cleared',termination_phase='retreat',
                max_wrist_force_n=.05,stop_peak_wrist_force_n=.05,retreat_peak_wrist_force_n=.01727)
            attempt['checkpoints'][2]['probes']=[dict(policy=policy,label_eligible=False,replay_matched=False,
                                                    safe_recovery=None,reason='replay_mismatch')
                                               for policy in ('straight','realign')]
            path.write_text(json.dumps(manifest))
            result=summarize_batch(directory);case=result['cases'][0];counts=result['summary']['counts']
            self.assertEqual(case['continuation_straight_origin'],'original_reference_continuation_without_replay')
            self.assertEqual(case['continuation_straight_retreat_peak_wrist_force_n'],.01727)
            self.assertEqual(case['continuation_straight_max_wrist_force_n'],.05)
            self.assertTrue(result['attempts'][0]['continuation_straight_safe_recovery'])
            self.assertEqual(counts['continuation_straight_observed_complete_references'],1)
            self.assertEqual(counts['continuation_straight_valid_reference_and_recovery'],1)
            self.assertEqual(counts['continuation_straight_valid_safe_recoveries'],1)
            self.assertEqual(counts['continuation_straight_valid_maximum_recorded_retreat_wrist_n'],.01727)
            self.assertEqual(counts['both_policy_pairs_eligible'],0)
            self.assertIsNone(counts['maximum_recorded_direct_retreat_wrist_n'])
            self.assertIsNone(result['checkpoints'][-1]['Y_R_tested'])


if __name__=='__main__':unittest.main()
