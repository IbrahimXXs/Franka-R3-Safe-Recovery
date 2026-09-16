"""Fresh-path exclusion, frozen sources, exact pairing and strict matching."""
from collections import Counter
from copy import deepcopy
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch
from research.productivity_detector_v2_generalization import conditions, path_signature, prior_paths, schedule, verify, CONFIG_SHA
from research.productivity_detector_v2_generalization_report import exact_mcnemar, paired_statistics, paired_prefix, selected, write_table


def pair(a,b,**kw):
    return dict(case_id=str(kw),family='combined',severity='severe',prefix_matched=True,
        v1_success=a,v2_success=b,success_delta=int(b)-int(a),**kw)


class DesignTests(unittest.TestCase):
    def test_coverage_distinct_and_balanced(self):
        cc=conditions()
        self.assertEqual(len({path_signature(c['case']) for c in cc}),64)
        self.assertEqual(Counter(c['severity'] for c in cc),dict(easy=8,moderate=28,severe=28))
        self.assertEqual(set(Counter(c['family'] for c in cc).values()),{8})
        for field in ('final_offset_x_mm','final_offset_y_mm','final_roll_deg','final_pitch_deg'):
            self.assertTrue(any(c['case'][field]>0 for c in cc));self.assertTrue(any(c['case'][field]<0 for c in cc))
    def test_no_overlap_with_any_prior_descriptor(self):
        root=Path(__file__).resolve().parents[1];old,audit=prior_paths(root)
        self.assertGreaterEqual(audit['descriptor_counts']['outputs/Contact-Productivity-Generalization-v1/experiment.json'],48)
        self.assertEqual(audit['descriptor_counts']['outputs/Contact-Productivity-DetectorV2-Dev/experiment.json'],80)
        self.assertFalse({path_signature(c['case']) for c in conditions()} & old)
    def test_signatures_ignore_metadata_and_signed_zero(self):
        a=conditions()[0]['case'];b=dict(a,path_group_id='alias',family='renamed',roll_deg=-0.)
        self.assertEqual(path_signature(a),path_signature(b))
        b['final_roll_deg']+=.1;self.assertNotEqual(path_signature(a),path_signature(b))
    def test_schedule_all_pairs_once_balanced_order(self):
        cc=conditions();ss=schedule(cc)
        self.assertEqual(len(ss),192)
        self.assertEqual(len({(r['case_id'],r['policy']) for r in ss}),192)
        for policy in ('nominal','v1','v2'):
            self.assertEqual(sorted(Counter(r['policy_order'] for r in ss if r['policy']==policy).values()),[21,21,22])
        self.assertEqual(schedule(cc),ss)
    def test_prior_overlap_check_includes_previous_benchmark_namespace(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'experiments').mkdir();d=root/'outputs/Contact-Productivity-Generalization-v1';d.mkdir(parents=True)
            c=conditions()[0]
            for p in (d/'experiment.json',root/'experiments/productivity_generalization_frozen.json',root/'experiments/productivity_detector_v2_dev.json'):
                p.write_text(json.dumps(dict(cases=[c])))
            old,audit=prior_paths(root);self.assertIn(path_signature(c['case']),old)
            self.assertEqual(audit['descriptor_counts']['outputs/Contact-Productivity-Generalization-v1/experiment.json'],1)
    def test_complete_lock_rejects_analysis_source_drift(self):
        from dataclasses import asdict
        from research.productivity_control import Design
        from research.productivity_unloading import UnloadingDesign
        from research.productivity_dewedge import DewedgeDesign
        p=dict(schema='Contact-Productivity-DetectorV2-Generalization-v1',cases=conditions(),schedule=schedule(conditions()),
            design=asdict(Design()),unloading_design=asdict(UnloadingDesign()),dewedge_design=asdict(DewedgeDesign()),
            detector_v2_config_sha256=CONFIG_SHA,source_sha256={'report.py':'expected'},holdout_audit={'source_sha256':{}})
        with patch('research.productivity_detector_v2_generalization.sha',side_effect=[CONFIG_SHA,'changed']):
            with self.assertRaisesRegex(ValueError,'report.py'):verify(Path('/unused'),p)


class StatisticsTests(unittest.TestCase):
    def test_exact_two_sided_test(self):
        self.assertEqual(exact_mcnemar(0,0),1.)
        self.assertEqual(exact_mcnemar(5,0),.0625)
        self.assertEqual(exact_mcnemar(0,5),.0625)
        self.assertEqual(exact_mcnemar(4,4),1.)
        self.assertAlmostEqual(exact_mcnemar(9,1),.021484375)
    def test_paired_counts_include_all_failures(self):
        rr=[pair(False,True),pair(False,True),pair(True,False),pair(True,True),pair(False,False)]
        r=paired_statistics(rr)
        self.assertEqual((r['conditions'],r['v2_wins'],r['v2_losses'],r['both_succeed'],r['both_fail']),(5,2,1,1,1))
        self.assertEqual(r['success_rate_difference'],.2)
        self.assertEqual(r,paired_statistics(rr))
    def test_matched_and_severe_subsets_do_not_replace_primary(self):
        rr=[pair(False,True),pair(True,False)]
        rr[1].update(prefix_matched=False,severity='moderate')
        self.assertEqual(paired_statistics(rr)['conditions'],2)
        self.assertEqual(paired_statistics(rr,'strict_matched')['v2_wins'],1)
        self.assertEqual(len(selected(rr,'severe')),1)
        self.assertEqual(paired_statistics([], 'strict_matched')['exact_mcnemar_p'],None)
    def test_strict_prefix_rejects_transient_mismatch_even_if_endpoint_matches(self):
        a=[dict(intervention_started=i==3,depth_mm=i,time_s=i) for i in range(5)]
        b=deepcopy(a)
        with patch('research.productivity_detector_v2_generalization_report.endpoint_prefix',return_value={'prefix_matched':True}),patch('research.productivity_detector_v2_generalization_report.match',side_effect=[(True,{}),(False,{}),(True,{}),(True,{})]):
            r=paired_prefix(a,b,None)
        self.assertTrue(r['endpoint_prefix_matched']);self.assertFalse(r['prefix_matched'])
        self.assertEqual(r['prefix_mismatched_samples'],1);self.assertEqual(r['prefix_samples'],4)
    def test_empty_csv_has_explicit_schema(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'empty.csv';write_table(p,[],('case_id','outcome'))
            self.assertEqual(p.read_text().strip(),'case_id,outcome')


class ReportTests(unittest.TestCase):
    def test_report_handles_no_triggers_and_empty_matched_subset(self):
        from research.productivity_detector_v2_generalization_report import policy_totals, report
        pp=[pair(True,True)];pp[0].update(case_id='fixture',prefix_matched=False,neither_intervenes=True,
            v2_new_branch_used=False,v2_trigger_branches='',v1_outcome='success',v2_outcome='success')
        keys=('stalled safety_stop safety_during_recovery safety_before_any_recovery terminal_timeout '
            'intervention_count verified_unloadings ready_unloadings successful_retries repeated_stalls '
            'normal_interventions urgent_interventions terminal_interventions').split()
        summaries=[dict(case_id='fixture',policy=p,insertion_success=True,**dict.fromkeys(keys,0)) for p in ('nominal','v1','v2')]
        totals=policy_totals(summaries,[],[],pp)
        self.assertEqual(next(r for r in totals if r['subset']=='strict_matched')['conditions'],0)
        stats=[paired_statistics(pp,k) for k in ('all','severe','strict_matched','severe_strict_matched')]
        branches=[dict(policy=p,branch=b,recoveries=0,verified_unloadings=0,ready_unloadings=0,retry_successes=0,repeated_stalls=0)
            for p in ('v1','v2') for b in ('normal','urgent','terminal')]
        audit=dict(physics_rows=1,causal_checks=0,interventions=0,preexisting_outputs_checked=1)
        with tempfile.TemporaryDirectory() as tmp:
            report(Path(tmp),totals,{'normal_eta_threshold':.4212659765112803},pp,[],audit,[],stats,branches,
                dict(frozen_at_utc='fixture',optional_baselines_reason='fixture'))
            text=(Path(tmp)/'report.md').read_text()
            self.assertIn('N/A',text);self.assertIn('0/64',text)


if __name__=='__main__':unittest.main()
