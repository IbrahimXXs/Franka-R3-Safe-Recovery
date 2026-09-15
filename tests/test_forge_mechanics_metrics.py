"""Mechanics measurements and partial-ledger summaries preserve evidence scope."""
import copy
import unittest

from research.forge_mechanics_metrics import mechanics_reference_metrics, mechanics_recovery_metrics
from research.forge_mechanics_plan import make_plan, make_case, MechanicsState
from simulation.summarize_mechanics_study import summarize_manifest
from simulation.plot_mechanics_study import measurement_cell


def row(t, phase, depth=6., wrist=1.):
    return dict(time_s=t, phase=phase, depth_mm=depth, command_depth_mm=6., tilt_deg=0.,
                command_tilt_deg=0., wrist_force_n=wrist, wrist_torque_nm=.01,
                wrist_force_world_2=wrist, force_norm_n=wrist, fz=-wrist, contact_power_w=0.)


def completed_attempt(case, *, prefix=True, equal=False):
    recovery=dict(label_eligible=True,numerically_valid=True,safe_recovery=True,cleared=True,
                  recovery_censored=False,max_wrist_force_n=3.,retreat_peak_wrist_force_n=2.,
                  retreat_peak_wrist_force_mean100ms_n=1.8)
    return dict(case,trajectory_id=case['case_id']+'_try00',folder=case['case_id'],status='complete',
                metrics=dict(numerically_valid=True,grasp_retained=True,aligned_entry_reached=True,
                             reference_protocol_completed=True,tilt_terminal_depth_loss_mm=.2),
                final_retreat=recovery,
                probes=[dict(recovery,policy='realign',replay_matched=True,replay_prefix_matched=prefix,
                             replay_prefix_equal=equal,retreat_peak_wrist_force_n=.25)])


class MechanicsMetricsTests(unittest.TestCase):
    def setUp(self):
        self.case=dict(target_depth_mm=6.,settle_tolerance_mm=.1,settle_dwell_s=.2,tilt_amplitude_deg=2.)

    def test_aligned_entry_and_tilt_depth_loss_are_separate_observations(self):
        rows=[row(0.,'insert',4.),row(.1,'depth_settle'),row(.2,'depth_settle'),row(.3,'depth_settle'),
              row(.4,'tilt_ramp',5.9),row(.5,'tilt_hold',5.4)]
        rows[-1].update(tilt_deg=1.8,command_tilt_deg=2.,reference_termination_reason='reference_complete')
        result=mechanics_reference_metrics(rows,self.case,.1)
        self.assertTrue(result['aligned_entry_reached'])
        self.assertAlmostEqual(result['aligned_entry_gate_time_s'],.3)
        self.assertAlmostEqual(result['tilt_terminal_depth_loss_mm'],.6)
        self.assertAlmostEqual(result['tilt_max_depth_loss_mm'],.6)
        self.assertTrue(result['reference_protocol_completed'])
        self.assertEqual(result['terminal_actual_tilt_deg'],1.8)
        self.assertNotIn('insertion_success',result)
        self.assertNotIn('numerically_valid',result)

    def test_depth_gate_is_not_satisfied_by_earlier_insert_samples(self):
        rows=[row(t,'insert') for t in (0.,.1,.2,.3)] + [row(.4,'depth_settle',5.)]
        result=mechanics_reference_metrics(rows,self.case,.1)
        self.assertFalse(result['aligned_entry_reached'])
        self.assertIsNone(result['tilt_terminal_depth_loss_mm'])
        for r in rows:r['depth_gate_passed']=False
        self.assertFalse(mechanics_reference_metrics(rows,self.case,.1)['aligned_entry_reached'])

    def test_runner_settle_tilt_hold_phases_are_measured(self):
        rows=[dict(row(0.,'settle'),depth_gate_passed=False),
              dict(row(.1,'settle'),depth_gate_passed=True,depth_gate_time_s=.1),
              row(.2,'tilt',5.8,wrist=2.),
              dict(row(.3,'hold',5.5,wrist=3.),reference_termination_reason='reference_complete')]
        result=mechanics_reference_metrics(rows,self.case,.1)
        self.assertTrue(result['tilt_window_observed'])
        self.assertTrue(result['settle_observed'])
        self.assertTrue(result['tilt_observed'])
        self.assertTrue(result['hold_observed'])
        self.assertEqual(result['tilt_peak_wrist_force_n'],2.)
        self.assertEqual(result['hold_peak_wrist_force_n'],3.)
        self.assertEqual(result['depth_at_tilt_start_mm'],6.)
        self.assertEqual(result['tilt_terminal_depth_loss_mm'],.5)

    def test_real_state_machine_generated_rows_preserve_tilt_measurements(self):
        case=make_case(6.,(.5,.5),2.)
        state=MechanicsState(case);dt=.05;rows=[]
        state.observe(0.,-10.,True,True,physics_step=0)
        for step in range(1,500):
            t=step*dt;command=state.command(t)
            depth=command['depth_mm']
            if command['phase'] in ('tilt','hold'):depth-=.3*command['pitch_deg']/2.
            state.observe(t,depth,True,True,physics_step=step)
            sample=row(t,command['phase'],depth,wrist=2.)
            sample.update(state.as_dict(),command_depth_mm=command['depth_mm'],
                          command_tilt_deg=abs(command['pitch_deg']),tilt_deg=abs(command['pitch_deg']))
            rows.append(sample)
            if state.done:break
        self.assertEqual({r['phase'] for r in rows},{'approach','insert','settle','tilt','hold'})
        result=mechanics_reference_metrics(rows,case,dt)
        self.assertTrue(result['aligned_entry_reached'])
        self.assertTrue(result['reference_protocol_completed'])
        self.assertTrue(result['tilt_ramp_completed'])
        self.assertAlmostEqual(result['depth_at_tilt_start_mm'],6.)
        self.assertAlmostEqual(result['tilt_terminal_depth_loss_mm'],.3)
        self.assertEqual(result['tilt_peak_wrist_force_n'],2.)
        self.assertEqual(result['hold_peak_wrist_force_n'],2.)

    def test_serialized_gate_can_include_boundary_sample(self):
        rows=[dict(row(0.,'insert'),depth_gate_passed=False),
              dict(row(.1,'depth_settle'),depth_gate_passed=True,depth_gate_time_s=.1)]
        result=mechanics_reference_metrics(rows,self.case,.1)
        self.assertTrue(result['aligned_entry_reached'])
        self.assertEqual(result['aligned_entry_gate_time_s'],.1)

    def test_contact_masks_and_same_instant_components_at_peak(self):
        rows=[dict(row(0.,'stop'),recovery_time_s=0.),
              dict(row(.05,'retreat',wrist=5.),recovery_time_s=.05,
                   normal_force_world_x_n=0.,normal_force_world_y_n=0.,normal_force_world_z_n=1.,
                   friction_force_world_x_n=0.,friction_force_world_y_n=0.,friction_force_world_z_n=-6.,
                   contact_axial_span_mm=10.,contact_axial_span_valid=True,
                   wall_centroid_axial_span_mm=9.,wall_both_sides_loaded=False,
                   geometry_overlap_estimate_mm=.02,geometry_overlap_estimate_valid=True),
              dict(row(.1,'retreat',wrist=3.),recovery_time_s=.1,
                   normal_force_world_x_n=0.,normal_force_world_y_n=0.,normal_force_world_z_n=-2.,
                   friction_force_world_x_n=0.,friction_force_world_y_n=0.,friction_force_world_z_n=-1.,
                   contact_axial_span_mm=12.,contact_axial_span_valid=False,
                   wall_centroid_axial_span_mm=4.,wall_both_sides_loaded=True)]
        result=mechanics_recovery_metrics(rows,.05)
        self.assertEqual(result['retreat_peak_contact_normal_axial_resistance_n'],2.)
        self.assertEqual(result['retreat_peak_contact_friction_axial_resistance_n'],6.)
        self.assertEqual(result['retreat_at_wrist_peak_normal_force_world_z_n'],1.)
        self.assertEqual(result['retreat_at_wrist_peak_friction_force_world_z_n'],-6.)
        self.assertEqual(result['retreat_peak_contact_axial_span_mm'],10.)
        self.assertEqual(result['retreat_peak_wall_centroid_axial_span_mm'],4.)
        self.assertIsNone(result['retreat_at_wrist_peak_wall_centroid_axial_span_mm'])
        self.assertIsNone(result['retreat_peak_contact_axial_span_mm_mean100ms'])
        self.assertEqual(result['retreat_peak_wrist_force_mean100ms_n'],4.)
        self.assertEqual(rows[1]['wall_centroid_axial_span_mm'],9.)

    def test_unobserved_contact_and_recovery_phases_are_not_zero(self):
        result=mechanics_recovery_metrics([dict(row(0.,'stop'),recovery_time_s=0.)],.1)
        self.assertIsNone(result['retreat_peak_contact_normal_axial_resistance_n'])
        self.assertIsNone(result['realign_peak_wrist_force_n'])
        self.assertFalse(result['retreat_observed'])


class MechanicsSummaryTests(unittest.TestCase):
    def test_plot_masks_preserve_rejects_missing_and_censored_costs(self):
        valid=dict(reference_state='valid_protocol_completed', straight_numerically_valid=True,
                   straight_safe_recovery=True, straight_cleared=True, straight_recovery_censored=False,
                   straight_retreat_peak_wrist_force_n=0.)
        key='straight_retreat_peak_wrist_force_n'
        self.assertEqual(measurement_cell(valid,key,recovery=True),(0.,None))
        self.assertEqual(measurement_cell(dict(valid,reference_state='numerically_invalid'),key,recovery=True),(None,'NV'))
        self.assertEqual(measurement_cell(dict(valid,straight_recovery_censored=True),key,recovery=True),(None,'RC'))
        self.assertEqual(measurement_cell(dict(valid,straight_retreat_peak_wrist_force_n=None),key,recovery=True),(None,'NA'))
        self.assertEqual(measurement_cell(dict(valid,reference_state='not_started'),key,recovery=True),(None,'—'))
        self.assertEqual(measurement_cell(dict(valid,reference_state='excluded'),key,recovery=True),(None,'EX'))

    def test_partial_plan_preserves_24_cases_and_exclusions(self):
        plan=make_plan();case=plan['cases'][0]
        manifest=dict(study='Forge-mechanics-v1',status='running',case_plan=plan,
                      attempts=[completed_attempt(case)],
                      skipped_cases=[{'case_id':plan['cases'][-1]['case_id'],'reason':'aligned_control_failed'}])
        result=summarize_manifest(manifest)
        self.assertEqual(len(result['cases']),24)
        self.assertEqual(result['summary']['counts']['completed_cases'],1)
        self.assertEqual(result['cases'][-1]['reference_state'],'excluded')
        self.assertEqual(result['cases'][-1]['exclusion_reason'],'aligned_control_failed')

    def test_tolerance_matched_prefix_does_not_require_bitwise_equality(self):
        plan=make_plan();attempt=completed_attempt(plan['cases'][0],prefix=True,equal=False)
        result=summarize_manifest(dict(study='Forge-mechanics-v1',case_plan=plan,attempts=[attempt]))
        first=result['cases'][0]
        self.assertTrue(first['paired_results_known'])
        self.assertFalse(first['realign_replay_prefix_equal'])
        self.assertEqual(first['paired_recorded_retreat_peak_wrist_force_n_reduction_n'],1.75)
        self.assertEqual(result['summary']['counts']['matched_policy_pairs'],1)

    def test_unknown_prefix_keeps_continuous_cost_but_no_pair(self):
        plan=make_plan();attempt=completed_attempt(plan['cases'][0],prefix=False,equal=False)
        result=summarize_manifest(dict(study='Forge-mechanics-v1',case_plan=plan,attempts=[attempt]))
        first=result['cases'][0]
        self.assertEqual(first['straight_retreat_peak_wrist_force_n'],2.)
        self.assertEqual(first['paired_state'],'replay_prefix_unmatched')
        self.assertFalse(first['paired_results_known'])
        self.assertIsNone(first['paired_recorded_retreat_peak_wrist_force_n_reduction_n'])
        overview=result['overview'][0]
        self.assertEqual(len(overview),25)
        self.assertEqual(overview['straight_retreat_peak_wrist_force_n'],2.)
        self.assertIsNone(overview['matched_realign_retreat_peak_wrist_force_n'])
        self.assertIsNone(overview['matched_realign_max_wrist_force_n'])
        self.assertEqual(len(result['overview']),24)
        self.assertIsNone(result['overview'][-1]['straight_retreat_peak_wrist_force_n'])

    def test_invalid_control_does_not_create_factor_effect(self):
        plan=make_plan();control=plan['cases'][0]
        tilted=next(c for c in plan['cases'] if c['tilt_amplitude_deg']==2. and
                    all(c[k]==control[k] for k in ('target_depth_mm','pair_static_friction','pair_dynamic_friction')))
        bad=completed_attempt(control);bad['metrics']['numerically_valid']=False
        good=completed_attempt(tilted)
        manifest=dict(study='Forge-mechanics-v1',case_plan=plan,attempts=[bad,good])
        frozen=copy.deepcopy(manifest)
        result=summarize_manifest(manifest)
        effect=next(e for e in result['control_comparisons'] if e['case_id']==tilted['case_id'])
        self.assertFalse(effect['reference_comparison_eligible'])
        self.assertIsNone(effect['difference_tilt_terminal_depth_loss_mm'])
        self.assertEqual(manifest,frozen)


if __name__=='__main__':unittest.main()
