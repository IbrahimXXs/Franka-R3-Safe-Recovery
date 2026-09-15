"""Checks of probe signs, actual-motion differences, frame handling, and exclusions."""
import unittest
import numpy as np
from research.active_probing import schedule, waveform, summarize, central_difference, POSE, WRENCH


def metric(x, w):
    return dict(zip(POSE,x), **{f'delta_{k}':v for k,v in zip(WRENCH,w)})


class ActiveProbingTests(unittest.TestCase):
    def test_balanced_schedule_reproducible_and_bounded(self):
        specs=schedule()
        self.assertEqual(specs,schedule());self.assertEqual(len(specs),18)
        for repeat in (0,1):
            rows=[r for r in specs if r['repeat']==repeat]
            self.assertEqual(len({(r['axis'],r['sign']) for r in rows}),9)
            self.assertEqual(sum(r['axis']=='hold' for r in rows),1)
            for r in rows:
                expected=0 if r['axis']=='hold' else (.0001 if r['axis'] in ('x','y') else np.deg2rad(.2))
                self.assertAlmostEqual(r['amplitude_si'],expected)
        with self.assertRaises(ValueError):schedule(1)
        with self.assertRaises(ValueError):schedule(100)

    def test_waveform_returns_to_zero_and_has_dwell(self):
        for t,v in ((0,0),(.25,0),(.5,1),(.75,1),(1,0),(1.5,0)):
            self.assertAlmostEqual(waveform(t)[1],v)
        self.assertEqual(waveform(.6)[0],'during')
        self.assertEqual(waveform(1.2)[0],'after')
        self.assertAlmostEqual(waveform(.375)[1],.5)
        self.assertAlmostEqual(waveform(.875)[1],.5)

    def test_actual_motion_denominator_not_command_amplitude(self):
        xp=np.array([.00008,0,0,0,0,0]);xm=np.array([-.00006,0,0,0,0,0])
        g=np.array([1000,200,300,4,5,6])
        out=central_difference(metric(xp,g*xp[0]),metric(xm,g*xm[0]),'x')
        self.assertTrue(out['valid']);self.assertAlmostEqual(out['actual_span_si'],.00014)
        np.testing.assert_allclose([out[f'd_{k}_per_si'] for k in WRENCH],g)

    def test_opposite_actual_signs_and_motion_required(self):
        for x in (0.,1e-8,.0001):
            out=central_difference(metric([.0001,0,0,0,0,0],[1]*6),metric([x,0,0,0,0,0],[-1]*6),'x')
            self.assertFalse(out['valid'])
            self.assertNotIn('d_fx_per_si',out)

    def test_coupled_motion_does_not_claim_partial_derivative(self):
        xp=np.array([.0001,.0002,0,0,0,0]);xm=-xp
        out=central_difference(metric(xp,[1]*6),metric(xm,[-1]*6),'x')
        self.assertFalse(out['valid']);self.assertEqual(out['reason'],'coupled_motion')
        self.assertAlmostEqual(out['off_axis_ratio'],2.)

    def test_nonfinite_central_data_is_rejected(self):
        for values in ([float('nan')]*6,[float('inf')]*6):
            with self.assertRaises(ValueError):
                central_difference(metric(values,[1]*6),metric([-.001]*6,[-1]*6),'x')
            with self.assertRaises(ValueError):
                central_difference(metric([.001]*6,values),metric([-.001]*6,[-1]*6),'x')

    def test_rotation_uses_radians(self):
        x=np.array([0,0,0,np.deg2rad(.15),0,0])
        out=central_difference(metric(x,[2]*6),metric(-x,[-2]*6),'roll')
        self.assertTrue(out['valid']);self.assertAlmostEqual(out['d_fx_per_si'],2/np.deg2rad(.15))

    def test_tail_means_torque_transport_and_return_hysteresis(self):
        rows=[]
        # Constant force at the fixed world origin: moving-origin raw torque changes,
        # but transported wrench must not. Constant spatial rotation near 180 yaw.
        for i in range(181):
            t=i/120;phase,u=waveform(t)
            p=np.array([.0001*u,0,0]);f=np.array([0.,2.,0.]);torque=-np.cross(p,f)
            q=[0,0,0,1]
            rows.append(dict(phase=phase,probe_time_s=t,depth_mm=10,normal_load_n=2,
                peg_world_x_m=p[0],peg_world_y_m=0,peg_world_z_m=0,
                qw=q[0],qx=q[1],qy=q[2],qz=q[3],**dict(zip(WRENCH,np.r_[f,torque]))))
        out=summarize(rows,[0,0,0])
        self.assertAlmostEqual(out['dx_m'],.0001)
        self.assertAlmostEqual(out['response_n_equiv'],0)
        self.assertAlmostEqual(out['return_position_error_mm'],0)
        with self.assertRaises(ValueError):summarize(rows[:40],[0,0,0])


    def test_analysis_pipeline_and_invalid_holds(self):
        import csv
        import json
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from research.active_probing import analyze, AXES
        from research.forge_protocol import protocol
        from dataclasses import asdict
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            probes=[]
            for spec in schedule():
                records=[]
                for i in range(181):
                    phase,u=waveform(i/120)
                    xi=np.zeros(6)
                    if spec['axis']!='hold':xi[AXES[spec['axis']]]=u*spec['sign']*spec['amplitude_si']
                    theta=xi[3:];angle=np.linalg.norm(theta)
                    q=np.r_[np.cos(angle/2),theta/angle*np.sin(angle/2)] if angle else np.array([1,0,0,0])
                    wrench=np.array([1000,1100,1200,2,3,4])*xi
                    # Synthetic independent channels with a fixed torque anchor.
                    raw=wrench.copy();raw[3:]-=np.cross(xi[:3],wrench[:3])
                    records.append(dict(phase=phase,probe_time_s=i/120,depth_mm=10,normal_load_n=2,
                        peg_world_x_m=xi[0],peg_world_y_m=xi[1],peg_world_z_m=xi[2],
                        qw=q[0],qx=q[1],qy=q[2],qz=q[3],numerically_valid=True,**dict(zip(WRENCH,raw))))
                log=spec['probe_id']+'.csv'
                with (root/log).open('w') as f:
                    writer=csv.DictWriter(f,fieldnames=records[0]);writer.writeheader();writer.writerows(records)
                probes.append(dict(**spec,log=log,eligible=True,completed=True))
            metrics=dict(insertion_success=True,stalled=False,max_depth=10,max_normal_load=2)
            manifest=dict(status='complete',physics_hz=120,repeats=2,protocol=asdict(protocol()),cases=[dict(case_id='synthetic',
                metrics=metrics,archived_metrics=metrics,wrench_anchor_world_m=[0,0,0],probes=probes)])
            (root/'pilot.json').write_text(json.dumps(manifest))
            with patch('research.active_probing.plots'), patch('research.active_probing.wrench_traces'), patch('research.active_probing.match',return_value=(True,{})):
                result=analyze(root)
            self.assertEqual(result['valid_central_pairs'],8)
            self.assertEqual(result['signed_above_baseline'],16)
            self.assertEqual(result['repeatable_signed_directions'],8)
            self.assertEqual(result['distinguishable_axis_pairs'],6)
            with patch('research.active_probing.plots'), patch('research.active_probing.wrench_traces'), patch('research.active_probing.match',return_value=(False,{'force_n':1.})):
                mismatched=analyze(root)
            self.assertEqual(mismatched['valid_central_pairs'],0)
            for probe in probes:
                if probe['axis']=='hold':probe['eligible']=False
            (root/'pilot.json').write_text(json.dumps(manifest))
            with patch('research.active_probing.plots'), patch('research.active_probing.wrench_traces'), patch('research.active_probing.match',return_value=(True,{})):
                result=analyze(root)
            self.assertEqual(result['signed_above_baseline'],0)
            self.assertEqual(result['repeatable_signed_directions'],8)
            self.assertEqual(result['repeatable_above_baseline_signed_directions'],0)
            self.assertEqual(result['distinguishable_axis_pairs'],0)


if __name__=='__main__':unittest.main()
