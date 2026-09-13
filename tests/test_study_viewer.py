"""Viewer data boundaries: invalid costs, missing samples, and safe embedding."""
import json
from pathlib import Path
import tempfile
import unittest
from visualization.view_study import build_dashboard, load_study


class StudyViewerTests(unittest.TestCase):
    def test_manifest_invalid_status_overrides_stale_summary(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'study.json').write_text(json.dumps({'status':'complete_with_invalid_trials',
                'invalid_trials':[{'scenario':'offset','recovery':'straight','status':'numerically_invalid',
                                   'recovery_resistive_work_j':None}]}))
            (p/'summary.csv').write_text('scenario,recovery,status,recovery_resistive_work_j\noffset,straight,cleared_within_budget,4\n')
            t=load_study(p)['trials'][0]
            self.assertFalse(t['eligible'])
            self.assertIsNone(t['summary']['recovery_resistive_work_j'])
            self.assertFalse(t['series'])

    def test_partial_csv_is_incomplete_and_nonfinite_stays_missing(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'study.json').write_text('{"status":"incomplete"}')
            (p/'tilted__straight.csv').write_text('time_s,phase,fz\n0.1,insert,NaN\n0.2,insert,3\n')
            t=load_study(p)['trials'][0]
            self.assertEqual(t['status'],'incomplete')
            self.assertFalse(t['eligible'])
            self.assertEqual(t['series']['fz'],[None,3.0])

    def test_script_text_cannot_escape_data_and_original_files_untouched(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);content=json.dumps({'status':'complete','note':'</script><script>alert(1)</script>@@APP@@'})
            (p/'study.json').write_text(content)
            out=build_dashboard(p)
            page=out.read_text()
            self.assertIn('\\u003c/script>',page)
            self.assertNotIn('</script><script>alert(1)',page)
            self.assertEqual((p/'study.json').read_text(),content)
            self.assertIn('@@APP@@',page)

    def test_invalid_sensor_study_never_eligible(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'study.json').write_text(json.dumps({'status':'invalid_sensor_configuration',
                'completed_trials':[{'scenario':'aligned','recovery':'straight','status':'cleared_within_budget'}]}))
            self.assertFalse(load_study(p)['trials'][0]['eligible'])


if __name__=='__main__':
    unittest.main()
