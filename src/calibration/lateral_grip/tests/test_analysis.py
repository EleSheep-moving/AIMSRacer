import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('analyze',Path(__file__).parents[1]/'scripts'/'analyze.py')
analyze=importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyze)
class AnalysisTest(unittest.TestCase):
    def test_same_message_acceleration_and_stage_stats(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)
            with (p/'evidence.csv').open('w') as f:
                f.write('channel,source_time,receive_ros,receive_steady,state,direction,stage,x,y,z,quality\n')
                f.write('rear_axle/acceleration,1,1.1,2,RUN,1,0,2,3,0,ok\n')
                f.write('rear_axle/acceleration,2,2.1,3,RUN,1,0,4,5,0,ok\n')
                f.write('rear_axle/velocity,1.5,1.6,2.5,RUN,1,0,1,0,0,ok\n')
            r=analyze.summarize(p)
            self.assertEqual(r['stages'][0]['ax_mean'],3)
            self.assertEqual(r['stages'][0]['ay_mean'],4)
            self.assertEqual(r['stages'][0]['samples'],2)
            self.assertTrue((p/'stage_stats.csv').exists())
            self.assertEqual(json.loads((p/'analysis.json').read_text())['source_alignment'],'AX/AY from same acceleration message; other channels unaligned')
    def test_radius_and_vxr_comparator_keep_each_source_stamp(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)
            (p/'evidence.csv').write_text('channel,source_time,receive_ros,receive_steady,state,direction,stage,x,y,z,quality\n'
                'rear_axle/velocity,1,1.1,2,RUN,1,0,2,0,0,ok\n'
                'rear_axle/yaw_rate,1.01,1.2,2.1,RUN,1,0,0.5,0,0,ok\n'
                'rear_axle/acceleration,1.02,1.3,2.2,RUN,1,0,0,0.8,0,ok\n')
            r=analyze.summarize(p)
            self.assertEqual(r['diagnostic_pairs'][0]['radius_vx_over_yaw_rate_m'],4)
            self.assertEqual(r['diagnostic_pairs'][0]['vx_times_yaw_rate_mps2'],1)
            self.assertEqual(r['diagnostic_pairs'][0]['velocity_source_time'],1)
            self.assertEqual(r['diagnostic_pairs'][0]['yaw_rate_source_time'],1.01)
            self.assertEqual(r['diagnostic_pairs'][0]['measured_ay_mps2'],0.8)
if __name__=='__main__':unittest.main()
