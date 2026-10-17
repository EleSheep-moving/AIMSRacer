import importlib.util
from pathlib import Path
import tempfile
import unittest
import numpy as np

P=Path(__file__).with_name('quality.py')

class Contracts(unittest.TestCase):
    def setUp(self):
        self.assertTrue(P.exists(), 'quality implementation missing')
        s=importlib.util.spec_from_file_location('quality',P); self.m=importlib.util.module_from_spec(s);s.loader.exec_module(self.m)
    def test_arrival_order_preserves_delayed_source(self):
        rows=self.m.receipt_order([(20,1,'L'),(10,9,'I'),(20,2,'I')])
        self.assertEqual(rows,[(10,9,'I'),(20,1,'L'),(20,2,'I')])
    def test_oadev_constant_and_support(self):
        rows=self.m.oadev(np.ones((100,3)),.01,[1,4,10])
        self.assertEqual([r['pairs'] for r in rows],[99,93,81])
        self.assertEqual(rows[2]['tau_s'],.1)
        np.testing.assert_array_equal(rows[0]['deviation'],[0.,0.,0.])
    def test_calibration_known_poses_and_rank_guard(self):
        truth=np.vstack([np.eye(3),-np.eye(3)])*9.80665
        bias=np.array([.1,-.2,.3]);scale=np.array([1.02,.98,1.01])
        fit=self.m.fit_known_poses(truth*scale+bias,truth)
        np.testing.assert_allclose(fit['bias'],bias,atol=1e-12)
        np.testing.assert_allclose(fit['scale'],scale,atol=1e-12)
        with self.assertRaises(ValueError):self.m.fit_known_poses(np.ones((5,3)),np.ones((5,3)))
    def test_psd_never_bridges_gap(self):
        t=np.r_[np.arange(500)*.005,5+np.arange(500)*.005]
        rows=self.m.segment_welch(t,np.zeros((1000,3)),nperseg=256)
        self.assertEqual(rows['segments'],2)
        self.assertEqual(rows['samples'],1000)
        self.assertEqual(len(rows['frequency_hz']),129)
    def test_reference_metadata_guard_and_quadratic(self):
        t=np.arange(101)*.01
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.csv';p.write_text('source_ns,receive_ns,px,py,pz,qx,qy,qz,qw\n'+''.join(f'{int(x*1e9)},{int(x*1e9+1e6)},{x*x},0,0,0,0,0,1\n' for x in t))
            with self.assertRaises(ValueError):self.m.load_reference(p,{})
            meta={'clock_scale':1.,'clock_offset_ns':0,'clock_uncertainty_ns':1000,'rotation_xyzw':[0,0,0,1],'translation_m':[0,0,0],'window_s':.11,'polyorder':3,'body_origin_in_marker_m':[0,0,0],'body_orientation_in_marker_xyzw':[0,0,0,1]}
            r=self.m.load_reference(p,meta)
            np.testing.assert_allclose(r['acceleration_world'][10:-10,0],2.,atol=1e-7)
            self.assertTrue(r['offline_centered_future_samples'])


class RegressionHistory(unittest.TestCase):
    def test_trailing_linear_fit_preserves_slope_and_midpoint(self):
        s=importlib.util.spec_from_file_location('quality',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
        self.assertTrue(hasattr(m,'trailing_derivative'),'trailing derivative missing')
        t=np.arange(20)*.005;w=np.column_stack([2*t,3*t,4*t])
        alpha,age=m.trailing_derivative(t,w)
        np.testing.assert_allclose(alpha[5:],np.tile([2,3,4],(15,1)),atol=1e-12)
        np.testing.assert_allclose(age[5:],.0125,atol=1e-12)

class ExactArrival(unittest.TestCase):
    def test_corrected_receipt_is_preserved(self):
        s=importlib.util.spec_from_file_location('quality',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
        self.assertTrue(hasattr(m,'corrected_events'),'corrected event support missing')
        z={'corrected':np.array([[1.,2.,3.,4.,5.,6.]]),'corrected_ts':np.array([[30,10]],dtype=np.int64),'lio':np.array([[0.,0.,0.,0.,0.,0.,0.,0.,1.]]),'lio_ts':np.array([[20,1]],dtype=np.int64)}
        rows=m.corrected_events(z)
        self.assertEqual(rows[0],(20,1,'L',0));self.assertEqual(rows[1],(30,10,'I',0))

class ReferenceGeometry(unittest.TestCase):
    def test_rotating_marker_lever_and_edges_and_epoch_stamps(self):
        s=importlib.util.spec_from_file_location('quality',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
        from scipy.spatial.transform import Rotation
        t=np.arange(201)*.01;q=Rotation.from_euler('z',t[:,None]).as_quat();epoch=1792002365474876930
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.csv';p.write_text('source_ns,receive_ns,px,py,pz,qx,qy,qz,qw\n'+''.join(f'{epoch+int(round(x*1e9))},{epoch+int(round(x*1e9))+1000000},0,0,0,{qq[0]},{qq[1]},{qq[2]},{qq[3]}\n' for x,qq in zip(t,q)))
            meta={'clock_scale':1.,'clock_offset_ns':17,'clock_uncertainty_ns':1000,'rotation_xyzw':[0,0,0,1],'translation_m':[0,0,0],'body_origin_in_marker_m':[1,0,0],'body_orientation_in_marker_xyzw':[0,0,0,1],'window_s':.11,'polyorder':3}
            r=m.load_reference(p,meta)
            self.assertIn('valid',r,'reference edge validity missing')
            self.assertFalse(r['valid'][0]);self.assertFalse(r['valid'][-1]);self.assertTrue(r['valid'][100])
            self.assertEqual(r['source_origin_mapped_ns'],epoch+17)
            np.testing.assert_allclose(r['acceleration_world'][10:-10,0],-np.cos(t[10:-10]),atol=.001)
            np.testing.assert_allclose(r['acceleration_body'][10:-10,0],-1.,atol=.001)


class ReferenceCovariance(unittest.TestCase):
    def test_pose_covariance_is_preserved_but_not_precision_claim(self):
        s=importlib.util.spec_from_file_location('quality',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.csv';covnames=[f'cov_{i}{j}' for i in range(6) for j in range(6)]
            p.write_text('source_ns,receive_ns,px,py,pz,qx,qy,qz,qw,'+','.join(covnames)+'\n'+''.join(f'{i*10000000},{i*10000000+1},0,0,0,0,0,0,1,'+','.join(str(.01 if j%7==0 else 0) for j in range(36))+'\n' for i in range(101)))
            meta={'clock_scale':1.,'clock_offset_ns':0,'clock_uncertainty_ns':1000,'rotation_xyzw':[0,0,0,1],'translation_m':[0,0,0],'body_origin_in_marker_m':[0,0,0],'body_orientation_in_marker_xyzw':[0,0,0,1],'window_s':.11,'polyorder':3}
            r=m.load_reference(p,meta)
            self.assertIn('original_pose_covariance',r)
            self.assertEqual(r['original_pose_covariance'].shape,(101,6,6))
            self.assertFalse(r['pose_covariance_unknown'])

if __name__=='__main__':unittest.main()
