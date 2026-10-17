import unittest
import numpy as np
from evaluate import interpolate,residual
class QualityContracts(unittest.TestCase):
    def test_interpolation_preserves_gaps_and_exact_samples(self):
        y=interpolate([0.,.005,.1],[1.,2.,3.],np.array([-.01,.0025,.05,.1,.11]))
        self.assertTrue(np.isnan(y[[0,2,4]]).all());self.assertEqual(y[1],1.5);self.assertEqual(y[3],3.)
    def test_residual_does_not_bridge_missing_data(self):
        x=np.r_[np.ones(40),np.full(10,np.nan),np.ones(40)*5]
        y=residual(x);self.assertTrue(np.isnan(y[30:60]).all());self.assertLess(np.nanmax(abs(y)),1e-10)
    def test_cubic_trend_is_not_called_noise(self):
        x=np.arange(100)*.005;y=residual(x**3+2*x)
        self.assertLess(np.nanmax(abs(y)),1e-10)
if __name__=='__main__':unittest.main()
