"""Exact-duration RK4 contracts for the isolated 50 ms evaluation."""
import unittest
import importlib.util
from pathlib import Path
import casadi as ca
import numpy as np
from aims_mpcc.backend_models import NumericalBackend, interval_symbolic
from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


class FractionalInterval(unittest.TestCase):
    def model(self, dt):
        x=ca.SX.sym('x',6);u=ca.SX.sym('u',3);previous=ca.SX.sym('previous')
        end,samples=interval_symbolic(x,u,previous,VehicleConfig(steering_tau=.08),dt)
        return ca.Function('interval',[x,u,previous],[end,ca.horzcat(*samples)])

    def test_fifty_ms_integrates_the_complete_duration(self):
        initial=np.array([1.,-.2,.3,.7,.8,.12]);control=np.array([.4,.17,.65])
        end,samples=self.model(.05)(initial,control,.15)
        self.assertAlmostEqual(float(end[3]),initial[3]+control[0]*.05,places=14)
        self.assertAlmostEqual(float(end[4]),initial[4]+control[2]*.05,places=14)
        self.assertEqual(samples.shape,(6,4),'.02+.02+.01 mesh must include four samples')
        np.testing.assert_allclose(np.asarray(samples)[3],initial[3]+control[0]*np.array([0.,.02,.04,.05]),rtol=0.,atol=1e-14)

    def test_one_hundred_ms_retains_original_rk4_values(self):
        x=np.array([1.,-.2,.3,.7,.8,.12]);u=np.array([.4,.17,.65]);previous=.15;cfg=VehicleConfig(steering_tau=.08)
        def rhs(v,command):
            return np.array([v[3]*np.cos(v[2]),v[3]*np.sin(v[2]),v[3]*np.tan(v[5])/cfg.wheelbase,u[0],u[2],(command-v[5])/cfg.steering_tau])
        expected=x.copy()
        for j in range(5):
            command=previous+(u[1]-previous)*(j+1)/5
            k1=rhs(expected,command);k2=rhs(expected+.01*k1,command)
            k3=rhs(expected+.01*k2,command);k4=rhs(expected+.02*k3,command)
            expected+=.02*(k1+2*k2+2*k3+k4)/6
        actual,_=self.model(.1)(x,u,previous)
        np.testing.assert_allclose(np.asarray(actual).ravel(),expected,rtol=0.,atol=1e-14)

    def test_legacy_backend_still_rejects_fractional_intervals(self):
        theta=np.linspace(0,2*np.pi,40,endpoint=False)
        path=ReferencePath(2*np.c_[np.cos(theta),np.sin(theta)],1.,1.)
        cfg=VehicleConfig(profile='synthetic',rear_offset=.18,half_length=.5,half_width=.3,geometry_verified=True)
        with self.assertRaisesRegex(ValueError,'multiple of 20 ms'):
            NumericalBackend(path,cfg,horizon=20,dt=.05)

    def test_legacy_near_multiples_retain_original_mesh(self):
        initial=np.array([1.,-.2,.3,.7,.8,.12]);control=np.array([.4,.17,.65])
        expected,_=self.model(.1)(initial,control,.15)
        for dt in (.1000001,.1000000000005):
            with self.subTest(dt=dt):
                actual,samples=self.model(dt)(initial,control,.15)
                self.assertEqual(samples.shape,(6,6),'legacy np.isclose durations require the original six samples')
                np.testing.assert_allclose(np.asarray(actual),np.asarray(expected),rtol=0.,atol=1e-14)

    def test_runtime_export_canonicalizes_supported_near_values(self):
        script=Path(__file__).resolve().parents[1]/'scripts/export_bundle.py'
        spec=importlib.util.spec_from_file_location('runtime_export',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertTrue(hasattr(module,'canonical_runtime_dt'),'runtime exporter must normalize accepted stage durations')
        for dt,expected in ((.1,.1),(.1000000000005,.1),(.05,.05),(.0500000000005,.05)):
            self.assertEqual(module.canonical_runtime_dt(dt),expected)


if __name__=='__main__':unittest.main()
