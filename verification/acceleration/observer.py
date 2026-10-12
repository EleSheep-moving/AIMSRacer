"""Offline import compatibility for the canonical runtime observer core."""
from pathlib import Path
import sys

# Offline replay scripts run directly from verification/acceleration without an
# installed ROS workspace. The estimator itself has no ROS dependency.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/aims_racer_system'))
from aims_racer_system.longitudinal_observer import Estimate, LongitudinalObserver, ObserverConfig

__all__ = ['Estimate', 'LongitudinalObserver', 'ObserverConfig']
