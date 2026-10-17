#!/usr/bin/env python3
"""Compatibility CLI; implementation is owned by aims_mocap_validation."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/calibration/mocap_validation/aims_mocap_validation'))
from aims_mocap_validation.cli import main
if __name__=='__main__':main(legacy=True)
