import importlib.util
from pathlib import Path

def comparison():
    path=Path(__file__).resolve().parents[1]/'tools/compare_output_profiles.py'
    spec=importlib.util.spec_from_file_location('compare_output_profiles',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.compare_reports

def test_failed_baseline_is_unavailable_not_passing():
    answer=comparison()({'overall_pass':False},{'overall_pass':True})
    assert answer['status']=='unavailable' and not answer['relative_pass']

def test_relative_tracking_thresholds_and_new_failure():
    base=dict(overall_pass=True,contour_p95_m=.02,heading_p95_rad=.03)
    good=dict(overall_pass=True,contour_p95_m=.03,heading_p95_rad=.04)
    assert comparison()(base,good)['relative_pass']
    assert not comparison()(base,{**good,'contour_p95_m':.04})['relative_pass']
    assert not comparison()(base,{**good,'overall_pass':False})['relative_pass']
