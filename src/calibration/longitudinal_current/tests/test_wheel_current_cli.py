import json
from pathlib import Path
import pytest


def cli():
    import importlib.util
    assert importlib.util.find_spec('aims_calibration.wheel_current_cli') is not None, 'wheel-current CLI is missing'
    from aims_calibration import wheel_current_cli
    return wheel_current_cli


def artifact(tmp_path):
    from aims_calibration.wheel_current_fit import fit_model
    from test_wheel_current_fit import synthetic_records
    p=tmp_path/'model.json'
    p.write_text(json.dumps(fit_model(synthetic_records(),dict(coefficients_ascending=[10.],speed_domain_mps=[1.,5.]))))
    return p


def test_query_command_forward_and_inverse(tmp_path,capsys):
    p=artifact(tmp_path)
    assert cli().query_main(['--model',str(p),'--speed','2','--iq','70'])==0
    forward=json.loads(capsys.readouterr().out);assert forward['valid'] and forward['value']==pytest.approx(6.)
    assert cli().query_main(['--model',str(p),'--speed','2','--acceleration','6'])==0
    assert json.loads(capsys.readouterr().out)['value']==pytest.approx(70.)


def test_query_unsupported_is_structured_nonzero(tmp_path,capsys):
    assert cli().query_main(['--model',str(artifact(tmp_path)),'--speed','0','--iq','80'])==2
    r=json.loads(capsys.readouterr().out);assert not r['valid'] and r['reason']=='speed_out_of_domain'


def test_query_corrupt_json_is_structured_nonzero(tmp_path,capsys):
    p=tmp_path/'bad.json';p.write_text('{')
    assert cli().query_main(['--model',str(p),'--speed','2','--iq','80'])==2
    assert json.loads(capsys.readouterr().out)['reason']=='invalid_model'


def test_fit_rejects_nonempty_output_before_source_reads(tmp_path,capsys):
    output=tmp_path/'results';output.mkdir();source=tmp_path/'does-not-exist'
    (output/'keep.txt').write_text('keep')
    assert cli().fit_main(['--source-root',str(source),'--output',str(output)])==2
    r=json.loads(capsys.readouterr().out)
    assert 'not empty'in r['error'] and (output/'keep.txt').read_text()=='keep'


def test_fit_must_not_target_original_evidence(tmp_path,capsys):
    root=tmp_path/'sources';root.mkdir()
    assert cli().fit_main(['--source-root',str(root),'--output',str(root/'raw-extracts')])==2
    assert 'source'in json.loads(capsys.readouterr().out)['error']
