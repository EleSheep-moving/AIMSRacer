import json
from types import SimpleNamespace
import pytest
from aims_calibration import cli


def test_help_and_results_do_not_import_ros(tmp_path,capsys):
    for main in (cli.inspect_main,cli.start_main,cli.stop_main,cli.results_main):
        with pytest.raises(SystemExit) as exit:
            main(['--help'])
        assert exit.value.code==0
    path=tmp_path/'trials.json';path.write_text(json.dumps({}))
    assert cli.results_main([str(path)])==3
    assert 'metadata_evidence_invalid' in capsys.readouterr().out


def test_start_never_retries_enable_on_uncertain_response(monkeypatch):
    calls=[]
    class Monitor:
        def __init__(self,*args): pass
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def wait_status(self,timeout):
            return {'preflight_json':json.dumps({'ready':True}), 'start_ready':'true'}
        def request(self,value,timeout):
            calls.append(value)
            if value: raise RuntimeError('service response timeout')
    monkeypatch.setattr(cli,'Monitor',Monitor)
    monkeypatch.setattr(cli,'check_start',lambda data:[])
    assert cli.start_main(['--timeout','1'])==3
    assert calls==[True,False]


def test_start_refusal_never_calls_enable(monkeypatch):
    calls=[]
    class Monitor:
        def __init__(self,*args): pass
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def wait_status(self,timeout): return {'preflight_json':'{}'}
        def request(self,*args): calls.append(args)
    monkeypatch.setattr(cli,'Monitor',Monitor)
    assert cli.start_main([])==2
    assert calls==[]
