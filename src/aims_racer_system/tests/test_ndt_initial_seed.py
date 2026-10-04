"""Failed bootstrap hypotheses must not destroy usable candidate evidence."""
import importlib.util
import json
from pathlib import Path
import sys

SCRIPTS=Path(__file__).parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
spec=importlib.util.spec_from_file_location('ndt_seed',SCRIPTS/'ndt_initial_seed.py')
seed=importlib.util.module_from_spec(spec)
spec.loader.exec_module(seed)


def test_failed_candidate_is_serializable_and_good_candidate_retained():
    candidates={'result':{'candidates':[{'fitness':float('inf'),'converged':False},
                                      {'fitness':.003,'converged':True}]},'nan':float('nan')}
    cleaned=seed.json_safe(candidates)
    decoded=json.loads(json.dumps(cleaned,allow_nan=False))
    assert decoded['result']['candidates'][0]['fitness'] is None
    assert decoded['result']['candidates'][1]['fitness']==.003
    assert decoded['nan'] is None
