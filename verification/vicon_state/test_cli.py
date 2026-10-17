import json
from pathlib import Path
import subprocess
import sys

import pytest

from test_bag import bag_config, fixture_bag
from test_pipeline import analyzer, example_data


def test_cli_real_serialized_bag_to_complete_report(tmp_path):
    run=Path(__file__).with_name('run.py');assert run.exists()
    config=bag_config();config['body_orientation_measured']=True
    config['vicon'].update(bridge_timestamp_semantics='synthetic camera source',prediction_or_retiming_enabled=False)
    config_path=tmp_path/'config.json';config_path.write_text(json.dumps(config))
    bag=tmp_path/'bag';fixture_bag(bag,samples=301)
    for action,arguments in [
        ('extract',['--bag',str(bag),'--output',str(tmp_path/'extracted')]),
        ('analyze',['--input',str(tmp_path/'extracted'),'--output',str(tmp_path/'report')])]:
        result=subprocess.run([sys.executable,str(run),action,'--config',str(config_path),*arguments],capture_output=True,text=True)
        assert result.returncode==0,result.stdout+result.stderr
    metrics=json.loads((tmp_path/'report/metrics.json').read_text())
    assert metrics['complete_required_outputs']
    assert metrics['comparisons']['whole/own_rear_velocity/rear_axle/vx']['windows']['0.12']['rmse']==pytest.approx(.1,abs=1e-6)
    assert metrics['status_message_counts']['own_rear_status']['vx_valid=true']==301
    common=metrics['common_grid_comparisons']['whole/rear_axle/vx']
    assert common['streams']['robot_localization']['n']==common['streams']['own_rear_velocity']['n']==common['n']
    assert common['streams']['own_rear_velocity']['rmse']==pytest.approx(.1,abs=1e-6)


def test_missing_output_gap_and_wrong_frame_remain_visible(tmp_path):
    import csv
    m=analyzer();folder=tmp_path/'data';config=example_data(folder)
    with (folder/'estimates.csv').open() as stream:
        reader=csv.DictReader(stream);fields=reader.fieldnames;rows=list(reader)
    origin=config['clock']['ros_anchor_ns'];kept=[]
    for row in rows:
        if row['reference']=='center_of_mass' and row['quantity']=='ay':continue
        t=(int(row['source_ns'])-origin)*1e-9
        if row['stream']=='own_rear_axle' and row['quantity']=='vx' and 1.5<t<2.5:continue
        kept.append(row)
    kept[0]['frame_id']='wrong_frame'
    with (folder/'estimates.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(kept)
    result=m.analyze(folder,config,tmp_path/'report')
    assert not result['complete_required_outputs']
    assert result['rejected_estimates']['wrong_frame']==1
    assert result['comparisons']['whole/own_rear_axle/rear_axle/vx']['reference_grid_coverage']['fraction']<.9


def test_tampered_extracted_csv_is_rejected_before_scoring(tmp_path):
    m=analyzer();folder=tmp_path/'data';config=example_data(folder)
    (folder/'metadata.json').write_text(json.dumps(dict(data_sha256={'estimates.csv':'bad'})))
    with pytest.raises(ValueError,match='hash'):m.analyze(folder,config,tmp_path/'report')
