"""One template, explicit site data, no fabricated clock or geometry calibration."""
from copy import deepcopy
import json
from pathlib import Path


def load(path):
    result=json.loads(Path(__file__).with_name('config.template.json').read_text())
    if path is None:return result
    incoming=json.loads(Path(path).read_text())
    def merge(target,source):
        for key,value in source.items():
            if isinstance(value,dict) and isinstance(target.get(key),dict):merge(target[key],value)
            else:target[key]=deepcopy(value)
    merge(result,incoming)
    tracker=result['vrpn'].get('tracker')
    if result['vicon']['topic'] is None and tracker:
        import re
        node=re.sub('_+','_',re.sub('[^a-zA-Z0-9_]','_',tracker))
        result['vicon']['topic']='/vrpn_mocap/'+node+'/pose'
        result['vicon']['message_type']='geometry_msgs/msg/PoseStamped'
    return result
