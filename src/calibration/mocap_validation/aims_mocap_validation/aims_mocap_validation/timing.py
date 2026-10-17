"""Clock-domain separated timing diagnostics, never a pure compute-time claim."""
import csv
import json
from pathlib import Path
from collections import defaultdict
import numpy as np
from .analyze import age_statistics,sha
from .reference import map_time


def analyze(folder,config,output):
    folder,output=Path(folder),Path(output)
    if output.exists():raise ValueError('timing output directory must not exist')
    path=folder/'timing.csv';groups=defaultdict(list)
    if path.exists():
        with path.open() as stream:
            for row in csv.DictReader(stream):groups[row['topic']].append(row)
    result=dict(topics={},physical_acceptance=False,clock_domains='DDS/system timestamps kept separate from ROS and steady clocks',source=str(path),available=path.exists())
    if path.exists():result['input_sha256']=sha(path)
    for topic,rows in groups.items():
        metrics=dict(messages=len(rows),missing_source=sum(not r['source_ns'] for r in rows),missing_dds_pub=sum(not r['dds_pub_ns'] for r in rows))
        series=defaultdict(list)
        for row in rows:
            source=int(row['source_ns']) if row['source_ns'] else None
            if topic==config.get('vicon',{}).get('topic') and source is not None:
                try:source=int(map_time([source],config.get('clock',{}))[0])
                except (ValueError,KeyError,TypeError):source=None;metrics['vicon_clock_unmapped']=True
            pub=int(row['dds_pub_ns']) if row['dds_pub_ns'] else None
            receive=int(row['receive_system_ns']);ros=int(row['receive_ros_ns'])
            if pub is not None:series['dds_to_callback_ms'].append((receive-pub)*1e-6)
            if source is not None:
                series['source_to_callback_ros_ms'].append((ros-source)*1e-6)
                if pub is not None:
                    # Only combine ROS header and DDS system epoch where the
                    # receiving ROS and system clocks agree to within 1 ms.
                    if abs(ros-receive)<=1_000_000:series['source_to_dds_ms'].append((pub-source)*1e-6)
                    else:metrics['ros_system_clock_mismatch']=True
        for name,values in series.items():metrics[name]=age_statistics(values)
        steady=np.asarray([int(r['receive_steady_ns']) for r in rows],np.int64)
        if len(steady)>1:metrics['callback_period_ms']=age_statistics(np.diff(steady)*1e-6)
        result['topics'][topic]=metrics
    health=folder/'timing.health.json'
    if health.exists():result['collector_health']=json.loads(health.read_text())
    output.mkdir(parents=True);(output/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    text=['# 消息时间统计','', '源时间→DDS 发布、DDS 发布→采集回调、源时间→ROS 接收分别统计；不相加，也不称为纯计算耗时。','', '| 话题 | 数量 | 源→发布 P95 ms | 发布→回调 P95 ms | 源→回调 P95 ms |','|---|---:|---:|---:|---:|']
    for topic,m in result['topics'].items():
        def value(name):return f"{m[name]['p95']:.3f}" if name in m else '不可用'
        text.append(f"| {topic} | {m['messages']} | {value('source_to_dds_ms')} | {value('dds_to_callback_ms')} | {value('source_to_callback_ros_ms')} |")
    text+=['','负值、缺失 DDS 时间、时钟跳变及队列丢失保留在 JSON。Vicon 未对时则不计算其源时间年龄；VRPN 时间不自动等于相机采样时刻。']
    (output/'report.zh-CN.md').write_text('\n'.join(text)+'\n');return result
