"""Three independent reports sharing immutable extracted inputs."""
import json
from pathlib import Path
from .fastlio import _provenance


def analyze(folder,config,output,task='all'):
    from . import analyze as state,fastlio,actuator,timing
    folder,output=Path(folder),Path(output)
    if output.exists():raise ValueError('analysis output directory must not exist')
    provenance=_provenance(folder,config)
    output.mkdir(parents=True)
    result=dict(synthetic=bool(provenance['extraction_metadata'].get('synthetic',False)),physical_acceptance=False,provenance=provenance,tasks={})
    selected=['state','fastlio','actuator'] if task=='all' else [task]
    lines=['# 动捕校对总报告','', '原始误差不平移曲线，不把时间年龄、波形偏移和物理响应混为同一延迟。','', '数据：'+('**合成示例，不能作为实车测量。**' if result['synthetic'] else '记录数据；参考资格以各子报告为准。'),'']
    for name in selected:
        function={'state':state.analyze,'fastlio':fastlio.analyze,'actuator':actuator.analyze}[name]
        try:
            measured=function(folder,config,output/name)
            result['tasks'][name]=dict(computed=True,report=name+'/report.zh-CN.md',metrics=name+'/metrics.json')
            if name=='state':result['tasks'][name]['complete_required_outputs']=measured['complete_required_outputs']
            lines.append(f'- [{name} 结果]({name}/report.zh-CN.md)')
        except (ValueError,KeyError) as error:
            result['tasks'][name]=dict(computed=False,reason=str(error));lines.append(f'- {name}：尚不能计算，{error}')
    timing.analyze(folder,config,output/'timing');lines.append('- [消息时间与采集健康](timing/report.zh-CN.md)')
    result['all_selected_tasks_computed']=all(item['computed'] for item in result['tasks'].values())
    (output/'config.analysis.json').write_text(json.dumps(config,indent=2)+'\n')
    (output/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'report.zh-CN.md').write_text('\n'.join(lines)+'\n');return result
