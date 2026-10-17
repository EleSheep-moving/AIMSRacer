"""Deterministic synthetic example with independent analytic derivatives; no ROS input."""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .analyze import analyze, sha


def create(output):
    output=Path(output)
    if output.exists():raise ValueError('example output must not exist')
    output.mkdir(parents=True);data=output/'data';data.mkdir()
    config=json.loads(Path(__file__).with_name('config.template.json').read_text())
    source_origin=1_700_000_000_000_000_000;ros_origin=1_800_000_000_000_000_000;scale=1.00001
    config.update(synthetic=True,body_orientation_measured=True)
    config['clock'].update(source_anchor_ns=source_origin,ros_anchor_ns=ros_origin,
                           scale=scale,uncertainty_ns=500_000,verified=True)
    config['vicon'].update(topic='/synthetic/vicon',message_type='geometry_msgs/msg/TransformStamped',
        tracking_semantics='csv_valid_flag',bridge_timestamp_semantics='synthetic camera source time',
        prediction_or_retiming_enabled=False,camera_rate_hz=100.)
    mount=Rotation.from_euler('xyz',[.1,.05,-.3]);rear_marker=np.array([.22,.02,.08]);cg_body=np.array([.21,-.03,0.])
    config['body_orientation_in_marker_xyzw']=mount.as_quat().tolist()
    for name,point in config['references'].items():
        point.update(origin_in_marker_m=(rear_marker+(mount.apply(cg_body) if name=='center_of_mass' else 0)).tolist(),measured=True)

    def kinematics(t,point):
        t=np.asarray(t)
        p=np.column_stack([.4*t+.15*np.sin(1.1*t),.25*np.sin(.8*t),.04*np.sin(.6*t)])
        v=np.column_stack([.4+.165*np.cos(1.1*t),.2*np.cos(.8*t),.024*np.cos(.6*t)])
        a=np.column_stack([-.1815*np.sin(1.1*t),-.16*np.sin(.8*t),-.0144*np.sin(.6*t)])
        q=Rotation.from_euler('xyz',[.25,-.2,.3])*Rotation.from_rotvec(np.column_stack([0*t,0*t,.5*np.sin(1.3*t)]))
        omega=np.column_stack([0*t,0*t,.65*np.cos(1.3*t)])
        alpha=np.column_stack([0*t,0*t,-.845*np.sin(1.3*t)])
        d=cg_body if point=='center_of_mass' else np.zeros(3)
        vb=q.inv().apply(v)+np.cross(omega,d)
        ab=q.inv().apply(a)+np.cross(alpha,d)+np.cross(omega,np.cross(omega,d))
        return p,q,np.column_stack([vb[:,:2],ab[:,:2],omega[:,2]])

    with (data/'poses.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['source_ns','receive_ns','px','py','pz','qx','qy','qz','qw','valid'])
        camera_t=np.arange(1201)*.01;t=camera_t*scale;p,body,_=kinematics(t,'rear_axle');marker=body*mount.inv()
        marker_p=p-marker.apply(rear_marker)
        for i in range(len(t)):
            source=source_origin+int(round(camera_t[i]*1e9));receive=ros_origin+int(round(t[i]*1e9))+8_000_000
            valid=not (4.<t[i]<4.15)
            writer.writerow([source,receive,*marker_p[i],*marker[i].as_quat(),int(valid)])
    with (data/'estimates.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['source_ns','receive_ns','stream','reference','quantity','value','frame_id'])
        t=np.arange(2401)*.005
        for point in ['rear_axle','center_of_mass']:
            for j,(quantity,delay,bias) in enumerate(zip(['vx','vy','ax','ay','yaw_rate'],[.03,.02,.015,.015,.005],[.05,.01,0,0,0])):
                values=kinematics(t-delay,point)[2][:,j]+bias
                for i,value in enumerate(values):
                    if 7.<t[i]<7.2:continue
                    source=ros_origin+int(round(t[i]*1e9));receive=source+(70_000_000 if i%50==0 else 3_000_000)
                    writer.writerow([source,receive,'own_'+point,point,quantity,value,'base_link' if point=='rear_axle' else 'center_of_mass'])
        for j,quantity,delay in [(0,'vx',.06),(1,'vy',.22),(4,'yaw_rate',.05)]:
            values=kinematics(t-delay,'rear_axle')[2][:,j]
            for i,value in enumerate(values):
                source=ros_origin+int(round(t[i]*1e9))
                writer.writerow([source,source+2_000_000,'robot_localization','rear_axle',quantity,value,'base_link'])
    metadata=dict(source='analytic synthetic example; not measured vehicle data',synthetic=True,
        analytic_imposed_delays_s=dict(vx=.03,vy=.02,ax=.015,ay=.015,yaw_rate=.005),
        data_sha256={name:sha(data/name) for name in ['poses.csv','estimates.csv']})
    (data/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    (output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    return analyze(data,config,output/'report')
