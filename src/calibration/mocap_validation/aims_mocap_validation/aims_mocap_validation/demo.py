"""Synthetic three-task session; never represents measured vehicle accuracy."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.spatial.transform import Rotation
from .configuration import load
from .analyze import sha
from .pipeline import analyze


def create(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);folder=output/'data';folder.mkdir()
    config=load(None);config['synthetic']=True;config['vrpn']['start_bridge']=False
    origin=1_800_000_000_000_000_000
    config['clock'].update(source_anchor_ns=origin,ros_anchor_ns=origin,scale=1.,uncertainty_ns=500000,verified=True)
    config['vicon'].update(topic='/synthetic/vicon',message_type='geometry_msgs/msg/PoseStamped',tracking_semantics='csv_valid_flag',camera_rate_hz=100.,bridge_timestamp_semantics='synthetic source',prediction_or_retiming_enabled=False)
    config['body_orientation_in_marker_xyzw']=[0.,0.,0.,1.];config['body_orientation_measured']=True
    config['references']['rear_axle'].update(origin_in_marker_m=[0.,0.,0.],measured=True)
    config['references']['center_of_mass'].update(origin_in_marker_m=[.21,-.03,0.],measured=True)
    config['fastlio'].update(geometry_measured=True);config['fastlio']['world_alignment']=dict(rotation_xyzw=[0.,0.,0.,1.],translation_m=[0.,0.,0.],static_window_s=None)
    t=np.arange(2401)*.005;ns=origin+np.rint(t*1e9).astype(np.int64)
    vx=np.clip(.6*(t-2.2),0,.6)-np.clip(.3*(t-6.2),0,.3)
    dvx=.6*((t>2.2)&(t<3.2))-.3*((t>6.2)&(t<7.2))
    activation=np.clip((t-2.2)/.5,0,1);vy=.05*np.sin(.8*t)*activation
    dvy=.04*np.cos(.8*t)*activation+.1*np.sin(.8*t)*((t>2.2)&(t<2.7))
    r=.3*np.clip((t-8.15)/.1,0,1)-.3*np.clip((t-11.15)/.1,0,1)
    alpha=3.*((t>8.15)&(t<8.25))-3.*((t>11.15)&(t<11.25))
    yaw=cumulative_trapezoid(r,t,initial=0.);q=Rotation.from_euler('z',yaw)
    worldv=q.apply(np.column_stack([vx,vy,0*t]));p=cumulative_trapezoid(worldv,t,axis=0,initial=0.)
    a=np.column_stack([dvx-r*vy,dvy+r*vx,0*t]);omega=np.column_stack([0*t,0*t,r]);angularacc=np.column_stack([0*t,0*t,alpha])
    def write(name,fields,rows):
        with (folder/name).open('w') as stream:
            writer=csv.writer(stream);writer.writerow(fields);writer.writerows(rows)
    write('poses.csv',['source_ns','receive_ns','px','py','pz','qx','qy','qz','qw','valid'],[[int(ns[i]),int(ns[i])+8000000,*p[i],*q[i].as_quat(),1] for i in range(0,len(t),2)])
    estimates=[]
    for point,d in [('rear_axle',np.zeros(3)),('center_of_mass',np.array([.21,-.03,0.]))]:
        values=np.column_stack([vx,vy])+np.cross(omega,d)[:,:2]
        accelerations=a+np.cross(angularacc,d)+np.cross(omega,np.cross(omega,d))
        truth=np.column_stack([values,accelerations[:,:2],r])
        for j,quantity in enumerate(['vx','vy','ax','ay','yaw_rate']):
            observed=np.interp(t-.02,t,truth[:,j])+(.05 if quantity=='vx' else 0)
            for i,value in enumerate(observed):estimates.append([int(ns[i]),int(ns[i])+3000000,'own_'+point,point,quantity,float(value),'base_link' if point=='rear_axle' else 'center_of_mass'])
    write('estimates.csv',['source_ns','receive_ns','stream','reference','quantity','value','frame_id'],estimates)
    odometry=[];lever=np.array([.30,0.,.03]);sensorp=p+q.apply(lever);sensorv=worldv+q.apply(np.cross(omega,lever))
    for i in range(0,len(t),20):
        odometry.append([int(ns[i]),int(ns[i])+40000000,'raw_fastlio','odom','livox_frame',*sensorp[i],*q[i].as_quat(),*sensorv[i],0.,0.,0.])
        odometry.append([int(ns[i]),int(ns[i])+42000000,'rear_fastlio','odom','base_link',*p[i],*q[i].as_quat(),vx[i],vy[i],0.,0.,0.,r[i]])
    write('odometry.csv',['source_ns','receive_ns','stream','frame_id','child_frame_id','px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz'],odometry)
    write('gyro.csv',['source_ns','receive_ns','wx','wy','wz'],[[int(n),int(n)+1000000,0.,0.,rate] for n,rate in zip(ns,r)])
    commands=[];telemetry=[]
    for i in range(0,len(t),2):
        x=t[i];speed=1. if 2<=x<6 else .5 if x>=6 else 0.;steering=.15 if 8<=x<11 else 0.
        commands.append([int(ns[i]),int(ns[i])+1000000,0,steering,speed,0.,0.,0.])
        erpm=2000. if 2.12<=x<6.12 else 1000. if x>=6.12 else 0.
        telemetry.append([int(ns[i]),int(ns[i])+2000000,erpm,12.,12.])
    write('commands.csv',['source_ns','receive_ns','mode','steering_angle','target_speed_mps','target_current','target_duty','target_brake_current'],commands)
    write('telemetry.csv',['source_ns','receive_ns','erpm','current_motor','avg_iq'],telemetry)
    write('timing.csv',['sequence','topic','type','source_ns','dds_pub_ns','dds_receive_ns','receive_ros_ns','receive_system_ns','receive_steady_ns','cdr_sha256'],[[i,'/synthetic/own','sensor_msgs/msg/Imu',int(n),int(n)+2000000,int(n)+2500000,int(n)+3000000,int(n)+3000000,i*5000000,'synthetic'] for i,n in enumerate(ns)])
    (folder/'status.jsonl').write_text('')
    metadata=dict(synthetic=True,source='synthetic integrated trajectory, injected own vx bias .05m/s and20ms waveform delay; ERPM response120ms',data_sha256={p.name:sha(p) for p in folder.iterdir() if p.is_file()})
    (folder/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n');(output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    return analyze(folder,config,output/'report')
