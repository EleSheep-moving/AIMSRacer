import numpy as np
from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


def test_shared_serialization_freezes_exact_curve_and_speed():
    import aims_mpcc.reference_export as module
    angles=np.linspace(0,2*np.pi,12,endpoint=False)
    path=ReferencePath(4*np.c_[np.cos(angles),np.sin(angles)],.5,.5)
    config=VehicleConfig(geometry_verified=True,rear_offset=0.,half_width=.16,front_extent=.52,rear_extent=.1)
    result=module.serialize_reference(path,config)
    assert result['schema_version']==1
    assert result['knots']==path.curve.s.tolist()
    assert result['coefficients']==path.curve.coefficients.tolist()
    assert all(result['metadata'][key]==value for key,value in path.metadata.items())
    assert result['metadata']['coordinate_reference']=='rear_axle'
    assert result['speed_profile']['schema_version']==1
    assert len(result['speed_profile']['positions'])==len(result['speed_profile']['speeds'])


def test_new_recordings_label_the_actual_source_without_changing_legacy_loading(tmp_path):
    import csv
    import json
    from aims_mpcc.path import prepare_recording
    config=VehicleConfig(geometry_verified=True,rear_offset=0.,half_width=.16,front_extent=.52,rear_extent=.1)
    recording=tmp_path/'lap.csv'
    with recording.open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['timestamp','x','y','yaw','speed','frame_id','child_frame_id'])
        for i,t in enumerate(np.linspace(0,2*np.pi,81)):
            writer.writerow([i*.4,4*np.cos(t),4*np.sin(t),t+np.pi/2,.8,'odom','base_link'])
    result=prepare_recording(recording,tmp_path/'prepared',config,.5,.5)
    assert result.metadata['source']=='recorded'
    metadata=json.loads((tmp_path/'prepared'/'metadata.json').read_text());metadata.pop('source')
    (tmp_path/'prepared'/'metadata.json').write_text(json.dumps(metadata))
    assert ReferencePath.load(tmp_path/'prepared').validate_config(config,require_recording=True)


def test_serialization_is_identical_before_and_after_standard_path_save(tmp_path):
    from aims_mpcc.reference_export import serialize_reference
    angles=np.linspace(0,2*np.pi,12,endpoint=False)
    path=ReferencePath(4*np.c_[np.cos(angles),np.sin(angles)],.5,.5)
    config=VehicleConfig(geometry_verified=True,rear_offset=0.,half_width=.16,front_extent=.52,rear_extent=.1)
    path.save(tmp_path/'saved')
    assert serialize_reference(path,config)==serialize_reference(ReferencePath.load(tmp_path/'saved'),config)
