import pytest
from aims_calibration import planner
from test_planner import a_payload


def test_full_B_defaults_run_one_to_five_and_repeat_every_independent_trial():
    baseline=planner.export_baseline(a_payload())
    result=planner.build_full_step_plan('accel_interval',baseline,repeats=2)
    assert all((p.start_speed,p.end_speed)==(1.,5.) for p in result)
    assert [p.current for p in result[::2]]==pytest.approx([.104+5*k for k in range(1,20)]+[100.])
    assert [p.repeat for p in result[:4]]==[1,2,1,2]
    assert result[0] is not result[1]


def test_full_C_defaults_share_five_to_one_boundary_for_every_current():
    result=planner.build_full_step_plan('decel_current',planner.export_baseline(a_payload()))
    assert [p.current for p in result[:9]]==pytest.approx([40.104-5*k for k in range(1,9)]+[0.])
    assert all((p.start_speed,p.end_speed)==(5.,1.) for p in result)
    assert result[-1].current==-100.
    assert {p.kind for p in result}=={'positive_deceleration','coast','braking'}


def test_full_plan_selects_polynomial_and_includes_nonmultiple_brake_cap():
    baseline=planner.with_polynomial_baseline(planner.export_baseline(a_payload()),[7.,1.,0.,0.])
    b=planner.build_full_step_plan('accel_interval',baseline,start_speed=1.25,end_speed=4.75,max_current=23.)
    assert [p.current for p in b]==pytest.approx([13.25,18.25,23.])
    c=planner.build_full_step_plan('decel_current',baseline,start_speed=4.75,end_speed=1.25,max_current=23.)
    assert [p.current for p in c]==pytest.approx([6.75,1.75,0.,-5.,-10.,-15.,-20.,-23.])


@pytest.mark.parametrize('kwargs',[
    dict(stage='speed_hold'),dict(start_speed=5.),dict(end_speed=1.),
    dict(start_speed=.9),dict(end_speed=5.1),dict(start_speed=1.25),
    dict(current_step=0.),dict(max_current=101.),dict(repeats=True),
    dict(start_speed=float('nan')),dict(current_step=True),dict(max_current=4.),
])
def test_full_plan_refuses_invalid_grid_direction_bounds_and_uncovered_speed(kwargs):
    args=dict(stage='accel_interval',baseline=planner.export_baseline(a_payload()))
    args.update(kwargs)
    with pytest.raises(ValueError):planner.build_full_step_plan(**args)


def test_full_plan_validates_endpoint_coverage_and_baseline_hash():
    baseline=planner.export_baseline(a_payload())
    with pytest.raises(ValueError):planner.build_full_step_plan('decel_current',baseline,end_speed=1.25)
    baseline['points'][0]['mean_iq']=12.
    with pytest.raises(ValueError):planner.build_full_step_plan('accel_interval',baseline)
