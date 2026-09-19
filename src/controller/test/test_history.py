import math
import pytest
from aims_mpcc.history import AppliedHistory


def test_old_measurement_uses_old_command_and_steering_estimate():
    history=AppliedHistory(.1)
    history.record(1.,0.,0.,0.,0.)
    history.record(1.05,.2,.1,.5,.4)
    estimate,previous=history.at(1.04)
    assert estimate==0. and previous['steering']==0.
    estimate,previous=history.at(1.1)
    assert estimate==pytest.approx(.2*(1-math.exp(-.05/.1)))
    assert previous['steering']==.2 and previous['steering_rate']==.4
    assert history.at(.9)[1]['steering']==0.


def test_reject_history_reordering():
    history=AppliedHistory(.1);history.record(1.,0.,0.,0.,0.)
    with pytest.raises(ValueError):history.record(.9,0.,0.,0.,0.)
