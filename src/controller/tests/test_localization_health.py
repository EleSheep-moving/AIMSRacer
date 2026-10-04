import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from aims_mpcc.localization import LocalizationHealth


def status(stamp=1000000000,sequence=1,epoch='one',ready='true'):
    return dict(protocol_version='1',epoch=epoch,anchor_sequence=str(sequence),
                last_anchor_stamp_ns=str(stamp),ready=ready,state='tracking',health_sequence=str(sequence))


def test_source_age_and_monotonic_watchdog_cannot_be_refreshed_by_heartbeat():
    h=LocalizationHealth()
    h.observe(status(),1000000000,10.)
    assert h.usable(1400000000,10.2)
    h.observe(status(),1400000000,10.4)
    assert not h.usable(1400000000,10.51) # source ROS clock may freeze
    assert not h.usable(1510000000,10.42)


def test_health_heartbeat_expires_independently_of_anchor():
    h=LocalizationHealth();h.observe(status(),1000000000,10.)
    assert not h.usable(1200000000,10.31)


def test_epoch_change_and_retired_epoch_messages():
    h=LocalizationHealth();assert not h.observe(status(),1000000000,10.)
    assert h.observe(status(epoch='two',ready='false'),1100000000,10.1)
    assert not h.usable(1100000000,10.1)
    h.observe(status(stamp=1200000000,sequence=2,epoch='two'),1200000000,10.2)
    assert h.usable(1200000000,10.2)
    h.observe(status(stamp=1250000000,epoch='one',sequence=5),1250000000,10.25)
    assert h.epoch=='two'


def test_invalid_messages_future_and_reordered_anchor_fail_closed():
    h=LocalizationHealth();h.observe(status(),1000000000,10.)
    h.observe(status(stamp=2000000000,sequence=2),1100000000,10.1)
    assert not h.usable(1100000000,10.1)
    h.observe(status(stamp=1200000000,sequence=3),1200000000,10.2)
    h.observe(status(sequence=2),1210000000,10.21)
    assert h.sequence==3
    assert h.usable(1210000000,10.21)
    h.observe({'ready':'true'},1210000000,10.22)
    assert not h.usable(1210000000,10.22)


def test_reordered_health_cannot_restore_authorization_after_loss():
    h=LocalizationHealth()
    current=status(sequence=5);current['health_sequence']='8'
    h.observe(current,1000000000,10.)
    lost=dict(current,ready='false',health_sequence='9')
    h.observe(lost,1100000000,10.1)
    h.observe(current,1200000000,10.2)
    assert not h.usable(1200000000,10.2)
    counterfeit=dict(current,health_sequence='10')
    h.observe(counterfeit,1200000000,10.2)
    assert not h.usable(1200000000,10.2)
    recovered=status(stamp=1300000000,sequence=8)
    recovered['health_sequence']='11'
    h.observe(recovered,1300000000,10.3)
    assert h.usable(1300000000,10.3)
