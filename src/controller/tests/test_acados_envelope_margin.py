"""The experimental RTI reserve is explicit and dimensionless."""
import pytest
from aims_mpcc.config import VehicleConfig


@pytest.mark.parametrize('margin',[-.01,1.,float('nan'),float('inf'),True])
def test_invalid_acados_envelope_reserve_rejected(margin):
    with pytest.raises(ValueError,match='acados_envelope_margin'):
        VehicleConfig(acados_envelope_margin=margin).validate()


@pytest.mark.parametrize('margin',[0.,.01,.999])
def test_acados_reserve_allows_zero_and_strictly_less_than_one(margin):
    cfg=VehicleConfig(acados_envelope_margin=margin).validate()
    assert cfg.acados_envelope_margin==margin


@pytest.mark.parametrize('margin',[-.01,1.,float('nan'),float('inf'),True])
def test_invalid_common_optimization_reserve_rejected(margin):
    with pytest.raises(ValueError,match='optimization_envelope_margin'):
        VehicleConfig(optimization_envelope_margin=margin).validate()


@pytest.mark.parametrize('margin',[0.,.01,.999])
def test_common_reserve_preserves_zero_default_and_valid_bounds(margin):
    assert VehicleConfig().optimization_envelope_margin==0.
    assert VehicleConfig(optimization_envelope_margin=margin).validate().optimization_envelope_margin==margin
