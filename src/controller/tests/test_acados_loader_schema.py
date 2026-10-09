from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import numpy as np
import pytest
from aims_mpcc.acados_backend import AcadosSolver
from aims_mpcc.config import VehicleConfig

def fixture():
    solver=AcadosSolver.__new__(AcadosSolver);solver.config=VehicleConfig(enforce_corridor=False)
    solver.dt=.1;solver.n=10;solver.model_name='loader_schema';solver._expected_lib_path='/sdk/lib'
    solver.shared_library=Path('/cache/generated/libacados_ocp_solver_loader_schema.so')
    routes=dict(shared_lib_ext='.so',code_export_directory='/cache/generated',
                acados_lib_path='/sdk/lib',acados_include_path='/sdk/include')
    doc=dict(name=solver.model_name,model=dict(name=solver.model_name),problem_class='OCP',
      code_gen_options=routes,dims=dict(N=10,nx=9,np=10,nu=3,nh_0=9,nh=9,nh_e=1,nbx_0=9,nbxe_0=9),
      constraints=dict(has_x0=True),solver_options=dict(N_horizon=10,nlp_solver_type='SQP_RTI',
      qp_solver='PARTIAL_CONDENSING_HPIPM',integrator_type='DISCRETE',tf=1.,time_steps=[.1]*10))
    return solver,doc

def test_pinned_sdk_nested_loader_routes_are_validated():
    solver,doc=fixture()
    assert solver._loader_metadata_matches(doc)

@pytest.mark.parametrize('key,value',[('acados_lib_path','/different/lib'),('shared_lib_ext','.dll'),
                                    ('code_export_directory','/different/cache')])
def test_nested_loader_route_tampering_is_rejected(key,value):
    solver,doc=fixture();doc['code_gen_options'][key]=value
    assert not solver._loader_metadata_matches(doc)

def test_conflicting_old_and_new_route_metadata_is_rejected():
    solver,doc=fixture();doc.update(deepcopy(doc['code_gen_options']));doc['acados_lib_path']='/wrong/lib'
    assert not solver._loader_metadata_matches(doc)


def test_verified_runtime_load_cannot_reenable_sdk_generation(monkeypatch, tmp_path):
    """The pinned SDK may override generate/build=False after its reuse check."""
    sdk = pytest.importorskip('acados_template')
    from aims_mpcc.acados_backend import _dependency_versions
    from aims_mpcc.path import ReferencePath
    assert _dependency_versions()['acados_source'] == '59d93e17d2985fdd73fc58b8a83ed8f83a024171'
    angles = np.linspace(0., 2*np.pi, 40, endpoint=False)
    path = ReferencePath(2*np.c_[np.cos(angles), np.sin(angles)], 1., 1., 'odom')
    config = VehicleConfig(profile='synthetic', rear_offset=.15, half_length=.28,
                           half_width=.15, geometry_verified=True, enforce_corridor=False)
    cache = Path(os.environ.get('AIMS_ACADOS_LOADER_TEST_CACHE', str(tmp_path / 'cache')))
    prepared = AcadosSolver(path, config, horizon=3, prepare=True, artifact_directory=cache)
    artifact_files = [prepared._manifest, prepared.json_file, prepared.shared_library]
    before = {file: hashlib.sha256(file.read_bytes()).hexdigest() for file in artifact_files}
    manifest = json.loads(prepared._manifest.read_text())
    assert manifest['shared_library_sha256'] == before[prepared.shared_library]
    assert manifest['ocp_json_sha256'] == before[prepared.json_file]
    verified = []
    own_check = AcadosSolver._artifact_ready

    def record_verification(self, payload):
        accepted = own_check(self, payload)
        verified.append(accepted)
        return accepted

    def forbidden(*args, **kwargs):
        raise AssertionError('runtime load attempted SDK generation or build')

    monkeypatch.setattr(AcadosSolver, '_artifact_ready', record_verification)
    monkeypatch.setattr(sdk.AcadosOcpSolver, 'is_code_reuse_possible', lambda *args, **kwargs: False)
    monkeypatch.setattr(sdk.AcadosOcpSolver, 'generate', staticmethod(forbidden))
    monkeypatch.setattr(sdk.AcadosOcpSolver, 'build', staticmethod(forbidden))
    loaded = AcadosSolver(path, config, horizon=3, prepare=False, artifact_directory=cache)
    assert verified and all(verified)
    assert loaded._verified_json_bytes == prepared._verified_json_bytes
    assert Path(loaded._native.shared_lib_name).resolve() == prepared.shared_library.resolve()
    assert {file: hashlib.sha256(file.read_bytes()).hexdigest() for file in artifact_files} == before
