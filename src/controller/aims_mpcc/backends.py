"""Explicit backend selection. IPOPT remains the established default."""
BACKENDS=('ipopt','acados','qp')


def create_solver(backend,path,config,horizon=10,dt=.1,prepare=False,
                  artifact_directory=None,jit_enabled=False,native_options=None):
    if backend=='ipopt':
        from .solver import MPCCSolver
        return MPCCSolver(path,config,horizon,dt,jit_enabled,native_options)
    if backend=='acados':
        from .acados_backend import AcadosSolver
        return AcadosSolver(path,config,horizon,dt,prepare=prepare,artifact_directory=artifact_directory)
    if backend=='qp':
        from .qp_backend import QPSolver
        return QPSolver(path,config,horizon,dt)
    raise ValueError(f'unknown solver backend {backend!r}; expected one of {BACKENDS}')
