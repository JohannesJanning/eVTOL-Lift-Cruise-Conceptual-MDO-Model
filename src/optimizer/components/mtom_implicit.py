import openmdao.api as om
import jax
import jax.numpy as jnp
import numpy as np


class JaxMTOMImplicit(om.JaxImplicitComponent):
    """Implicit MTOM component using a simple JAX residual:

    Residual R = MTOM - MTOM_est

    - `MTOM` is the implicit state (output)
    - `MTOM_est` is an input (computed by `MassComp`)

    The residual is intentionally simple so that AD through the coupled
    system is stable: the detailed mass calculation stays in `MassComp` and
    its JAX partials are used by OpenMDAO when computing total derivatives.
    """

    def initialize(self):
        self.options.declare('parameters', default=None)

    def setup(self):
        self.add_input('MTOM_est', val=1500.0)
        self.add_output('MTOM', val=1500.0, lower=100.0)
        self.declare_partials(of='MTOM', wrt=['MTOM', 'MTOM_est'])

    def setup_partials(self):
        self.nonlinear_solver = om.NewtonSolver(solve_subsystems=False)
        self.linear_solver = om.DirectSolver()

    @staticmethod
    @jax.jit
    def _residual(MTOM, MTOM_est):
        # R = MTOM - MTOM_est
        r = MTOM - MTOM_est
        return jnp.nan_to_num(r, nan=0.0, posinf=1e9, neginf=-1e9)

    def compute_primal(self, MTOM_est, MTOM):
        return self._residual(MTOM, MTOM_est)

    def compute_residual(self, inputs, outputs, residuals):
        MTOM = jnp.array(outputs['MTOM'])
        MTOM_est = jnp.array(inputs['MTOM_est'])
        residuals['MTOM'] = float(self._residual(MTOM, MTOM_est))

    def guess_nonlinear(self, inputs, outputs, residuals):
        mtom_est = float(np.asarray(inputs['MTOM_est'])[0])
        if not np.isfinite(mtom_est) or mtom_est < 100.0:
            mtom_est = 1500.0
        current = float(np.asarray(outputs['MTOM'])[0])
        if not np.isfinite(current) or current < 100.0:
            outputs['MTOM'] = mtom_est

    def linearize(self, inputs, outputs, partials):
            eps = 1e-8
            partials[('MTOM', 'MTOM')] = np.array([[1.0 + eps]], dtype=float)
            partials[('MTOM', 'MTOM_est')] = np.array([[-1.0]], dtype=float)
