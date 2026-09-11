import openmdao.api as om
import numpy as np
import jax
import jax.numpy as jnp

from src.models.aerodynamics.AR import aspect_ratio as AR_calculation
from src.models.aerodynamics.lift import cl_required_cruise, cl_required_climb
from src.models.aerodynamics.ROC import gamma_from_roc
from src.models.aerodynamics.drag_coefficient import cd_calculation
from src.models.aerodynamics.lift_to_drag import lift_to_drag_ratio


class AerodynamicsComp(om.ExplicitComponent):
    """Compute aerodynamic coefficients from wing geometry.

    Inputs: b, c, MTOM, V_cruise, V_climb
    Outputs: `CL_cruise`, `CD_cruise`, `CL_climb`, `CD_climb`
    """

    def setup(self):
        self.add_input('b', val=10.0)
        self.add_input('c', val=1.5)
        self.add_input('MTOM', val=1500.0)
        self.add_input('V_cruise', val=50.0)
        self.add_input('V_climb', val=50.0)

        self.add_output('CL_cruise', val=0.0)
        self.add_output('CD_cruise', val=0.0)
        self.add_output('CL_climb', val=0.0)
        self.add_output('CD_climb', val=0.0)
        self.add_output('AR', val=0.0)
        self.add_output('LD_cruise', val=0.0)
        self.add_output('LD_climb', val=0.0)

        self.declare_partials('*', '*')

    def initialize(self):
        self.options.declare('parameters')

    def compute_partials(self, inputs, partials):        

        in_names = ['b', 'c', 'MTOM', 'V_cruise', 'V_climb']
        x = jnp.array([inputs[n][0] for n in in_names])

        def fun(x):
            b, c, MTOM, V_cruise, V_climb = x
            p = self.options['parameters']
            AR = AR_calculation(b, c)
            gamma_deg = gamma_from_roc(p.roc_climb_target, V_climb)
            CL_cruise = cl_required_cruise(MTOM, p.g, p.rho, V_cruise, b, c)
            CL_climb = cl_required_climb(MTOM, p.g, p.rho, V_climb, b, c, gamma_deg)
            CD_cruise = cd_calculation(CL_cruise, AR, p.c_d_min, p.e)
            CD_climb = cd_calculation(CL_climb, AR, p.c_d_min, p.e)
            LD_cruise = lift_to_drag_ratio(CL_cruise, CD_cruise)
            LD_climb = lift_to_drag_ratio(CL_climb, CD_climb)
            return jnp.array([CL_cruise, CD_cruise, CL_climb, CD_climb, AR, LD_cruise, LD_climb])

        J = jax.jacfwd(fun)(x)
        J = np.asarray(J)
        J = np.nan_to_num(J, nan=0.0, posinf=0.0, neginf=0.0)

        out_names = ['CL_cruise', 'CD_cruise', 'CL_climb', 'CD_climb', 'AR', 'LD_cruise', 'LD_climb']
        for i, out in enumerate(out_names):
            for j, inp in enumerate(in_names):
                partials[(out, inp)] = J[i, j]

    def compute(self, inputs, outputs):
        b = inputs['b'][0]
        c = inputs['c'][0]
        MTOM = inputs['MTOM'][0]
        V_cruise = inputs['V_cruise'][0]
        V_climb = inputs['V_climb'][0]

        AR = AR_calculation(b, c)
        p = self.options['parameters']
        gamma_deg = gamma_from_roc(p.roc_climb_target, V_climb)
        CL_cruise = cl_required_cruise(MTOM, p.g, p.rho, V_cruise, b, c)
        CL_climb = cl_required_climb(MTOM, p.g, p.rho, V_climb, b, c, gamma_deg)
        CD_cruise = cd_calculation(CL_cruise, AR, p.c_d_min, p.e)
        CD_climb = cd_calculation(CL_climb, AR, p.c_d_min, p.e)
        LD_cruise = lift_to_drag_ratio(CL_cruise, CD_cruise)
        LD_climb = lift_to_drag_ratio(CL_climb, CD_climb)

        outputs['CL_cruise'] = float(np.asarray(CL_cruise).item())
        outputs['CD_cruise'] = float(np.asarray(CD_cruise).item())
        outputs['CL_climb'] = float(np.asarray(CL_climb).item())
        outputs['CD_climb'] = float(np.asarray(CD_climb).item())
        outputs['AR'] = float(np.asarray(AR).item())
        outputs['LD_cruise'] = float(np.asarray(LD_cruise).item())
        outputs['LD_climb'] = float(np.asarray(LD_climb).item())
