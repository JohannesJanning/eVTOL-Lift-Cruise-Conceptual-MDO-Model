import openmdao.api as om

from src.optimizer.components.aerodynamics_comp import AerodynamicsComp
from src.optimizer.components.energy_comp import EnergyComp
from src.optimizer.components.geometry_comp import GeometryComp
from src.optimizer.components.mass_comp import MassComp
from src.optimizer.components.mtom_implicit import JaxMTOMImplicit
from src.optimizer.components.noise_comp import NoiseComp
from src.optimizer.components.performance_comp import PerformanceComp
from src.optimizer.components.ops_comp import OpsComp
from src.optimizer.components.gwp_comp import GWPComp
from src.optimizer.components.economic_comp import EconomicComp

class eVTOLGroup(om.Group):
    """eVTOL group."""

    def initialize(self):
        self.options.declare('parameters')

    def setup(self):
        params = self.options['parameters']

        self.add_subsystem('aero', AerodynamicsComp(parameters=params), promotes=['*'])
        self.add_subsystem('mtom', JaxMTOMImplicit(parameters=params))
        self.add_subsystem('geom', GeometryComp(parameters=params), promotes=['*'])
        self.add_subsystem('perf', PerformanceComp(parameters=params), promotes=['*'])
        self.add_subsystem('energy', EnergyComp(parameters=params), promotes=['*'])
        self.add_subsystem('mass', MassComp(parameters=params), promotes=['*'])
        self.add_subsystem('ops', OpsComp(parameters=params), promotes=['*'])
        self.add_subsystem('gwp', GWPComp(parameters=params), promotes=['*'])
        self.add_subsystem('economic', EconomicComp(parameters=params), promotes=['*'])
        self.add_subsystem('noise', NoiseComp(parameters=params), promotes=['*'])

        self.connect('MTOM_est', 'mtom.MTOM_est')
        self.connect('mtom.MTOM', 'MTOM')

        self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)
        self.nonlinear_solver.options['maxiter'] = 50
        self.nonlinear_solver.options['rtol'] = 1e-6
        # keep Newton steps within variable bounds (e.g. MTOM > 0) to avoid NaN propagation
        self.nonlinear_solver.linesearch = om.BoundsEnforceLS(bound_enforcement='vector')
        self.linear_solver = om.DirectSolver(assemble_jac=True)
