import os
import tempfile
import time
import numpy as np
import openmdao.api as om

from src.parameters import model_parameters as parameters
from src.optimizer.eVTOL_group import eVTOLGroup
os.environ.setdefault('OPENMDAO_WORKDIR', tempfile.mkdtemp(prefix='openmdao_'))

prob = om.Problem(model=eVTOLGroup(parameters=parameters), reports=False)

# Design variables 
iv = om.IndepVarComp()
iv.add_output('b', val=15.0)
iv.add_output('c', val=1.0)
iv.add_output('r_cruise', val=1.0)
iv.add_output('r_hover', val=1.0)
iv.add_output('V_cruise', val=60.0)
iv.add_output('V_climb', val=60.0)
iv.add_output('rho_bat', val=parameters.rho_bat)
iv.add_output('c_charge', val=2.0)
prob.model.add_subsystem('iv', iv, promotes=['*'])

# Design variables bounds 
prob.model.add_design_var('b', lower=6.0, upper=15.0, ref0=6.0, ref=15.0)
prob.model.add_design_var('c', lower=1.0, upper=2.5, ref0=1.0, ref=2.5)
prob.model.add_design_var('r_cruise', lower=0.7, upper=1.2, ref0=0.6, ref=1.2)
prob.model.add_design_var('r_hover', lower=0.6, upper=1.9, ref0=0.6, ref=1.3)
prob.model.add_design_var('V_cruise', lower=40.0, upper=129.0, ref0=40.0, ref=129.0) 
prob.model.add_design_var('V_climb', lower=40.0, upper=129.0, ref0=40.0, ref=129.0)
prob.model.add_design_var('c_charge', lower=1.0, upper=4.0, ref0=1.0, ref=4.0)

# Constraints 
cons_comp = om.ExecComp('c1 = b - rotor_spacing', b=15.0, rotor_spacing=1.0)
prob.model.add_subsystem('cons_comp', cons_comp)
prob.model.connect('rotor_spacing', 'cons_comp.rotor_spacing')
prob.model.connect('b', 'cons_comp.b')

prob.model.add_constraint('cons_comp.c1', lower=0.0, ref=1.0)
prob.model.add_constraint('vertiport_span', upper=15.0, ref=1.0)
prob.model.add_constraint('MTOM', upper=3750.0, ref=2000.0)
prob.model.add_constraint('CL_cruise', lower=0, upper=0.7, ref=1.0)
prob.model.add_constraint('CL_climb', lower=0.0, upper=1.2, ref=1.0)
prob.model.add_constraint('gamma_deg', lower=5.0, upper=15.0, ref=10.0)
prob.model.add_constraint('SPL_hover', upper=77.0, ref=100.0)
prob.model.add_constraint('AR', lower=6.0, upper=10.0, ref=8.0)

# Optimization objective 
prob.model.add_objective('MTOM_est', ref=2000.0)

prob.driver = om.ScipyOptimizeDriver()
prob.driver.options['optimizer'] = 'SLSQP'
prob.driver.options['tol'] = 1e-6
prob.driver.options['disp'] = True
if hasattr(prob.driver, 'declare_coloring'):
    prob.driver.declare_coloring()

# intial design variables values 
x0 = np.array([9.0, 1.0, 1.0, 1.0, 60.0, 60.0, 1.0], dtype=float)

prob.setup()
prob.set_val('b', x0[0])
prob.set_val('c', x0[1])
prob.set_val('r_cruise', x0[2])
prob.set_val('r_hover', x0[3])
prob.set_val('V_cruise', x0[4])
prob.set_val('V_climb', x0[5])
prob.set_val('rho_bat', parameters.rho_bat)
prob.set_val('c_charge', x0[6])


def value(name):
    """Return a scalar OpenMDAO value for display, falling back to model parameters."""
    try:
        return float(np.asarray(prob.get_val(name)).item())
    except Exception:
        pass
    param_value = getattr(parameters, name, None)
    return float(param_value) if param_value is not None else None


def print_section(title, names, unit=None):
    """Print a table of variables. Entries can be a name string, or a
    (name, label, unit, scale[, decimals[, pct_name]]) tuple to override the
    display label/unit/scale/decimal precision (decimals defaults to 2). If
    pct_name is given, it names a fraction output (0-1) that is displayed
    alongside the value as a percentage."""
    def _normalize(n):
        if not isinstance(n, tuple):
            return (n, n, unit, 1.0, 2, None)
        defaults = (n[0], n[0], unit, 1.0, 2, None)
        return n + defaults[len(n):]

    entries = [_normalize(n) for n in names]
    print('-------------------------------')
    print(title)
    label_width = max(len(label) for _, label, _, _, _, _ in entries)
    unit_width = max((len(entry_unit) for _, _, entry_unit, _, _, _ in entries if entry_unit), default=0)
    for name, label, entry_unit, scale, decimals, pct_name in entries:
        result = value(name)
        label = label.ljust(label_width)
        pct_text = ''
        if pct_name is not None:
            pct_value = value(pct_name)
            if pct_value is not None:
                pct_text = f'  ({100 * pct_value:>5.1f}% of total)'
        if result is None:
            print(f'{label} : None')
        elif name.endswith('_fraction'):
            print(f'{label} ({"%".ljust(unit_width)}) = {100 * result:>10.2f}')
        elif entry_unit:
            print(f'{label} ({entry_unit.ljust(unit_width)}) = {result * scale:>10.{decimals}f}{pct_text}')
        else:
            print(f'{label} : {result}')


def print_constraints(constraints):
    """Print constraint value/bound/residual/status. residual >= 0 means the
    constraint is satisfied; ACTIVE means it's satisfied but sitting right
    at (within ~0.5% of) a bound."""
    print('-------------------------------')
    print('CONSTRAINT RESIDUALS')
    label_width = max(len(c['label']) for c in constraints)
    for c in constraints:
        val = c['compute']() if 'compute' in c else value(c['name'])
        if val is None:
            print(f'{c["label"].ljust(label_width)} : None')
            continue
        kind = c['kind']
        if kind == 'lower':
            bound = c['bound']
            residual = val - bound
            bound_txt = f'>= {bound:.3f}'
            tol = max(1e-6, 0.005 * abs(bound))
        elif kind == 'upper':
            bound = c['bound']
            residual = bound - val
            bound_txt = f'<= {bound:.3f}'
            tol = max(1e-6, 0.005 * abs(bound))
        else:  # range
            lo, hi = c['bound']
            residual = min(val - lo, hi - val)
            bound_txt = f'[{lo:.3f}, {hi:.3f}]'
            tol = max(1e-6, 0.005 * abs(hi - lo))
        if residual < -0.0005:
            status = 'VIOLATED'
        elif residual <= tol:
            status = 'ACTIVE'
        else:
            status = 'OK'
        label = c['label'].ljust(label_width)
        print(f'{label} = {val:>9.3f}  bound {bound_txt:<16} resid {residual:>9.3f}  {status}')


# runtime measure 
t0 = time.time()
try:
    prob.run_driver()
except Exception as exc:
    print('Optimization failed:', exc)
    raise
t1 = time.time()

print(f'Elapsed (s): {t1 - t0:.1f}')
print('-------------------------------')
print('RUN SUMMARY')
print('Objective MTOM_est:', value('MTOM_est'))

print_section('DESIGN VARIABLES (OPTIMIZED)', [
    ('b', 'b', 'm', 1.0),
    ('c', 'c', 'm', 1.0),
    ('r_cruise', 'r_cruise', 'm', 1.0),
    ('r_hover', 'r_hover', 'm', 1.0),
    ('V_cruise', 'V_cruise', 'm/s', 1.0),
    ('V_climb', 'V_climb', 'm/s', 1.0),
    ('rho_bat', 'rho_bat', 'Wh/kg', 1.0),
    ('c_charge', 'c_charge', '1/h', 1.0),
])

print_section('CONSTRAINT-RELATED OUTPUTS', [
    ('vertiport_span', 'vertiport_span', 'm', 1.0),
    ('SPL_hover', 'SPL_hover', 'dB', 1.0),
    ('RPM_hover', 'RPM_hover', 'rpm', 1.0),
    ('gamma_deg', 'gamma_deg', 'deg', 1.0),
])

print_constraints([
    {
        'label': 'b - rotor_spacing',
        'kind': 'lower',
        'bound': 0.0,
        'compute': lambda: value('b') - value('rotor_spacing'),
    },
    {'name': 'vertiport_span', 'label': 'vertiport_span', 'kind': 'upper', 'bound': 15.0},
    {'name': 'MTOM', 'label': 'MTOM', 'kind': 'upper', 'bound': 3750.0},
    {'name': 'CL_cruise', 'label': 'CL_cruise', 'kind': 'range', 'bound': (0.0, 0.7)},
    {'name': 'CL_climb', 'label': 'CL_climb', 'kind': 'range', 'bound': (0.0, 1.2)},
    {'name': 'gamma_deg', 'label': 'gamma_deg', 'kind': 'range', 'bound': (5.0, 15.0)},
    {'name': 'SPL_hover', 'label': 'SPL_hover', 'kind': 'upper', 'bound': 77.0},
    {'name': 'AR', 'label': 'AR', 'kind': 'range', 'bound': (6.0, 10.0)},
])

print_section('MASS', [
    'MTOM',
    'MTOM_est',
    ('m_pay', 'm_payload', 'kg', 1.0),
    'm_battery',
    'm_empty',
    'm_empty_fraction',
    'm_battery_fraction',
    'm_payload_fraction',
    'm_motor',
    'm_system',
    'm_rotor',
    'm_fuselage',
    'm_gear',
    'm_interior',
    'm_wing',
    'm_crew',
], unit='kg')

print_section('AERODYNAMICS', [
    'CL_cruise',
    'CD_cruise',
    'CL_climb',
    'CD_climb',
    'LD_cruise',
    'LD_climb',
    'AR',
], unit='-')

print_section('PERFORMANCE', [
    ('P_req_total_hover', 'P_req_hover', 'kW', 1e-3),
    ('P_req_total_climb', 'P_req_climb', 'kW', 1e-3),
    ('P_req_total_cruise', 'P_req_cruise', 'kW', 1e-3),
    ('wing_loading_kg_m2', 'wing_loading', 'kg/m^2', 1.0),
    ('wing_loading_n_m2', 'wing_loading', 'N/m^2', 1.0),
    ('wing_loading_lb_ft2', 'wing_loading', 'lb/ft^2', 1.0),
    ('disk_loading_hover_kg_m2', 'disk_loading_hover', 'kg/m^2', 1.0),
    ('disk_loading_hover_n_m2', 'disk_loading_hover', 'N/m^2', 1.0),
    ('disk_loading_hover_lb_ft2', 'disk_loading_hover', 'lb/ft^2', 1.0),
    ('roc_climb_target_ft_min', 'ROC', 'ft/min', 1.0),
])

print_section('ENERGY', [
    ('E_total_req', 'E_total_req', 'kWh', 1e-3),
    ('E_trip', 'E_trip', 'kWh', 1e-3),
    ('E_hover', 'E_hover', 'kWh', 1e-3),
    ('E_climb', 'E_climb', 'kWh', 1e-3),
    ('E_cruise', 'E_cruise', 'kWh', 1e-3),
    ('E_reserve', 'E_reserve', 'kWh', 1e-3),
])

print_section('BATTERY', [
    ('C_rate_hover', 'C_rate_hover', '-', 1.0),
    ('C_rate_climb', 'C_rate_climb', '-', 1.0),
    ('C_rate_cruise', 'C_rate_cruise', '-', 1.0),
    ('C_rate_avg', 'C_rate_avg', '-', 1.0),
    ('DOD', 'DOD', '-', 1.0),
    ('n_battery_lifecycle', 'n_battery_lifecycle', '-', 1.0),
    ('E_batt_design_Wh', 'E_batt_design', 'kWh', 1e-3),
])

print_section('OPERATIONS / UTILIZATION', [
    ('FC_d', 'FC_d', '1/day', 1.0, 0),
    ('FC_a', 'FC_a', '1/yr', 1.0, 0),
    ('annual_flight_hours', 'annual_flight_hours', 'h/yr', 1.0, 0),
    ('time_turn', 'time_turn', 'min', 1 / 60.0, 2),
    ('t_climb', 't_climb', 'min', 1 / 60.0, 2),
    ('t_cruise', 't_cruise', 'min', 1 / 60.0, 2),
    ('t_trip', 't_trip', 'min', 1 / 60.0, 2),
])

print_section('ECONOMICS (PER FLIGHT)', [
    ('TOC_flight', 'TOC (DOC+IOC)', 'EUR', 1.0),
    ('DOC_flight', 'DOC (ops)', 'EUR', 1.0, 2, 'DOC_frac_TOC'),
    ('IOC_value_flight', 'IOC (admin)', 'EUR', 1.0, 2, 'IOC_frac_TOC'),
    ('COC_flight', 'COC', 'EUR', 1.0, 2, 'COC_frac_TOC'),
    ('COO_value_flight', 'COO', 'EUR', 1.0, 2, 'COO_frac_TOC'),
    ('C_maintenance_flight', 'C_maintenance', 'EUR', 1.0, 2, 'C_maintenance_frac_TOC'),
    ('batt_maint_flight', 'batt_maint', 'EUR', 1.0, 2, 'batt_maint_frac_TOC'),
    ('wrap_maint_flight', 'wrap_maint', 'EUR', 1.0, 2, 'wrap_maint_frac_TOC'),
    ('C_crew_flight', 'c_crew', 'EUR', 1.0, 2, 'C_crew_frac_TOC'),
    ('C_energy_flight', 'c_energy', 'EUR', 1.0, 2, 'C_energy_frac_TOC'),
    ('C_navigation_flight', 'c_navigation', 'EUR', 1.0, 2, 'C_navigation_frac_TOC'),
])

print_section('ECONOMICS (PER FLIGHT HOUR)', [
    ('TOC_per_hr', 'TOC', 'EUR/h', 1.0),
    ('DOC_per_hr', 'DOC', 'EUR/h', 1.0, 2, 'DOC_frac_TOC'),
    ('IOC_per_hr', 'IOC', 'EUR/h', 1.0, 2, 'IOC_frac_TOC'),
    ('COC_per_hr', 'COC', 'EUR/h', 1.0, 2, 'COC_frac_TOC'),
    ('COO_per_hr', 'COO', 'EUR/h', 1.0, 2, 'COO_frac_TOC'),
    ('C_maintenance_per_hr', 'C_maintenance', 'EUR/h', 1.0, 2, 'C_maintenance_frac_TOC'),
    ('batt_maint_per_hr', 'batt_maint', 'EUR/h', 1.0, 2, 'batt_maint_frac_TOC'),
    ('wrap_maint_per_hr', 'wrap_maint', 'EUR/h', 1.0, 2, 'wrap_maint_frac_TOC'),
    ('C_crew_per_hr', 'c_crew', 'EUR/h', 1.0, 2, 'C_crew_frac_TOC'),
    ('C_energy_per_hr', 'c_energy', 'EUR/h', 1.0, 2, 'C_energy_frac_TOC'),
    ('C_navigation_per_hr', 'c_navigation', 'EUR/h', 1.0, 2, 'C_navigation_frac_TOC'),
])

print_section('BUSINESS CASE', [
    ('Revenue_Flight', 'Revenue', 'EUR', 1.0),
    ('Revenue_per_hr', 'Revenue', 'EUR/h', 1.0),
    ('Revenue_Annual', 'Revenue', 'EUR/yr', 1.0),
    ('Profit_Flight', 'Profit', 'EUR', 1.0),
    ('Profit_per_hr', 'Profit', 'EUR/h', 1.0),
    ('Annual_Profit', 'Profit', 'EUR/yr', 1.0),
    ('Ticket_Price', 'Ticket_Price', 'EUR', 1.0),
])

print_section('ENVIRONMENTAL (GWP)', [
    ('GWP_flight', 'GWP', 'kgCO2e', 1.0),
    ('GWP_per_hr', 'GWP', 'kgCO2e/h', 1.0),
    ('GWP_annual_ops', 'GWP', 'kgCO2e/yr', 1.0),
    ('GWP_energy_flight', 'GWP_energy', 'kgCO2e', 1.0, 2, 'GWP_energy_frac'),
    ('GWP_energy_per_hr', 'GWP_energy', 'kgCO2e/h', 1.0, 2, 'GWP_energy_frac'),
    ('GWP_energy_annual', 'GWP_energy', 'kgCO2e/yr', 1.0, 2, 'GWP_energy_frac'),
    ('GWP_battery_flight', 'GWP_battery', 'kgCO2e', 1.0, 2, 'GWP_battery_frac'),
    ('GWP_battery_per_hr', 'GWP_battery', 'kgCO2e/h', 1.0, 2, 'GWP_battery_frac'),
    ('GWP_battery_annual', 'GWP_battery', 'kgCO2e/yr', 1.0, 2, 'GWP_battery_frac'),
])

