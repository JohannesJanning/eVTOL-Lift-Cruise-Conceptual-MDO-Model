import openmdao.api as om
import numpy as np
import jax
import jax.numpy as jnp

from src.models.gwp.gwp_model import (
    battery_lifecycle_gwp,
    battery_annual_ops_gwp,
    battery_flight_cycle_gwp,
    gwp_operational_per_cycle,
    gwp_flight,
    gwp_annual,
    gwp_energy_fraction,
    gwp_battery_fraction,
)

from src.models.battery.E_battery_design import battery_energy_capacity
from src.models.battery.c_rate import c_rate
from src.models.battery.c_rate_trip import c_rate_average
from src.models.battery.n_cycles_design import battery_cycle_life
from src.models.battery.n_annual_battery_mission import number_of_battery_required_annually
from src.models.operations.ops_model import turnaround_time, time_efficiency_ratio
from src.models.battery.DOD import depth_of_discharge


class GWPComp(om.ExplicitComponent):
    """Compute GWP contributions from energy and battery for operations.

    Inputs: E_trip, m_battery
    Outputs: GWP_flight, GWP_annual_ops
    """

    def initialize(self):
        self.options.declare('parameters')

    def setup(self):
        self.add_input('E_trip', val=0.0)
        self.add_input('m_battery', val=0.0)
        self.add_input('rho_bat', val=300.0)
        self.add_input('FC_a', val=260.0)
        self.add_input('t_trip', val=3600.0)
        self.add_input('t_climb', val=0.0)
        self.add_input('P_req_total_hover', val=0.0)
        self.add_input('P_req_total_climb', val=0.0)
        self.add_input('P_req_total_cruise', val=0.0)
        # battery charging parameters
        self.add_input('c_charge', val=1.0)

        self.add_output('GWP_flight', val=0.0)
        self.add_output('GWP_annual_ops', val=0.0)
        self.add_output('DOD', val=0.3)
        self.add_output('C_rate_hover', val=0.0)
        self.add_output('C_rate_climb', val=0.0)
        self.add_output('C_rate_cruise', val=0.0)
        self.add_output('C_rate_avg', val=0.0)
        self.add_output('n_battery_lifecycle', val=1000.0)
        self.add_output('n_batt_annual', val=0.0)

        # energy vs. battery GWP breakdown, per flight / per hour / annual
        self.add_output('GWP_energy_flight', val=0.0)
        self.add_output('GWP_battery_flight', val=0.0)
        self.add_output('GWP_per_hr', val=0.0)
        self.add_output('GWP_energy_per_hr', val=0.0)
        self.add_output('GWP_battery_per_hr', val=0.0)
        self.add_output('GWP_energy_annual', val=0.0)
        self.add_output('GWP_battery_annual', val=0.0)
        self.add_output('GWP_energy_frac', val=0.0)
        self.add_output('GWP_battery_frac', val=0.0)

        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        p = self.options['parameters']
        E_trip = inputs['E_trip'][0]
        m_batt = inputs['m_battery'][0]
        rho_bat = inputs['rho_bat'][0]
        FC_a = inputs['FC_a'][0]
        t_trip = inputs['t_trip'][0]
        P_hover = inputs['P_req_total_hover'][0]
        P_climb = inputs['P_req_total_climb'][0]
        P_cruise = inputs['P_req_total_cruise'][0]
        c_charge = inputs['c_charge'][0]

        # estimate battery design energy (Wh) and guard against zero
        eps = 1e-6
        E_batt_design_Wh = float(np.asarray(battery_energy_capacity(rho_bat, m_batt)))
        E_batt_design_Wh = max(E_batt_design_Wh, eps)

        # compute C-rates for mission segments and average trip C-rate
        C_hover = float(np.asarray(c_rate(P_hover, E_batt_design_Wh)))
        C_climb = float(np.asarray(c_rate(P_climb, E_batt_design_Wh)))
        C_cruise = float(np.asarray(c_rate(P_cruise, E_batt_design_Wh)))

        t_hover = float(p.time_hover)
        t_climb = float(inputs['t_climb'][0])
        eps = 1e-3
        t_cruise = float(max(t_trip - t_hover - t_climb, eps))

        C_avg = float(np.asarray(c_rate_average(C_hover, C_climb, C_cruise, t_hover, t_climb, t_cruise, t_trip)))
        C_avg = max(C_avg, eps)

        # depth of discharge and cycle life
        DOD = float(np.asarray(depth_of_discharge(E_trip, E_batt_design_Wh)))
        DOD = max(min(DOD, 1.0), eps)

        N_cycles_available = float(np.asarray(battery_cycle_life(DOD, C_avg, c_charge)))
        N_cycles_available = max(N_cycles_available, 1.0)

        # number of battery packs required annually
        time_turn = float(np.asarray(turnaround_time(c_charge, DOD)))
        DH = float(np.asarray(time_efficiency_ratio(time_turn, max(t_trip, eps))))
        n_batt_annual = float(np.asarray(number_of_battery_required_annually(N_cycles_available, p.N_wd, p.T_D, max(t_trip, eps), DH)))

        battery_LC_GWP = float(np.asarray(battery_lifecycle_gwp(p.GWP_battery, E_batt_design_Wh)))
        batt_annual_ops = float(np.asarray(battery_annual_ops_gwp(n_batt_annual, battery_LC_GWP)))
        batt_flight_cycle_gwp = float(np.asarray(battery_flight_cycle_gwp(batt_annual_ops, FC_a)))

        gwp_op_per_cycle = float(np.asarray(gwp_operational_per_cycle(E_trip, p.GWP_energy)))
        gwp_flight_val = float(np.asarray(gwp_flight(gwp_op_per_cycle, batt_flight_cycle_gwp)))

        gwp_annual_ops = float(np.asarray(gwp_annual(gwp_flight_val, FC_a)))

        # energy vs. battery GWP breakdown
        t_trip_hr = max(t_trip, eps) / 3600.0
        gwp_flight_safe = gwp_flight_val if gwp_flight_val != 0.0 else eps

        outputs['GWP_flight'] = gwp_flight_val
        outputs['GWP_annual_ops'] = gwp_annual_ops
        outputs['DOD'] = DOD
        outputs['C_rate_hover'] = C_hover
        outputs['C_rate_climb'] = C_climb
        outputs['C_rate_cruise'] = C_cruise
        outputs['C_rate_avg'] = C_avg
        outputs['n_battery_lifecycle'] = N_cycles_available
        outputs['n_batt_annual'] = n_batt_annual

        outputs['GWP_energy_flight'] = gwp_op_per_cycle
        outputs['GWP_battery_flight'] = batt_flight_cycle_gwp
        outputs['GWP_per_hr'] = gwp_flight_val / t_trip_hr
        outputs['GWP_energy_per_hr'] = gwp_op_per_cycle / t_trip_hr
        outputs['GWP_battery_per_hr'] = batt_flight_cycle_gwp / t_trip_hr
        outputs['GWP_energy_annual'] = gwp_op_per_cycle * FC_a
        outputs['GWP_battery_annual'] = batt_flight_cycle_gwp * FC_a
        outputs['GWP_energy_frac'] = float(np.asarray(gwp_energy_fraction(E_trip, p.GWP_energy, gwp_flight_safe)))
        outputs['GWP_battery_frac'] = float(np.asarray(gwp_battery_fraction(outputs['GWP_energy_frac'])))

    def compute_partials(self, inputs, partials):
        in_names = ['E_trip', 'm_battery', 'rho_bat', 'FC_a', 't_trip', 't_climb', 'P_req_total_hover', 'P_req_total_climb', 'P_req_total_cruise', 'c_charge']
        out_names = [
            'GWP_flight', 'GWP_annual_ops', 'DOD', 'C_rate_hover', 'C_rate_climb', 'C_rate_cruise', 'C_rate_avg',
            'n_battery_lifecycle', 'n_batt_annual',
            'GWP_energy_flight', 'GWP_battery_flight', 'GWP_per_hr', 'GWP_energy_per_hr', 'GWP_battery_per_hr',
            'GWP_energy_annual', 'GWP_battery_annual', 'GWP_energy_frac', 'GWP_battery_frac',
        ]

        x = jnp.array([inputs[n][0] for n in in_names])

        def fun(x):
            E_trip, m_batt, rho_bat, FC_a, t_trip, t_climb, P_hover, P_climb, P_cruise, c_charge = x
            p = self.options['parameters']
            eps = 1e-6
            E_batt_design_Wh = battery_energy_capacity(rho_bat, m_batt)
            E_batt_design_Wh = jnp.maximum(E_batt_design_Wh, eps)
            C_hover = c_rate(P_hover, E_batt_design_Wh)
            C_climb = c_rate(P_climb, E_batt_design_Wh)
            C_cruise = c_rate(P_cruise, E_batt_design_Wh)
            t_hover = p.time_hover
            eps = 1e-3
            t_cruise = jnp.maximum(t_trip - t_hover - t_climb, eps)
            C_avg = c_rate_average(C_hover, C_climb, C_cruise, t_hover, t_climb, t_cruise, t_trip)
            DOD = depth_of_discharge(E_trip, E_batt_design_Wh)
            DOD = jnp.clip(DOD, eps, 1.0)
            N_cycles_available = battery_cycle_life(DOD, C_avg, c_charge)
            N_cycles_available = jnp.maximum(N_cycles_available, 1.0)
            time_turn = turnaround_time(c_charge, DOD)
            DH = time_efficiency_ratio(time_turn, t_trip)
            n_batt_annual = number_of_battery_required_annually(N_cycles_available, p.N_wd, p.T_D, t_trip, DH)
            battery_LC_GWP = battery_lifecycle_gwp(p.GWP_battery, E_batt_design_Wh)
            batt_annual_ops = battery_annual_ops_gwp(n_batt_annual, battery_LC_GWP)
            batt_flight_cycle_gwp = battery_flight_cycle_gwp(batt_annual_ops, FC_a)
            gwp_op_per_cycle = gwp_operational_per_cycle(E_trip, p.GWP_energy)
            gwp_flight_val = gwp_flight(gwp_op_per_cycle, batt_flight_cycle_gwp)
            gwp_annual_ops = gwp_annual(gwp_flight_val, FC_a)

            t_trip_hr = jnp.maximum(t_trip, eps) / 3600.0
            gwp_flight_safe = jnp.where(gwp_flight_val != 0.0, gwp_flight_val, eps)
            gwp_energy_frac_val = gwp_energy_fraction(E_trip, p.GWP_energy, gwp_flight_safe)
            gwp_battery_frac_val = gwp_battery_fraction(gwp_energy_frac_val)

            return jnp.array([
                gwp_flight_val,
                gwp_annual_ops,
                DOD,
                C_hover,
                C_climb,
                C_cruise,
                C_avg,
                N_cycles_available,
                n_batt_annual,
                gwp_op_per_cycle,
                batt_flight_cycle_gwp,
                gwp_flight_val / t_trip_hr,
                gwp_op_per_cycle / t_trip_hr,
                batt_flight_cycle_gwp / t_trip_hr,
                gwp_op_per_cycle * FC_a,
                batt_flight_cycle_gwp * FC_a,
                gwp_energy_frac_val,
                gwp_battery_frac_val,
            ])

        J = jax.jacfwd(fun)(x)
        J = np.asarray(J)
        J = np.nan_to_num(J, nan=0.0, posinf=0.0, neginf=0.0)

        for i, out in enumerate(out_names):
            for j, inp in enumerate(in_names):
                partials[(out, inp)] = J[i, j]
