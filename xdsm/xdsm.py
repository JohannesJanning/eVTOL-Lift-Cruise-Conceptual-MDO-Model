"""XDSM diagram matching the actual eVTOLGroup / run_optimize_evtol.py structure.

Diagram layout follows src/optimizer/eVTOL_group.py:
- aero, perf, energy, mass, mtom (implicit MTOM balance) are mutually coupled
  through the MTOM <-> MTOM_est feedback loop and are solved together by the
  group's NewtonSolver -> represented here as a single MDA solver block.
- geom / cons (rotor spacing constraint) only need r_hover / b, independent of
  the MDA loop.
- ops and gwp are coupled to each other (FC_a <-> DOD) but sit downstream of
  the MDA.
- economic and noise are purely downstream (post-processing) components.
- design variables and constraints/objective match run_optimize_evtol.py
  exactly (rho_bat is a fixed IndepVarComp input, not a design variable).
"""
from pyxdsm.XDSM import XDSM, OPT, SOLVER, FUNC, IFUNC
import os

xdsm = XDSM()

xdsm.add_system('Opt', OPT, r"\text{Optimizer}")
xdsm.add_system('Solver', SOLVER, r"\text{MDA Solver (Newton)}")
xdsm.add_system('Aero', FUNC, r"\text{Aerodynamics}")
xdsm.add_system('Perf', FUNC, r"\text{Performance}")
xdsm.add_system('Energy', FUNC, r"\text{Energy}")
xdsm.add_system('Mass', FUNC, r"\text{Mass}")
xdsm.add_system('MTOM', IFUNC, r"\text{MTOM Balance}")
xdsm.add_system('Geom', FUNC, r"\text{Geometry}")
xdsm.add_system('Cons', FUNC, r"\text{Rotor Spacing Constraint}")
xdsm.add_system('Ops', FUNC, r"\text{Operations}")
xdsm.add_system('GWP', FUNC, r"\text{GWP}")
xdsm.add_system('Economic', FUNC, r"\text{Economic}")
xdsm.add_system('Noise', FUNC, r"\text{Noise}")

# -----------------------------------------------------------------------------
# Design variables / fixed inputs (from run_optimize_evtol.py)
# -----------------------------------------------------------------------------
xdsm.add_input('Opt', r"b^{(0)}, c^{(0)}, R_{cr}^{(0)}, R_{hv}^{(0)}, V_{cr}^{(0)}, V_{cl}^{(0)}, C_{charge}^{(0)}")
xdsm.add_input('Mass', r"\rho_{bat}")
xdsm.add_input('GWP', r"\rho_{bat}")

xdsm.connect('Opt', 'Solver', r"b, c, R_{cr}, R_{hv}, V_{cr}, V_{cl}")
xdsm.connect('Opt', 'Geom', r"R_{hv}")
xdsm.connect('Opt', 'Cons', r"b")
xdsm.connect('Opt', 'Ops', r"C_{charge}")
xdsm.connect('Opt', 'GWP', r"C_{charge}")
xdsm.connect('Opt', 'Economic', r"C_{charge}")
xdsm.connect('Opt', 'Noise', r"R_{hv}")

# -----------------------------------------------------------------------------
# MDA loop: Solver <-> Aero, Perf, Energy, Mass, MTOM
# -----------------------------------------------------------------------------
xdsm.connect('Solver', 'Aero', r"b, c, V_{cr}, V_{cl}, M_{TOM}")
xdsm.connect('Solver', 'Perf', r"b, c, R_{cr}, R_{hv}, V_{cr}, V_{cl}, M_{TOM}")
xdsm.connect('Solver', 'Energy', r"V_{cr}, V_{cl}")
xdsm.connect('Solver', 'Mass', r"b, c, R_{cr}, R_{hv}, V_{cr}, M_{TOM}")

xdsm.connect('Aero', 'Perf', r"C_L, C_D")
xdsm.connect('Aero', 'Solver', r"AR, C_{L,cr}, C_{L,cl}")

xdsm.connect('Perf', 'Energy', r"V_{cl,hor}, \{P\}_{hv/cl/cr}")
xdsm.connect('Perf', 'Mass', r"\{P\}_{hv/cl/cr}")
xdsm.connect('Perf', 'Solver', r"\gamma")

xdsm.connect('Energy', 'Mass', r"E_{req}")
xdsm.connect('Energy', 'Solver', r"t_{trip}")

xdsm.connect('Mass', 'MTOM', r"M_{TOM,est}")
xdsm.connect('MTOM', 'Solver', r"M_{TOM}")

xdsm.connect('Solver', 'Opt', r"M_{TOM}, M_{TOM,est}, C_{L,cr}, C_{L,cl}, \gamma, AR")

# -----------------------------------------------------------------------------
# Rotor spacing / vertiport constraint (independent of MDA)
# -----------------------------------------------------------------------------
xdsm.connect('Geom', 'Cons', r"R_{spacing}")
xdsm.connect('Geom', 'Opt', r"S_{vertiport}")
xdsm.connect('Cons', 'Opt', r"c_1 = b - R_{spacing}")

# -----------------------------------------------------------------------------
# Downstream: operations, GWP, economics, noise (post-MDA)
# -----------------------------------------------------------------------------
xdsm.connect('Solver', 'Ops', r"t_{trip}")
xdsm.connect('Solver', 'GWP', r"E_{trip}, m_{bat}, \{P\}_{hv/cl/cr}, t_{trip}, t_{cl}")
xdsm.connect('Solver', 'Economic', r"E_{trip}, M_{TOM}, t_{trip}, m_{bat}, m_{empty}")
xdsm.connect('Solver', 'Noise', r"M_{TOM}")

xdsm.connect('Ops', 'GWP', r"FC_a")
xdsm.connect('GWP', 'Ops', r"DOD")
xdsm.connect('GWP', 'Economic', r"DOD, n_{bat,annual}")

xdsm.connect('Noise', 'Opt', r"SPL_{hv}")

# -----------------------------------------------------------------------------
# Final reported KPIs (not wired as optimizer constraints/objective)
# -----------------------------------------------------------------------------
xdsm.add_output('Economic', r"\mathcal{C}_{TOC}, \Pi_{flight}", side='right')
xdsm.add_output('GWP', r"GWP_{flight}, GWP_{annual}", side='right')
xdsm.add_output('Opt', r"b^*, c^*, R_{cr}^*, R_{hv}^*, V_{cr}^*, V_{cl}^*, C_{charge}^*", side='left')

xdsm.write('xdsm_evtol_group', build=True, cleanup=True)

for ext in ['.tex', '.aux', '.log', '.tikz']:
    try:
        os.remove('xdsm_evtol_group' + ext)
    except FileNotFoundError:
        pass
