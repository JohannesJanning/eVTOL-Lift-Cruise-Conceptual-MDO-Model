"""
Parameter configuration for eVTOL conceptual design model.
"""

# =============================================================================
# INITIAL VALUES
# =============================================================================
MTOM_initial            = 5000     # Maximum take-off mass (initial guess)          [kg]

# =============================================================================
# CONVERSION FACTORS
# =============================================================================
min_per_s               = 1 / 60   # Minutes per second                            [min/s]
km_per_m                = 1 / 1000 # Kilometers per meter                          [km/m]
ft_per_m                = 3.28084  # Feet per meter                                [ft/m]

# =============================================================================
# OPERATIONS PARAMETERS
# =============================================================================
h_hover                 = 15.24                    # Hover altitude above ground (50 ft)      [m]
h_hover_ft              = h_hover * ft_per_m       # Hover altitude above ground              [ft]
h_cruise                = 1219.2                   # Cruise altitude above ground (4000 ft)   [m]
h_cruise_ft             = h_cruise * ft_per_m      # Cruise altitude above ground             [ft]
r_obs_ft                = 250                      # Noise observer distance                  [ft]
distance_trip_km        = 70                       # Design trip distance                     [km]
distance_trip           = distance_trip_km * 1000  # Design trip distance                     [m]
time_hover              = 60                       # Hover duration (takeoff + landing)       [s]
time_reserve            = 20 * 60                  # Reserve flight time (e.g. 20 min)         [s]

# =============================================================================
# PHYSICAL CONSTANTS & EFFICIENCIES
# =============================================================================
g                       = 9.81            # Gravitational acceleration                  [m/s^2]
eta_e                   = 0.9             # Electrical efficiency                       [-]
eta_p                   = 0.85            # Propulsive efficiency (cruise)              [-]
eta_hp                  = 0.7             # Hover propulsive efficiency                 [-]
eta_h                   = eta_hp * eta_e  # Total hover efficiency                      [-]
eta_c                   = eta_p * eta_e   # Total cruise efficiency                     [-]

# =============================================================================
# AERODYNAMIC PARAMETERS
# =============================================================================
rho                     = 1.225    # Air density at sea level                     [kg/m^3]
e                       = 0.8      # Oswald efficiency factor                     [-]
c_d_min                 = 0.0397   # Minimum drag coefficient                     [-]
roc_climb_target_ft_min = 900.0                                           # Target rate of climb  [ft/min]
roc_climb_target        = roc_climb_target_ft_min / ft_per_m * min_per_s  # Target rate of climb  [m/s]
C_T_hover               = 0.1      # Thrust coefficient of hovering rotor         [-]

# =============================================================================
# AIRCRAFT GEOMETRY & FIXED PARAMETERS
# =============================================================================
n_prop_hor              = 1         # Number of cruise (horizontal) propellers        [-]
n_prop_vert             = 8         # Number of hover (vertical) propellers           [-]
n_bladed_hor            = 2         # Number of blades per cruise propeller           [-]
n_blade_vert            = 2         # Number of blades per hover propeller            [-]
l_fus_m                 = 6         # Fuselage length                                 [m]
r_fus_m                 = 0.75      # Fuselage radius (-> diameter = 1.5 m)           [m]
rho_bat                 = 250       # Battery energy density                          [Wh/kg]
m_pay                   = 392.8     # Payload mass (4 pax + luggage)                  [kg]
m_crew                  = 82.5 + 14 # Crew mass (pilot + equipment)                   [kg]
d_rotors_space          = 0.025     # Minimum horizontal distance between hover rotors [m]

# =============================================================================
# ECONOMIC & OPERATIONS PARAMETERS
# =============================================================================
GWP_battery             = 124.5           # GWP of battery manufacturing                [kg CO2e/kWh]
GWP_energy              = 0.37896         # GWP of electricity generation               [kg CO2e/kWh]
P_bat_s                 = 115             # Battery replacement cost                    [EUR/kWh]
P_e                     = 0.096668        # Electricity price                           [EUR/kWh]
P_s_empty               = 1436.5          # Aircraft acquisition cost per kg empty mass [EUR/kg]
U_pilot                 = 2000 * 60 * 60  # Annual pilot utilization time               [s]
S_P                     = 45300           # Pilot salary per year                       [EUR/yr]
N_AC                    = 1               # Number of aircraft per pilot                [-]
fare_km                 = 1.98            # Passenger fare per km                       [EUR/km]
N_s                     = 4               # Number of paying seats                      [-]
LF                      = 0.68            # Average load factor, from Uber data         [-]
unitrate                = 80.14           # Unit rate, DFS 2024                         [EUR]
pm                      = 0.05            # Profit margin                               [-]
N_wd                    = 260              # Working days per year, based on Uber model     [-]
T_D                     = 8 * 60 * 60      # Daily operating time window (8 hours)          [s]
