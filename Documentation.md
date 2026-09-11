# eVTOL Model Documentation

## Scope

This document describes the eVTOL aircraft model assembled by `eVTOLGroup` in OpenMDAO. It documents the implementation in `src/` as the authoritative model and uses the supplied *Supplementary Material: Future Pathways for eVTOLs - A Design Optimization Perspective* to explain assumptions and identify references.

The transportation-mode comparison and all utility/objective-aggregation models are intentionally excluded.

The documented model contains:

- aerodynamic force coefficients;
- hover, climb, and cruise thrust and power;
- mission time and energy;
- battery sizing and aircraft mass closure;
- operational utilization;
- battery degradation;
- operating costs, revenue, and profit;
- operational and battery-manufacturing global-warming potential (GWP);
- hover tonal noise; and
- wing/rotor and vertiport geometry measures.

> **Implementation convention.** Equations below reproduce the current source code. Where the supplementary material differs from the code, the difference is called out explicitly.

## Model Assembly and Coupling

`eVTOLGroupFixedBaseline` promotes the variables of the explicit subsystems and closes the aircraft mass loop through an implicit MTOM component:

```text
wing/rotor geometry + speeds + MTOM
        -> aerodynamics
        -> thrust and power
        -> mission time and energy
        -> battery and empty mass
        -> MTOM estimate
        -> implicit MTOM closure

mission time + battery state
        -> annual utilization
        -> battery replacements
        -> economics and GWP

MTOM + hover power + hover radius
        -> hover tonal noise
```

The group uses a Newton nonlinear solver with subsystem solves (`maxiter = 50`, relative tolerance `1e-6`) and an assembled direct linear solver.

The MTOM state satisfies

$$
R_{\mathrm{MTOM}} = m_{\mathrm{TOM}}-m_{\mathrm{TOM,est}}=0,
$$

so the converged MTOM equals the sum of empty mass, battery mass, and payload mass. The residual Jacobian is analytically defined, with a numerical regularization of `1e-8` on $\partial R/\partial m_{\mathrm{TOM}}$.

## Parameters

### Principal model inputs

These variables are expected to be supplied to the promoted OpenMDAO group, or otherwise retain the component-level initialization value.

| Symbol / OpenMDAO name | Meaning | Unit | Component initialization |
|---|---|---:|---:|
| $b$ / `b` | rectangular wing span | m | 10.0 |
| $c$ / `c` | rectangular wing chord | m | 1.5 |
| $V_{cr}$ / `V_cruise` | cruise true airspeed | m/s | 50.0 in aero/performance; 30.0 in mass |
| $V_{cl}$ / `V_climb` | total climb speed | m/s | 50.0 |
| $R_{cr}$ / `r_cruise` | cruise-propeller radius | m | 1.5 |
| $R_h$ / `r_hover` | hover-rotor radius | m | 0.5 in mass; 1.0 in performance/geometry |
| $\rho_{bat}$ / `rho_bat` | installed battery specific energy | Wh/kg | 300 in components; central default 250 |
| $C_{ch}$ / `c_charge` | charging C-rate | h$^{-1}$ | 1.0 |
| $m_{TOM}$ / `MTOM` | implicit maximum take-off mass | kg | 1500 in components; central initial value 2000 |

Component initialization values are numerical starting values, not necessarily the values used by a particular optimization driver.

### Fixed physical and mission parameters

| Parameter | Symbol | Current value | Unit / note |
|---|---:|---:|---|
| gravitational acceleration | $g$ | 9.81 | m/s² |
| sea-level air density | $\rho$ | 1.225 | kg/m³ |
| Oswald efficiency factor | $e$ | 0.8 | - |
| minimum/parasite drag coefficient | $C_{D0}$ | 0.03 | - |
| electrical efficiency | $\eta_e$ | 0.90 | - |
| hover propulsive efficiency | $\eta_{hp}$ | 0.70 | - |
| hover-system efficiency | $\eta_h=\eta_e\eta_{hp}$ | 0.63 | - |
| cruise propulsive efficiency | $\eta_p$ | 0.85 | - |
| cruise/climb system efficiency | $\eta_c=\eta_e\eta_p$ | 0.765 | - |
| target rate of climb | $\mathrm{ROC}$ | 900 | ft/min = 4.572 m/s |
| hover altitude | $h_h$ | 15.24 | m |
| cruise altitude | $h_{cr}$ | 1219.2 | m |
| trip distance | $d_{trip}$ | 70 | km |
| total hover time | $t_h$ | 60 | s, take-off plus landing |
| reserve time | $t_{res}$ | 1200 | s (20 min) |
| vertical rotors | $n_h$ | 8 | - |
| cruise propellers | $n_{cr}$ | 1 | - |
| blades per hover rotor | $B_h$ | 2 | - |
| hover thrust coefficient | $C_{T,h}$ | 0.1 | - |
| fuselage length | $l_f$ | 6.0 | m |
| fuselage radius | $r_f$ | 0.75 | m |
| payload mass | $m_{pay}$ | 392.8 | kg |
| crew mass | $m_{crew}$ | 96.5 | kg |
| nominal rotor clearance | $d_r$ | 0.00125 | m |

### Fixed operational, environmental, and economic parameters

| Parameter | Symbol | Current value | Unit / note |
|---|---:|---:|---|
| operating days per year | $N_{wd}$ | 260 | days/year |
| daily operating window | $T_D$ | 8 | h/day (stored as 28,800 s) |
| grid electricity GWP | $g_e$ | 0.37896 | kg CO2e/kWh |
| battery production GWP | $g_{bat}$ | 124.5 | kg CO2e/kWh |
| electricity price | $P_e$ | 0.096668 | EUR/kWh |
| battery replacement price | $P_{bat}$ | 115 | EUR/kWh |
| aircraft price per empty mass | $P_{empty}$ | 1436.5 | EUR/kg |
| pilot annual utilization | $U_{pilot}$ | 2000 | h/year |
| pilot annual salary | $S_P$ | 45,300 | EUR/year |
| aircraft controlled per pilot | $N_{AC}$ | 1 | - |
| fare | $f$ | 1.98 | EUR/passenger-km |
| paying seats | $N_s$ | 4 | - |
| load factor | $LF$ | 0.68 | - |
| navigation unit rate | $u_r$ | 80.14 | EUR |

# Models

# Physical Models

## 1. Geometry Model

For a rectangular wing,

$$
S=bc, \qquad AR=\frac{b}{c}=\frac{b^2}{S}.
$$

For the fixed eight-lift-rotor layout, the code reports the minimum span required to avoid rotor/fuselage interference as

$$
b_{rotor}=2\left(3R_h+2d_r+r_f\right),
$$

and the overall rotor envelope relevant to a vertiport as

$$
D_{rotor}=2\left(4R_h+2d_r+r_f\right).
$$

These are geometry outputs; `eVTOLGroupFixedBaseline` itself does not apply numerical upper or lower bounds. Those bounds must be imposed by the optimization setup.

**Assumptions.** Eight hover rotors are arranged symmetrically on a lift-plus-cruise configuration; all hover rotors have equal radius; the fuselage is represented by radius $r_f$; rotor clearance is fixed.

**References.** The layout follows the supplementary material's Eqs. (108)-(112), drawing on EASA propeller-clearance guidance [7], FAA propeller-clearance rules [8], and the EASA vertiport D-value definition [35].

## 2. Aerodynamic Model

### 2.1 Required lift coefficients

The current implementation does not prescribe angle of attack or use the NACA lift-curve regression to calculate $C_L$. Instead, it calculates the lift coefficient required by force balance at the specified speeds:

$$
C_{L,cr}=\frac{2m_{TOM}g}{\rho V_{cr}^2S},
$$

$$
\gamma=\sin^{-1}\left(\frac{\mathrm{ROC}}{V_{cl}}\right),
\qquad
C_{L,cl}=\frac{2m_{TOM}g\cos\gamma}{\rho V_{cl}^2S}.
$$

The argument of the inverse sine is clipped to $[-0.999999,0.999999]$, and speeds/area are given small positive numerical floors.

### 2.2 Drag polar and forces

The finite-wing parabolic drag polar is

$$
C_D=C_{D0}+\frac{C_L^2}{\pi AR e},
$$

and the drag in each forward-flight segment is

$$
D=\frac{1}{2}\rho V^2SC_D.
$$

**Assumptions.** Rectangular wing; steady flight; constant sea-level density; induced drag represented by lifting-line theory; one constant parasite-drag coefficient represents the aircraft and stopped lift rotors; compressibility, Reynolds-number variation, stall, transition, rotor-wing interaction, and moment trim are not resolved.

**References.** Lifting-line and conceptual optimization formulations are attributed to Martins and Ning [11] and Chauhan and Martins [12]. The supplementary material links the parasite-drag treatment to Kaneko and Martins [5] and lift-rotor drag experiments by Bacchini et al. [14]. Its NACA data were generated with XFOIL [15], but those airfoil regressions are not used by the current `AerodynamicsComp`.

**Implementation difference.** The supplementary material states $C_{D0}=0.0397$; the current parameter file uses **0.03**. It also describes fixed climb/cruise angles of attack, whereas the code derives $C_L$ from force balance and derives $\gamma$ from the specified ROC.

## 3. Thrust and Power Model

### 3.1 Rotor disk quantities

For a rotor of radius $R$,

$$
A=\pi R^2, \qquad T_p=\frac{T}{n}.
$$

In hover, the per-rotor disk loading and induced velocity are

$$
\sigma_h=\frac{T_{p,h}}{A_h},
\qquad
v_{i,h}=\sqrt{\frac{\sigma_h}{2\rho}}.
$$

The implementation smoothly floors disk area at approximately $0.5\ \mathrm{m^2}$ for numerical robustness.

### 3.2 Hover

The implemented hover thrust contains a 25% thrust margin:

$$
T_h=1.25\,m_{TOM}g.
$$

Hover power is

$$
P_h=\frac{T_hv_{i,h}}{\eta_h}.
$$

### 3.3 Climb

The horizontal component of climb speed is

$$
V_{cl,x}=V_{cl}\cos\gamma.
$$

Required climb thrust is

$$
T_{cl}=D_{cl}+m_{TOM}g\sin\gamma.
$$

For climb or cruise, the induced velocity of each cruise propeller is

$$
v_i=-\frac{V}{2}+\sqrt{\left(\frac{V}{2}\right)^2+\frac{T_p}{2\rho A_p}},
$$

and total shaft/electrical power is represented as

$$
P=\frac{TV+nT_pv_i}{\eta_c}.
$$

The climb expression uses $V=V_{cl}$, $T=T_{cl}$, and the cruise-propeller count and area.

### 3.4 Cruise

Steady cruise thrust equals drag:

$$
T_{cr}=D_{cr},
$$

and the same forward-flight power equation is evaluated with $V_{cr}$, $T_{cr}$, and the cruise propeller.

**Assumptions.** Steady, unaccelerated segment conditions; actuator-disk momentum theory; uniform inflow; no blade-element effects; no rotor interference; fixed efficiencies; one cruise propeller; eight hover rotors; climb propulsion is provided by the cruise propeller.

**References.** The supplementary material bases the force balances and momentum-theory treatment on Rotaru and Todorov [18] and Marchman [19], with efficiency assumptions from Brown and Harris [16] and Yang et al. [17].

**Implementation difference.** The supplementary hover equation uses $T_h=m_{TOM}g$; the code uses $1.25m_{TOM}g$. The PDF lists $\eta_c\approx0.77$; the exact product in code is $0.85\times0.90=0.765$.

## 4. Mission Time and Energy Model

Climb time is prescribed by altitude gain and target ROC:

$$
t_{cl}=\frac{h_{cr}-h_h}{\mathrm{ROC}}.
$$

The horizontal distance covered in climb is $d_{cl}=V_{cl,x}t_{cl}$. Remaining cruise time is

$$
t_{cr}=\frac{\max(d_{trip}-d_{cl},10^{-6})}{\max(V_{cr},10^{-3})}.
$$

Total airborne trip time is

$$
t_{trip}=t_h+t_{cl}+t_{cr}.
$$

With power in watts and time in seconds, segment energies in watt-hours are

$$
E_h=\frac{P_ht_h}{3600},\qquad
E_{cl}=\frac{P_{cl}t_{cl}}{3600},\qquad
E_{cr}=\frac{P_{cr}t_{cr}}{3600}.
$$

The trip and reserve energies are

$$
E_{trip}=E_h+E_{cl}+E_{cr},
\qquad
E_{res}=\frac{P_{cr}t_{res}}{3600},
$$

and battery-sizing energy is

$$
E_{req}=E_{trip}+E_{res}.
$$

**Assumptions.** One hover, one climb, and one cruise allocation; no descent-energy or regenerative-credit model; climb is at constant ROC and speed; reserve is flown at cruise power; all computed energies are constrained to be non-negative.

**References.** The supplementary material uses the standard $E=Pt$ relation and cites Yang et al. [17] for battery utilization limits. Unlike the PDF's example of a 30-minute reserve, the current code uses **20 minutes**.

## 5. Battery Sizing Model

The installed battery mass is

$$
m_{bat}=\frac{E_{req}}{0.64\rho_{bat}}.
$$

The factor $0.64=0.8\times0.8$ represents 80% end-of-life capacity multiplied by an 80% usable state-of-charge window (90% ceiling to 10% floor). Installed nameplate energy is

$$
E_{bat}=\rho_{bat}m_{bat}.
$$

**Assumptions.** Constant pack-level specific energy; reserve energy is installed but normally unused; end-of-life capacity is 80% of beginning-of-life capacity; only 80% of that capacity is operationally usable; pack structural, thermal-management, and power-density constraints are not modeled separately.

**References.** Battery usable-energy assumptions are attributed to Yang et al. [17].

## 6. Empty-Mass Model

The empty mass assembled in the code is

$$
m_{empty}=m_w+m_{motor}+m_{rotor}+m_{crew}+m_{furn}+m_{fus}+m_{sys}+m_{gear}.
$$

The MTOM estimate is

$$
m_{TOM,est}=m_{empty}+m_{bat}+m_{pay}.
$$

Crew mass is included in `m_empty`; it is not added a second time in the MTOM closure.

Most component equations are legacy empirical weight equations evaluated in imperial units and converted back to kilograms using approximately $1\ \mathrm{kg}=2.205\ \mathrm{lb}$, $1\ \mathrm{m}=3.281\ \mathrm{ft}$, and $1\ \mathrm{m/s}=1.9438\ \mathrm{kt}$.

### 6.1 Wing mass

For the rectangular wing, $S_w=b_fc_f$ in ft² and $AR=b/c$. Dynamic pressure is converted to lbf/ft². The code evaluates Raymer- and Nicolai-labelled correlations:

$$
W_{w,R}=0.036S_w^{0.758}
\left(\frac{AR}{\cos^2\Lambda_{c/4}}\right)^{0.6}
q^{0.006}\lambda^{0.04}
\left(\frac{100t/c}{\cos\Lambda_{c/4}}\right)^{-0.3}
(n_zW_0)^{0.49},
$$

$$
W_{w,N}=96.948
\left(\frac{n_zW_0}{10^5}\right)^{0.65}
\left(\frac{AR}{\cos^2\Lambda_{c/4}}\right)^{0.57}
\left(\frac{S_w}{100}\right)^{0.61}
\left(1+\frac{\lambda}{2}(t/c)\right)^{0.36}
\left(\sqrt{1+\frac{V_H}{500}}\right)^{0.993},
$$

then averages their kilogram equivalents:

$$
m_w=\frac{m_{w,R}+m_{w,N}}{2}.
$$

The implementation fixes $\Lambda_{c/4}=0$, $\lambda=1$, $t/c=0.12$, $n_z=2.5$, zero fuel-in-wing factor, and $V_H=V_{cr}$.

### 6.2 Rotor mass

$$
m_{rotor}=13\left[n_h\left(0.7484R_h^{1.2}-0.0403R_h\right)
+n_{cr}\left(0.7484R_{cr}^{1.2}-0.0403R_{cr}\right)\right].
$$

**Implementation difference.** The supplementary material reports a calibration factor of 22.649; the current source uses **13.0**.

### 6.3 Motor mass

With a 1.5 power-sizing margin and $745.7\ \mathrm{W/hp}$,

$$
m_{motor}=n_h\,0.6756\left(\frac{1.5P_h}{n_h\,745.7}\right)^{0.783}
+n_{cr}\,0.6756\left(\frac{1.5P_{cl}}{n_{cr}\,745.7}\right)^{0.783}.
$$

Cruise motor mass is sized by climb power, which is assumed to be the more demanding horizontal-propulsion segment.

**Reference.** Govindarajan and Sridharan [9]. The supplementary text is internally inconsistent: its assumptions state a 1.5 margin while its equation explanation says 2.5; the code uses **1.5**.

### 6.4 Fuselage mass

The fuselage is approximated as an unpressurized cylinder. In imperial units,

$$
S_{fus}=2\pi r_fl_f+\pi r_f^2,\quad d_{FS}=2r_f,\quad
q=\tfrac12\rho V_{cr}^2,
$$

and the code computes

$$
W_{fus,R}=0.052S_{fus}^{1.086}(n_zW_0)^{0.177}l_{HT}^{-0.051}
\left(\frac{l_{FS}}{d_{FS}}\right)^{-0.072}q^{0.241},
$$

with $l_{HT}=0.5l_f$, and

$$
W_{fus,N}=200\left(\frac{n_zW_0}{10^5}\right)^{0.286}
\left(\frac{l_f}{10}\right)^{0.857}
\left(\frac{w_f+d_f}{10}\right)^{0.338}
\left(\frac{V_H}{100}\right)^{1.1}.
$$

The implementation sets the pressurized-volume term to zero and sets $w_f=d_f=r_f$ in the Nicolai-labelled correlation. The converted estimates are averaged.

### 6.5 Landing-gear mass

Cruise-propeller radius is smoothly floored at 0.7 m. Gear length is approximated by

$$
l_m=l_n=R_{cr}-r_f+0.1778\ \mathrm{m},
$$

then converted to inches. With $n_l=1.5\times2.5=3.75$ and landing weight $W_L$ in lbf,

$$
W_{mg,R}=0.095(n_lW_L)^{0.768}\left(\frac{l_m}{12}\right)^{0.409},
$$

$$
W_{ng,R}=0.125(n_lW_L)^{0.566}\left(\frac{l_n}{12}\right)^{0.845},
$$

$$
W_{g,N}=0.054(n_lW_L)^{0.684}\left(\frac{l_m}{12}\right)^{0.601}.
$$

The converted terms are combined as

$$
m_{gear}=\frac{m_{mg,R}+m_{ng,R}+m_{g,N}}{2}.
$$

**References.** Raymer [3] and the source labelled “Nicolai” (bibliography entry [4] is Gudmundsson). Propeller clearance is linked to EASA [7] and FAA [8].

### 6.6 Systems mass

In imperial units,

$$
W_{sys,N}=1.08W_0^{0.7},
$$

$$
W_{sys,R}=0.054l_f^{1.536}b^{0.371}
\left(1.5\,n_zW_0\,10^{-4}\right)^{0.8},
$$

with $n_z=2.5$, followed by conversion and averaging:

$$
m_{sys}=\frac{m_{sys,N}+m_{sys,R}}{2}.
$$

The systems category represents flight controls, avionics, environmental control, lighting, and other non-propulsive electrical equipment.

### 6.7 Furnishings/interior mass

With $W_0$ in lbf and $q$ in lbf/ft²,

$$
W_{furn,R}=0.0582W_0-65,
\qquad
W_{furn,N}=34.5q^{0.25},
$$

and

$$
m_{furn}=\frac{m_{furn,R}+m_{furn,N}}{2}
$$

after conversion to kilograms.

**General assumptions and references for empirical masses.** These are conceptual, statistically fitted general-aviation correlations and are sensitive to units and calibration data. Detailed load paths, materials, crashworthiness, thermal systems, manufacturing, and finite-element structural sizing are outside scope. The supplementary material attributes the correlations to Raymer [3] and a Nicolai method, although bibliography entry [4] is Gudmundsson; that citation should be checked before publication.

## 7. Operational Utilization Model

Charging turnaround time is

$$
t_{turn}=\frac{DOD}{C_{ch}}\,3600,
$$

where $C_{ch}$ is in h$^{-1}$. The total-cycle-time factor is

$$
D_H=1+\frac{t_{turn}}{t_{trip}}.
$$

Daily and annual flights are

$$
FC_d=\frac{T_D}{t_{trip}D_H}
=\frac{T_D}{t_{trip}+t_{turn}},
\qquad
FC_a=N_{wd}FC_d.
$$

**Assumptions.** A single aircraft repeatedly flies the design mission; turnaround contains charging only; no passenger handling, repositioning, queues, weather, maintenance downtime, curfew variation, or spare-aircraft logic; cycles are continuous expected values rather than integers.

**Reference.** The operating window is based on the Uber Elevate concept [20].

## 8. Battery State and Degradation Model

Depth of discharge and segment C-rates are

$$
DOD=\frac{E_{trip}}{E_{bat}},
\qquad
C_i=\frac{P_i}{E_{bat}},\quad i\in\{h,cl,cr\}.
$$

The mean mission C-rate is time-weighted:

$$
\bar C=\frac{C_ht_h+C_{cl}t_{cl}+C_{cr}t_{cr}}{t_{trip}}.
$$

The implemented empirical battery life is

$$
N_{cyc}=(-5986.8421DOD+11776.3158)
\bar C^{-1.1}\left(0.5C_{ch}^{-1.2}\right).
$$

The GWP component clips $DOD$ to $(10^{-6},1]$ and floors $N_{cyc}$ at one cycle. Annual battery-pack consumption is

$$
n_{bat,a}=\frac{N_{wd}T_D}{N_{cyc}\,t_{trip}D_H}
=\frac{FC_a}{N_{cyc}}.
$$

**Assumptions.** Degradation depends only on DoD, average discharge C-rate, and charging C-rate; the linear DoD branch is extrapolated throughout the clipped range; temperature, calendar aging, cell chemistry, thermal gradients, fast-charge protocol, and mission-to-mission variability are not modeled.

**References.** The empirical fit is based on cycle-life data cited as Duffy et al. [29] and Wetherell [30]. The supplementary material says the source data were divided by four to represent harsher eVTOL duty and degradation to 80% of beginning-of-life capacity.

**Implementation issue.** In `GWPComp`, the time used for the climb term of $\bar C$ is set equal to `time_hover` (60 s), rather than the climb time calculated by `EnergyComp`. This changes average C-rate, cycle life, battery replacements, battery cost, and battery GWP. The documentation above gives the intended time-weighted equation; current numerical results use 60 s for both hover and climb weighting.

## 9. Global-Warming-Potential Model

Battery production impact for one pack is

$$
G_{bat,pack}=g_{bat}\frac{E_{bat}}{1000}.
$$

Annual battery-production impact and its allocation per flight are

$$
G_{bat,a}=n_{bat,a}G_{bat,pack},
\qquad
G_{bat,flight}=\frac{G_{bat,a}}{FC_a}.
$$

Operational electricity impact per flight is

$$
G_{elec,flight}=g_e\frac{E_{trip}}{1000}.
$$

Total flight and annual GWP are

$$
G_{flight}=G_{elec,flight}+G_{bat,flight},
\qquad
G_{annual}=FC_aG_{flight}.
$$

All GWP outputs are in kg CO2e. Reserve energy affects battery size but is not counted as consumed operational energy.

**Assumptions.** Constant grid carbon intensity; battery manufacturing is the only embodied aircraft impact; battery production impact is linearly proportional to pack kWh; no recycling credit, infrastructure, airframe manufacturing, maintenance materials, or end-of-life impact.

**Source basis.** The parameter file uses German-grid-style electricity and battery-production factors, consistent with the geographical assumptions discussed in the supplementary material. No distinct literature citation is attached there to the battery-production factor of 124.5 kg CO2e/kWh, so this value should be sourced explicitly before publication.

## 10. Hover Tonal-Noise Model

The group evaluates hover tonal noise only. Per-rotor thrust is

$$
T_{p,h}=\frac{1.25m_{TOM}g}{n_h}.
$$

Rotor speed is inferred from the assumed thrust coefficient:

$$
N_{rps}=\sqrt{\frac{T_{p,h}}{C_{T,h}\rho(2R_h)^4}},
\qquad
N_{rpm}=60N_{rps},
\qquad
\Omega=2\pi N_{rps}.
$$

For the first blade-passing harmonic ($q=1$), effective acoustic radius $R_e=0.8R_h$, speed of sound $a=\sqrt{\gamma_{air}R_{air}T}$, and observer geometry $\theta$, the code implements a Gutin-Deming-type pressure:

$$
k=\frac{qB_h\Omega}{a},
$$

$$
p_{rms}=\frac{qB_h\Omega}{2\sqrt{2}\pi ar_{obs}}
\left|-T_{p,h}\cos\theta+Q\frac{a}{\Omega R_e^2}\right|
J_{qB_h}(kR_e\sin\theta).
$$

The total SPL is

$$
SPL_h=20\log_{10}\left(\frac{p_{rms}}{20\ \mu\mathrm{Pa}}\right)
+10\log_{10}(n_h).
$$

The current defaults use an observer distance of 250 ft, ISA sea-level temperature 288.15 K, $B_h=2$, and $C_{T,h}=0.1$. Multiple rotors are combined incoherently.

**Assumptions and limitations.** Tonal hover noise only; identical rotors; no phase coherence or installation/interference effects; no broadband component in the OpenMDAO `NoiseComp`; no climb/cruise noise, unsteady loading, vortex interaction, atmospheric absorption, ground reflection, exposure metric, or psychoacoustics.

**References.** Deming [31] and Gutin [32]; the assumed urban-hover thrust coefficient is linked to Ha et al. [34]. The supplementary material also describes the Schlegel-King-Mull broadband model [33], but that model is not called by the current OpenMDAO group.

**Implementation issue.** `NoiseComp` already divides total hover power by the number of vertical rotors before passing it to `tonal_noise_hover`; that function divides the supplied power by the rotor count again when calculating torque. The implemented torque term is therefore smaller by a factor of $n_h$ than a single division would imply. This should be reviewed before treating the SPL prediction as validated.

# Economic Models

## 11. Cash Operating Cost

### 11.1 Energy cost

$$
C_E=P_e\frac{E_{trip}}{1000}.
$$

Only energy used on the flown trip is purchased in this expression; reserve capacity is not charged unless used.

### 11.2 Navigation cost

The implemented DFS-style terminal and en-route charges are

$$
C_{term}=u_r\left(\frac{m_{TOM}/1000}{50}\right)^{0.7},
$$

$$
C_{route}=u_r\left(\frac{m_{TOM}/1000}{50}\right)^{0.5}\frac{d_{trip,km}}{100},
$$

$$
C_N=C_{term}+C_{route}.
$$

**Reference.** DFS charging framework [23]. Airport-specific landing, passenger, and infrastructure fees are excluded.

### 11.3 Crew cost

The number of pilots required by the operating schedule is

$$
N_{pilot}=\frac{N_{wd}T_D}{U_{pilot}N_{AC}},
$$

and the allocation of annual salary to one trip simplifies to

$$
C_C=S_P\,N_{pilot}\frac{t_{trip}}{(T_D/D_H)N_{wd}}
=\frac{S_Pt_{trip}D_H}{U_{pilot}N_{AC}}.
$$

**Assumptions.** No cabin crew; one aircraft per onboard pilot by default; constant salary and available working hours.

**References.** Uber Elevate [20] for the salary/operational concept; the supplementary material also cites ERI [24] for an unused alternative hourly-rate model.

### 11.4 Maintenance cost

Wrap-rate maintenance is

$$
C_{MWR}=33\frac{t_{trip}}{3600}.
$$

Battery replacement cost allocated to a flight is

$$
C_{MB}=n_{bat,a}P_{bat}\frac{E_{bat}}{1000}
\frac{t_{trip}D_H}{T_DN_{wd}}.
$$

Because $FC_a=N_{wd}T_D/(t_{trip}D_H)$, this is equivalently the annual battery-replacement spend divided by annual flights. Total maintenance is

$$
C_M=C_{MWR}+C_{MB}.
$$

**References.** The wrap-rate assumptions are attributed to Brown and Harris [16]; battery-cost assumptions to Mihara et al. [22].

### 11.5 Cash operating cost total

$$
COC=C_E+C_N+C_C+C_M.
$$

## 12. Cost of Ownership

The empty-mass fraction is

$$
\omega_{empty}=\frac{m_{empty}}{m_{TOM}},
$$

so $\omega_{empty}m_{TOM}P_{empty}=m_{empty}P_{empty}$ is the estimated acquisition price. Per-flight ownership cost is

$$
COO=0.06\,COC+
\frac{0.0796\,\omega_{empty}m_{TOM}P_{empty}}
{N_{wd}T_D/(t_{trip}D_H)}.
$$

The first term is insurance, fixed at 6% of cash operating cost. The second allocates annual annuity depreciation to flights; 0.0796 is the assumed annuity factor.

**Assumptions.** Acquisition price scales linearly with empty mass; fixed 15-year financial life, 3% interest, and 5% residual value underlie the annuity factor described in the supplement; no financing tax treatment or technology-dependent purchase price.

**References.** ATA-67 cost structure [21], Mihara et al. [22] for insurance, Investopedia [25] for annuity depreciation, Valor International [26] for an eVTOL order-value cross-check, and Brown and Harris [17 in the PDF text, but bibliography [17] is Yang et al.] for the mass-specific aircraft price. The last attribution should be checked.

## 13. Direct, Indirect, and Total Operating Cost

Direct operating cost is

$$
DOC=COC+COO.
$$

Indirect operating cost is

$$
IOC=0.233\,COC+
\frac{0.0175\,\omega_{empty}m_{TOM}P_{empty}}
{N_{wd}FC_d}.
$$

The implemented total operating cost per flight is

$$
TOC=DOC+IOC=COC+COO+IOC.
$$

**Assumptions.** The 23.3% and 1.75% factors are fixed proxies for indirect expenses; no explicit airport rent, administration, sales, marketing, route-development, or fleet overhead submodel is resolved.

**References.** ATA-67 structure [21] and ICAO operating-cost/productivity data [27].

## 14. Revenue and Profitability

The current code includes occupied seats in revenue:

$$
R_{flight}=f\,d_{trip,km}\,N_sLF.
$$

The implied passenger ticket price is

$$
P_{ticket}=\frac{R_{flight}}{N_sLF}=f\,d_{trip,km}.
$$

Per-flight and annual profit are

$$
\Pi_{flight}=R_{flight}-TOC,
\qquad
\Pi_{annual}=\Pi_{flight}FC_a.
$$

**Assumptions.** Constant distance-based fare, constant average load factor, no demand elasticity, no time-based pricing, no VAT, no ancillary revenue, and no deadhead/repositioning flights.

**Reference.** The fare assumption is based on a London taxi-fare source [28].

**Implementation difference.** The supplementary equation writes revenue as fare times distance only, while the source multiplies by occupied seats ($N_sLF$). The code formulation is documented here.

## Known Implementation and Documentation Discrepancies

The following items should be resolved before the model is presented as a verified reference implementation:

1. `c_d_min` is 0.03 in code but 0.0397 in the supplementary material.
2. Hover thrust is $1.25m_{TOM}g$ in code but $m_{TOM}g$ in the supplementary equation.
3. Rotor-mass calibration is 13.0 in code but 22.649 in the supplementary material.
4. Motor sizing margin is 1.5 in code; the supplementary section mentions both 1.5 and 2.5.
5. Reserve duration is 20 min in code but the supplementary battery-sizing discussion uses 30 min.
6. Rotor clearance is 0.00125 m in code, while the supplementary material mentions 0.0125 m and later 0.2 m.
7. The GWP/battery-life calculation weights climb C-rate with 60 s instead of the calculated climb time.
8. Hover-noise torque divides per-rotor power by rotor count a second time.
9. The current aero component does not use the NACA lift-curve/AoA model described in the supplementary material.
10. The current OpenMDAO noise component does not use the documented SKM broadband forward-flight model.
11. `params_as_tuple()` references several names that are commented out in `model_parameters.py`; calling that helper in the current file would raise a name error.
12. The source and supplementary bibliography use “Nicolai” terminology inconsistently; bibliography entry [4] is Gudmundsson.

## References

[1] European Union Aviation Safety Agency. *Special Condition VTOL*. 2024.

[3] D. P. Raymer. *Aircraft Design: A Conceptual Approach*. 2nd ed., AIAA, 1992.

[4] S. Gudmundsson. *General Aviation Aircraft Design: Applied Methods and Procedures*. Butterworth-Heinemann, 2013. (The source code and supplement label some equations “Nicolai”; verify the intended original citation.)

[5] S. Kaneko and J. R. R. A. Martins. “Simultaneous Optimization of Design and Takeoff Trajectory for an eVTOL Aircraft.” *Aerospace Science and Technology* 155, 109617, 2024.

[7] EASA. *Easy Access Rules for Large Aeroplanes (CS-25), CS 25.925: Propeller Clearance*. 2024.

[8] Federal Aviation Administration. *14 CFR §25.925 - Propeller Clearance*. 2024.

[9] B. Govindarajan and A. Sridharan. “Conceptual Sizing of Vertical Lift Package Delivery Platforms.” *Journal of Aircraft* 57(6), 2020.

[11] J. R. R. A. Martins and A. Ning. *Engineering Design Optimization*. Cambridge University Press, 2021.

[12] S. S. Chauhan and J. R. R. A. Martins. “Tilt-Wing eVTOL Takeoff Trajectory Optimization.” *Journal of Aircraft* 57(1), 2019.

[14] A. Bacchini, E. Cestino, B. Van Magill, and D. Verstraete. “Impact of Lift Propeller Drag on the Performance of eVTOL Lift+Cruise Aircraft.” *Aerospace Science and Technology* 109, 106429, 2021.

[15] M. Drela. “XFOIL: An Analysis and Design System for Low-Reynolds-Number Airfoils.” 1989.

[16] A. Brown and W. L. Harris. “A Vehicle Design and Optimization Model for On-Demand Aviation.” AIAA/ASCE/AHS/ASC Structures, Structural Dynamics, and Materials Conference, 2018.

[17] C. Yang et al. “Challenges and Key Requirements of Batteries for Electric Vertical Takeoff and Landing Aircraft.” *Joule* 5(8), 1884-1900, 2021.

[18] C. Rotaru and M. Todorov. *Helicopter Flight Physics*. IntechOpen, 2018.

[19] J. F. Marchman III. “Altitude Change: Climb and Glide.” Virginia Tech, 2021.

[20] J. Holden and N. Goel. *Fast-Forwarding to a Future of On-Demand Urban Air Transportation*. Uber Elevate, 2016.

[21] H. B. Faulkner. *The ATA-67 Formula for Direct Operating Cost*. NASA Technical Memorandum, 1973.

[22] Y. Mihara et al. “Cost Analysis of eVTOL Configuration Design for an Air Ambulance System in Japan.” *Journal of Advanced Transportation*, Article 8821234, 2021.

[23] DFS Deutsche Flugsicherung. *Charges - Legal Framework*. 2024.

[24] Economic Research Institute. *Helicopter Pilot Salary in Germany*. 2024.

[25] Investopedia. *Annuity Method of Depreciation*. 2023.

[26] Valor International. “Orders for EVE's Flying Car Total $8.3bn.” 2023.

[27] International Civil Aviation Organization. *Airlines Operating Costs and Productivity*. 2017.

[28] BetterTaxi. *Taxi Fare Calculator London*. Accessed 2024.

[29] M. J. Duffy et al. “A Study in Reducing the Cost of Vertical Flight with Electric Propulsion.” AIAA Conference, 2017.

[30] C. Wetherell. *Let's Talk About the Panasonic NCR18650B*. 2018.

[31] A. F. Deming. *Propeller Rotation Noise Due to Torque and Thrust*. 1940.

[32] L. Gutin. *On the Sound Field of Rotating Propeller*. 1948.

[33] R. Schlegel, R. King, and H. Mull. *Helicopter Rotor Noise Generation and Propagation*. 1966.

[34] T. H. Ha, K. Lee, and J. T. Hwang. “Large-Scale Design-Economics Optimization of eVTOL Concepts for Urban Air Mobility.” 2019.

[35] EASA. *Prototype Technical Specifications for the Design of VFR Vertiports for Operation with Manned VTOL-Capable Aircraft Certified in the Enhanced Category (PTS-VPT-DSN).* 2022.

## Source-to-Section Map

| Documentation section | Principal implementation files |
|---|---|
| Assembly and MTOM closure | `optimizer/eVTOL_group_fixed_baseline.py`, `optimizer/components/mtom_implicit.py` |
| Geometry | `optimizer/components/geometry_comp.py` |
| Aerodynamics | `optimizer/components/aerodynamics_comp.py`, `models_jax/aerodynamics/` |
| Thrust and power | `optimizer/components/performance_comp.py`, `models_jax/momentum/` |
| Mission time and energy | `optimizer/components/energy_comp.py`, `models_jax/time/`, `models_jax/energy/` |
| Mass | `optimizer/components/mass_comp.py`, `models_jax/mass/` |
| Operations | `optimizer/components/ops_comp.py`, `models_jax/operations/ops_model.py` |
| Battery degradation and GWP | `optimizer/components/gwp_comp.py`, `models_jax/battery/`, `models_jax/gwp/` |
| Noise | `optimizer/components/noise_comp.py`, `models_jax/noise/SPL_hover.py` |
| Economics | `optimizer/components/economic_comp.py`, `models_jax/economic/` |

