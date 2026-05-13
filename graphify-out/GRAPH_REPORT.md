# Graph Report - /home/yagiz/Desktop/Bitirme/Çalışmalar/Bahar_Bitirme_Calismalari/Missile_main-main  (2026-05-07)

## Corpus Check
- Large corpus: 130 files · ~3,332,090 words. Semantic extraction will be expensive (many Claude tokens). Consider running on a subfolder, or use --no-semantic to run AST-only.

## Summary
- 213 nodes · 328 edges · 14 communities (12 shown, 2 thin omitted)
- Extraction: 95% EXTRACTED · 5% INFERRED · 0% AMBIGUOUS · INFERRED: 18 edges (avg confidence: 0.59)
- Token cost: 98,085 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_AeroProp Lookup Database|Aero/Prop Lookup Database]]
- [[_COMMUNITY_Core Physics & Control Loop|Core Physics & Control Loop]]
- [[_COMMUNITY_Simulation Runner & Monte Carlo|Simulation Runner & Monte Carlo]]
- [[_COMMUNITY_Autopilot Modes (NDIINDIAccel)|Autopilot Modes (NDI/INDI/Accel)]]
- [[_COMMUNITY_Guidance Laws (ProNavLine)|Guidance Laws (ProNav/Line)]]
- [[_COMMUNITY_INS Error Model|INS Error Model]]
- [[_COMMUNITY_Kinematic Seeker|Kinematic Seeker]]
- [[_COMMUNITY_Quaternion Kinematics|Quaternion Kinematics]]
- [[_COMMUNITY_Target Scenario Dynamics|Target Scenario Dynamics]]
- [[_COMMUNITY_Fin Actuator Dynamics|Fin Actuator Dynamics]]
- [[_COMMUNITY_Aerodynamic Coefficients|Aerodynamic Coefficients]]
- [[_COMMUNITY_Atmosphere & Environment|Atmosphere & Environment]]
- [[_COMMUNITY_Global Configuration|Global Configuration]]
- [[_COMMUNITY_Test Registry|Test Registry]]

## God Nodes (most connected - your core abstractions)
1. `simulate.run` - 18 edges
2. `Missile` - 17 edges
3. `Missile` - 16 edges
4. `Aerodynamics` - 13 edges
5. `Control` - 11 edges
6. `Control` - 10 edges
7. `Guidance` - 9 edges
8. `run()` - 8 edges
9. `INS` - 8 edges
10. `INS` - 7 edges

## Surprising Connections (you probably didn't know these)
- `run()` --calls--> `Missile`  [INFERRED]
  simulate.py → missile/missile.py
- `run()` --calls--> `Target`  [INFERRED]
  simulate.py → target.py
- `simulate.run` --calls--> `Target`  [EXTRACTED]
  simulate.py → target.py
- `simulate.run` --references--> `TESTS registry`  [EXTRACTED]
  simulate.py → tests.py
- `Missile` --uses--> `Sensor`  [INFERRED]
  missile/missile.py → missile/sensor.py

## Hyperedges (group relationships)
- **6-DoF Physics Integration Loop** — environment_environment, kinematics_kinematics, aerodynamics_aerodynamics, propulsion_propulsion, forces_forces, euler_euler, newton_newton [INFERRED 0.95]
- **Midcourse-to-Terminal Guidance Hand-off Chain** — sensor_sensor, guidance_guidance, ins_ins [INFERRED 0.85]
- **Multi-Mode Autopilot Pattern (Accel / NDI / INDI / NDI-CoP)** — control_control_accel, control_control_ndi, control_control_indi, control_control_ndi_cop [EXTRACTED 0.95]

## Communities (14 total, 2 thin omitted)

### Community 0 - "Aero/Prop Lookup Database"
Cohesion: 0.07
Nodes (19): Reads aerodynamic coefficients and builds interpolators., Converts the raw number list into SciPy interpolation objects., Reads engine and mass data and builds interpolators., SRAAM6_Database, Euler, _integrate(), Rotational equations of motion for an axisymmetric missile.          Euler equat, Trapezoidal integration (CADAC standard). (+11 more)

### Community 1 - "Core Physics & Control Loop"
Cohesion: 0.12
Nodes (35): Actuator, Actuator.actuator_scnd, Aerodynamics, Aerodynamics.aerodynamics_der, config module, Control, Control.control_accel, Control.control_indi (+27 more)

### Community 2 - "Simulation Runner & Monte Carlo"
Cohesion: 0.11
Nodes (24): compare(), monte_carlo(), monte_carlo_all_scenarios(), _print_mc_table(), _print_summary_table(), ================================================================================, Run one scenario. `cfg` optionally overrides any global knob:         mins, ins_, Save log to logs/sim_log_<tag>.csv. (+16 more)

### Community 3 - "Autopilot Modes (NDI/INDI/Accel)"
Cohesion: 0.16
Nodes (11): Control, _integrate(), Calculates roll gains and commanded roll fin deflection., Calculates rate gyro feedback gain and pitch/yaw rate commands., Trapezoidal integration (CADAC standard)., Pole-placement acceleration controller for pitch and yaw planes., Nonlinear Dynamic Inversion style accel autopilot.             Outer loop: accel, Incremental Nonlinear Dynamic Inversion autopilot.          Same outer/middle lo (+3 more)

### Community 4 - "Guidance Laws (ProNav/Line)"
Cohesion: 0.17
Nodes (10): Guidance, mat2tr(), pol_from_cart(), Cartesian → spherical (CADAC convention).      Returns [range, azimuth, elevatio, Pro-nav from third-party kinematic data.          Args:             STBLC: targe, Line guidance: steer missile along the target-aircraft line.          Args:, 2-axis rotation DCM (azimuth then elevation), no roll.      Equivalent to mat3tr, Terminal compensated pro-nav (requires seeker data).          Reads from missile (+2 more)

### Community 5 - "INS Error Model"
Cohesion: 0.23
Nodes (8): INS, _integrate_vec(), INS (Inertial Navigation System) — SRAAM6 port of CADAC ins.cpp.  Ported with SR, Returns (EWBEB, WBECB_meas). EWBEB = measurement error - rad/s., Returns accelerometer measurement error EFSPB (random walk applied later)., Trapezoidal integration — matches the CADAC convention used elsewhere., Seed estimate channels with truth at t=0 (perfect alignment)., _skew()

### Community 6 - "Kinematic Seeker"
Cohesion: 0.21
Nodes (8): _mat2tr(), _pol_from_cart(), Kinematic (ideal) seeker — SRAAM6 port of CADAC sensor.cpp (sensor_ir_kin).  Pha, Ideal seeker: LOS angles + inertial LOS rates from geometry.          CADAC sens, Cartesian → (range, azimuth, elevation) in CADAC convention., 2-axis rotation DCM (az then el, no roll). Same as guidance.mat2tr., Zero the seeker output channels on the missile object., Sensor

### Community 7 - "Quaternion Kinematics"
Cohesion: 0.23
Nodes (8): _integrate(), Kinematics, _mat3tr(), Integrates quaternion ODEs; updates DCM, Euler angles, and incidence angles., Trapezoidal integration (CADAC standard)., Direction cosine matrix body-wrt-local for 3-2-1 (Z-Y-X) Euler sequence.      Ar, Set quaternion and TBL from missile initial Euler angles (psix/thtx/phix)., _sign()

### Community 8 - "Target Scenario Dynamics"
Cohesion: 0.22
Nodes (6): _cw_perp(), Target scenarios for SRAAM6 simulation.  NED frame [North, East, Down],  hbe = -, Return (pos, vel) at simulation time t.          S1 uses the exact analytic (con, Advance internal state by dt using a first-order Euler step., Unit vector 90° clockwise from the horizontal velocity component.      For vel h, Target

### Community 9 - "Fin Actuator Dynamics"
Cohesion: 0.31
Nodes (5): Actuator, Second order actuator dynamics (Euler integration)., Returns +1 for positive, -1 for negative, 0 for zero., Converts autopilot commands to fin deflections., _sign()

### Community 10 - "Aerodynamic Coefficients"
Cohesion: 0.33
Nodes (3): Aerodynamics, On-line calculation of aero derivatives for the aero-adaptive autopilot., Calculates aerodynamic coefficients from look-up tables.

### Community 11 - "Atmosphere & Environment"
Cohesion: 0.33
Nodes (4): atmosphere76(), Environment, US 1976 Standard Atmosphere.      Args:         hbe: geometric altitude above MS, Computes atmosphere, gravity, Mach number, and dynamic pressure.          Reads

## Knowledge Gaps
- **72 isolated node(s):** `Kinematic (ideal) seeker — SRAAM6 port of CADAC sensor.cpp (sensor_ir_kin).  Pha`, `Cartesian → (range, azimuth, elevation) in CADAC convention.`, `2-axis rotation DCM (az then el, no roll). Same as guidance.mat2tr.`, `Zero the seeker output channels on the missile object.`, `Ideal seeker: LOS angles + inertial LOS rates from geometry.          CADAC sens` (+67 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Missile` connect `Aero/Prop Lookup Database` to `Simulation Runner & Monte Carlo`, `Autopilot Modes (NDI/INDI/Accel)`, `Guidance Laws (ProNav/Line)`, `INS Error Model`, `Kinematic Seeker`, `Quaternion Kinematics`, `Fin Actuator Dynamics`, `Aerodynamic Coefficients`, `Atmosphere & Environment`?**
  _High betweenness centrality (0.404) - this node is a cross-community bridge._
- **Why does `run()` connect `Simulation Runner & Monte Carlo` to `Target Scenario Dynamics`, `Aero/Prop Lookup Database`?**
  _High betweenness centrality (0.238) - this node is a cross-community bridge._
- **Why does `Control` connect `Autopilot Modes (NDI/INDI/Accel)` to `Aero/Prop Lookup Database`?**
  _High betweenness centrality (0.115) - this node is a cross-community bridge._
- **Are the 14 inferred relationships involving `Missile` (e.g. with `SRAAM6_Database` and `Environment`) actually correct?**
  _`Missile` has 14 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Kinematic (ideal) seeker — SRAAM6 port of CADAC sensor.cpp (sensor_ir_kin).  Pha`, `Cartesian → (range, azimuth, elevation) in CADAC convention.`, `2-axis rotation DCM (az then el, no roll). Same as guidance.mat2tr.` to the rest of the system?**
  _72 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Aero/Prop Lookup Database` be split into smaller, more focused modules?**
  _Cohesion score 0.07 - nodes in this community are weakly interconnected._
- **Should `Core Physics & Control Loop` be split into smaller, more focused modules?**
  _Cohesion score 0.12 - nodes in this community are weakly interconnected._