"""
=============================================================================
MEDICAL VENTILATOR COMPRESSOR SYSTEM — LUMPED-PARAMETER SIMULATION
=============================================================================
System: G&M Tech 100RND-ED (Double Head, 24VDC) oil-free reciprocating pump
        + 2L SS storage tank with copper cooling coil
        + SMC filtration train (Open Drain, Water Sep, Mist/Oil Sep)
        + 2x 24V centrifugal blowers (PWM controlled)
        + HEPA intake filter + stainless steel silencer chamber

Author  : Generated for Ayush — Medical Device Systems Engineer
Model   : Tier 1 (Steady-State) + Tier 2 (Dynamic Transient) + Fault Injection
Standard: Physics consistent with ISO 6358 (orifice flow), ISO 13443 (air),
          IEC 60601-1 (pressure limits), ISO 14971 (risk scenarios)

STRUCTURE
---------
1.  Physical constants & helpers
2.  Component models (each component as a class)
     2a. HEPAFilter
     2b. SilencerChamber
     2c. FlexibleHose
     2d. ReciprocatingPump
     2e. CoolingCoil (on tank)
     2f. CoolingFan (x2)
     2g. ThermalModel (pump heads + cabinet)
     2h. NonReturnValve
     2i. StorageTank
     2j. OpenDrainFilter
     2k. WaterSeparator
     2l. MistOilSeparator
     2m. PressureSensor (with drift model)
     2n. PressureReliefValve
3.  System class — connects all components in series
4.  Dynamic simulator — time-domain ODE solver
5.  Fault injection engine
6.  Interactive matplotlib dashboard
=============================================================================
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button, CheckButtons, RadioButtons
from matplotlib.patches import FancyArrowPatch
import warnings
warnings.filterwarnings('ignore')

# ─────────────────────────────────────────────────────────────────────────────
# 1. PHYSICAL CONSTANTS & HELPERS
# ─────────────────────────────────────────────────────────────────────────────

R_AIR       = 287.05        # J/(kg·K)  specific gas constant for air
GAMMA       = 1.4           # —         ratio of specific heats
CP_AIR      = 1005.0        # J/(kg·K)  specific heat at constant pressure
RHO_ANR     = 1.204         # kg/m³     air density at ANR (0°C, 101325 Pa)
P_ATM       = 101325.0      # Pa        atmospheric pressure
T_ANR       = 293.15        # K         ANR temperature (20°C)
T_REF       = 293.15        # K
CHOKED_RATIO = (2/(GAMMA+1))**(GAMMA/(GAMMA-1))   # 0.5283 for air

def abs_pressure(gauge_bar):
    """Convert gauge pressure in bar to absolute pressure in Pa."""
    return gauge_bar * 1e5 + P_ATM

def gauge_bar(abs_pa):
    """Convert absolute pressure Pa to gauge bar."""
    return (abs_pa - P_ATM) / 1e5

def free_air_lpm(mass_flow_kgs):
    """Convert kg/s to standard litres per minute (ANR)."""
    return mass_flow_kgs / RHO_ANR * 1000 * 60

def mass_flow_from_lpm(lpm):
    """Convert ANR L/min to kg/s."""
    return lpm * RHO_ANR / (1000 * 60)

def orifice_mass_flow(P1_pa, P2_pa, T1_K, d_m, Cd=0.65):
    """
    Compressible orifice mass flow (ISO 6358).
    Handles both choked and unchoked conditions.
    Returns mass flow in kg/s.
    """
    if P1_pa <= P2_pa:
        return 0.0
    A = np.pi * d_m**2 / 4
    ratio = P2_pa / P1_pa
    if ratio <= CHOKED_RATIO:
        # Choked (sonic) flow
        choked_term = (2/(GAMMA+1))**((GAMMA+1)/(GAMMA-1))
        mdot = Cd * A * P1_pa * np.sqrt(GAMMA / (R_AIR * T1_K) * choked_term)
    else:
        # Unchoked (subsonic) flow
        term = (ratio**(2/GAMMA) - ratio**((GAMMA+1)/GAMMA))
        mdot = Cd * A * np.sqrt(2*GAMMA/(GAMMA-1) * P1_pa/
               (R_AIR*T1_K) * (P1_pa/T1_K) * term)
        # Simplified: use Bernoulli-compressible form
        rho1 = P1_pa / (R_AIR * T1_K)
        dP   = P1_pa - P2_pa
        mdot = Cd * A * np.sqrt(2 * rho1 * dP)
    return max(mdot, 0.0)

def polytropic_temp(T1_K, P1_pa, P2_pa, n=1.35):
    """
    Outlet temperature after polytropic compression.
    n=1.35 for oil-free reciprocating pump (between isothermal 1.0 and adiabatic 1.4)
    """
    return T1_K * (P2_pa / P1_pa)**((n-1)/n)

# ─────────────────────────────────────────────────────────────────────────────
# 2. COMPONENT MODELS
# ─────────────────────────────────────────────────────────────────────────────

class HEPAFilter:
    """
    Intake HEPA filter inside the SS silencer chamber.
    Models: pressure drop vs flow, loading state (clog fraction 0→1),
            glass fiber integrity (fracture threshold).

    Physics: ΔP = K_clean * Q * (1 + clog_factor * loading)
    Darcy-Forchheimer for fibrous media: ΔP = (µ*α/thickness)*v + β*ρ*v²
    Simplified to linear for lumped model.
    """
    def __init__(self):
        self.name            = "HEPA Filter"
        # Geometry
        self.face_area_m2    = 0.008    # m²  (approx 100x80mm element)
        self.thickness_m     = 0.02     # m
        # Clean filter resistance coefficient (Pa per m³/s)
        self.K_clean         = 8500     # Pa·s/m³  (typical HEPA at rated flow)
        # State
        self.loading         = 0.0     # 0=new, 1=fully clogged
        self.clog_rate       = 5e-7    # loading units per kg of air passed
        self.is_fractured    = False
        # Fracture threshold: peak suction pulse pressure (Pa gauge)
        self.fracture_threshold_pa = 4000   # 40 mbar peak suction

    def pressure_drop(self, Q_m3s, T_K=293.15):
        """Returns ΔP in Pa for given volumetric flow at inlet conditions."""
        if self.is_fractured:
            return 0.0      # fractured = no resistance (and no filtration)
        v = Q_m3s / self.face_area_m2   # face velocity m/s
        rho = P_ATM / (R_AIR * T_K)
        # Darcy term + Forchheimer (inertial) term
        dP_darcy = self.K_clean * (1 + 8 * self.loading) * Q_m3s
        dP_inertial = 0.5 * rho * v**2 * 1.2   # minor inertial term
        return dP_darcy + dP_inertial

    def update_loading(self, mass_flow_kgs, dt_s):
        """Advance loading state — called each timestep."""
        dm = mass_flow_kgs * dt_s
        self.loading = min(1.0, self.loading + self.clog_rate * dm)

    def check_fracture(self, peak_suction_pa):
        """Check if cyclic suction pulse exceeds fracture threshold."""
        if peak_suction_pa > self.fracture_threshold_pa:
            self.is_fractured = True

    @property
    def is_clogged(self):
        return self.loading > 0.85


class SilencerChamber:
    """
    Welded SS silencer chamber. Models volume buffering and minor flow resistance.
    Also models weld fatigue cycle count.
    """
    def __init__(self):
        self.name           = "SS Silencer Chamber"
        self.volume_m3      = 0.0005        # 0.5 L internal volume
        self.K_flow         = 200           # Pa·s/m³ minor loss through ports
        self.fatigue_cycles = 0
        self.weld_life_cycles = 50e6        # ~50 million cycles to crack

    def pressure_drop(self, Q_m3s):
        return self.K_flow * Q_m3s

    def update_fatigue(self, n_cycles):
        self.fatigue_cycles += n_cycles

    @property
    def weld_integrity(self):
        """0=new, 1=at fatigue life limit."""
        return min(1.0, self.fatigue_cycles / self.weld_life_cycles)


class FlexibleHose:
    """
    Flexible suction/discharge hoses.
    Models: minor loss, collapse risk (suction side only).
    Collapse occurs if suction ΔP exceeds wall stiffness threshold.
    """
    def __init__(self, length_m=0.15, ID_m=0.012, hose_type='suction'):
        self.name       = f"Flexible Hose ({hose_type})"
        self.L          = length_m
        self.D          = ID_m
        self.A          = np.pi * ID_m**2 / 4
        self.type       = hose_type
        self.is_collapsed = False
        # Wall stiffness: min gauge suction before collapse
        self.collapse_threshold_pa = 3000 if hose_type == 'suction' else 1e9

    def pressure_drop(self, Q_m3s, rho=1.2):
        """Darcy-Weisbach pressure drop. Pa."""
        if self.is_collapsed:
            return 1e6      # effectively blocked
        if Q_m3s < 1e-9:
            return 0.0
        v   = Q_m3s / self.A
        f   = 0.02          # Darcy friction factor (turbulent estimate)
        dP  = f * (self.L / self.D) * 0.5 * rho * v**2
        # Minor losses at fittings (entry + exit + bend)
        K_minor = 1.5
        dP += K_minor * 0.5 * rho * v**2
        return dP

    def check_collapse(self, suction_dp_pa):
        if self.type == 'suction' and suction_dp_pa > self.collapse_threshold_pa:
            self.is_collapsed = True


class ReciprocatingPump:
    """
    G&M Tech 100RND-ED double-head oil-free reciprocating pump.
    Rated: 24VDC, ~1600 RPM, ~20 L/min free air, max 8 bar (we run to 2.5 bar).

    Models:
    - Volumetric efficiency vs pressure ratio
    - Polytropic compression & outlet temperature
    - Motor electrical model (current draw vs load)
    - Piston ring wear (progressive blow-by)
    - Reed valve state (intact / fractured)
    - Motor winding temperature
    """
    def __init__(self):
        self.name               = "G&M 100RND-ED Reciprocating Pump"
        # Mechanical
        self.n_heads            = 2
        self.RPM_rated          = 1600
        self.displacement_L     = 0.0185    # L per head per revolution
        self.total_disp_m3_rev  = self.n_heads * self.displacement_L * 1e-3
        # Motor electrical
        self.V_rated            = 24.0      # V
        self.I_noload           = 1.2       # A
        self.I_rated            = 4.5       # A  at 2.5 bar
        self.I_stall            = 22.0      # A  locked rotor
        self.R_winding          = 0.8       # Ω
        self.thermal_resistance = 2.5       # °C/W  winding to ambient
        self.thermal_mass       = 180.0     # J/°C  winding thermal mass
        # State
        self.PWM                = 1.0       # 0–1 duty cycle
        self.RPM                = 0.0
        self.ring_wear          = 0.0       # 0=new, 1=worn out
        self.ring_wear_rate     = 2e-9      # wear per joule of heat energy
        self.reed_valve_ok      = [True, True]   # [head1, head2]
        self.T_winding          = 25.0      # °C
        self.T_head             = [25.0, 25.0]   # °C each head
        self.hours_run          = 0.0
        # Polytropic index
        self.n_poly             = 1.35

    @property
    def RPM_actual(self):
        """RPM as function of PWM and back-pressure loading."""
        return self.RPM_rated * self.PWM * (1 - 0.05)  # 5% slip under load

    def volumetric_efficiency(self, P_inlet_pa, P_outlet_pa):
        """
        Volumetric efficiency decreases with pressure ratio.
        Also decreases with ring wear (blow-by).
        η_vol = η0 - k*(Pr - 1) - blow_by_fraction
        """
        Pr = P_outlet_pa / P_inlet_pa
        eta0 = 0.92
        eta = eta0 - 0.028 * (Pr - 1) - 0.30 * self.ring_wear
        # If a reed valve is fractured, that head contributes nothing
        active_heads = sum(self.reed_valve_ok)
        eta *= active_heads / self.n_heads
        return max(0.0, min(eta, 1.0))

    def mass_flow(self, P_inlet_pa, P_outlet_pa, T_inlet_K):
        """
        Delivered mass flow rate kg/s.
        """
        if self.PWM < 0.10:     # below minimum PWM threshold — pump stalls
            return 0.0
        RPM = self.RPM_actual
        eta_vol = self.volumetric_efficiency(P_inlet_pa, P_outlet_pa)
        # Theoretical swept volume flow rate
        Q_swept = (RPM / 60) * self.total_disp_m3_rev   # m³/s at inlet conditions
        Q_actual = eta_vol * Q_swept                      # m³/s actual
        # Convert to mass flow using inlet air density
        rho_inlet = P_inlet_pa / (R_AIR * T_inlet_K)
        mdot = Q_actual * rho_inlet
        return max(0.0, mdot)

    def outlet_temperature(self, T_inlet_K, P_inlet_pa, P_outlet_pa):
        """Polytropic compression outlet temperature."""
        return polytropic_temp(T_inlet_K, P_inlet_pa, P_outlet_pa, self.n_poly)

    def current_draw(self, P_outlet_pa):
        """
        Motor current draw vs outlet pressure (load).
        Linear approximation between no-load and rated.
        """
        if self.PWM < 0.10:
            return 0.0
        P_gauge = gauge_bar(P_outlet_pa)
        I = self.I_noload + (self.I_rated - self.I_noload) * (P_gauge / 2.5)
        return I * self.PWM

    def power_input(self, P_outlet_pa):
        return self.V_rated * self.current_draw(P_outlet_pa)

    def update_thermal(self, P_inlet_pa, P_outlet_pa, T_cooling_air_K, dt_s):
        """
        Update winding and head temperatures.
        Q_in = I²R + mechanical losses
        Q_out = convection to cooling air (forced convection from fans)
        """
        I = self.current_draw(P_outlet_pa)
        Q_joule = I**2 * self.R_winding                 # W winding heat
        Q_mech  = self.power_input(P_outlet_pa) * 0.15  # 15% mech losses as heat
        Q_total = Q_joule + Q_mech
        # Heat rejection to cooling air (h*A*(T_head - T_air))
        # Fan cooling: h_eff estimated from fan curve
        h_eff_A = 8.0   # W/K  effective heat transfer coefficient × area
        T_air = T_cooling_air_K - 273.15
        dT_winding = (Q_total - h_eff_A * (self.T_winding - T_air)) / self.thermal_mass * dt_s
        self.T_winding += dT_winding
        # Head temperature follows winding with lag
        for i in range(self.n_heads):
            if self.reed_valve_ok[i]:
                # Compression heat raises head temperature
                T_outlet = self.outlet_temperature(T_inlet_K=T_cooling_air_K,
                                                    P_inlet_pa=P_inlet_pa,
                                                    P_outlet_pa=P_outlet_pa)
                T_head_target = (T_outlet - 273.15) * 0.6 + self.T_winding * 0.4
                tau_head = 30.0     # s  thermal time constant of head
                self.T_head[i] += (T_head_target - self.T_head[i]) / tau_head * dt_s

        # Ring wear rate proportional to head temperature above 60°C
        excess_T = max(0, max(self.T_head) - 60)
        self.ring_wear += self.ring_wear_rate * Q_total * (1 + excess_T/40) * dt_s
        self.ring_wear = min(1.0, self.ring_wear)

    @property
    def T_head_max(self):
        return max(self.T_head)


class CoolingFan:
    """
    24VDC 4-wire centrifugal blower with PWM speed control.
    Models: flow vs speed curve, FG tachometer, minimum PWM threshold,
            bearing wear, stall detection.
    """
    def __init__(self, fan_id=1):
        self.name           = f"Centrifugal Blower #{fan_id}"
        self.RPM_max        = 3200
        self.flow_max_m3s   = 0.025    # m³/s at max speed (~25 L/s = 1500 L/min)
        self.PWM_min        = 0.15     # below this → stall
        self.PWM            = 1.0
        self.is_seized      = False
        self.bearing_wear   = 0.0      # 0=new, 1=seized
        self.bearing_wear_rate = 3e-8  # per second at rated speed
        self.FG_signal      = True     # tachometer wire intact

    @property
    def RPM(self):
        if self.is_seized or self.PWM < self.PWM_min:
            return 0.0
        return self.RPM_max * self.PWM

    @property
    def flow_m3s(self):
        """Volumetric airflow delivered."""
        if self.is_seized or self.PWM < self.PWM_min:
            return 0.0
        # Fan affinity law: Q ∝ N
        return self.flow_max_m3s * (self.PWM)

    @property
    def is_stalled(self):
        return self.RPM == 0.0 and self.PWM > self.PWM_min

    def update(self, dt_s):
        if not self.is_seized:
            self.bearing_wear += self.bearing_wear_rate * self.PWM * dt_s
            if self.bearing_wear >= 1.0:
                self.is_seized = True


class ThermalModel:
    """
    Cabinet thermal model.
    Tracks:
    - Ambient temperature (input)
    - Cooling air temperature at pump inlet (after passing over hot tank+coil)
    - Cabinet internal temperature
    Physics: energy balance on the air stream
    """
    def __init__(self):
        self.name           = "Cabinet Thermal Model"
        self.T_ambient_C    = 25.0       # °C  room temperature
        self.T_cabinet_C    = 25.0       # °C  internal air
        self.T_coil_surface_C = 25.0     # °C  tank+coil outer surface
        self.thermal_mass_cabinet = 500  # J/°C  thermal mass of cabinet air + structure
        # Coil heat exchange parameters
        self.UA_coil        = 12.0       # W/K  overall heat transfer coeff × area
        # Pump head radiation to cabinet
        self.UA_head_cabinet = 4.0       # W/K

    def cooling_air_temp_K(self, fan_flow_m3s, Q_coil_W):
        """
        Temperature of air arriving at pump heads after passing over tank-coil.
        Energy balance: Q_coil = mdot_air * Cp * (T_out - T_ambient)
        → T_out = T_ambient + Q_coil / (mdot * Cp)
        """
        if fan_flow_m3s < 1e-6:
            # No airflow: air stagnates, reaches thermal equilibrium
            return (self.T_coil_surface_C + 10 + 273.15)  # stagnant air temp
        rho_air = 1.1      # kg/m³ slightly warm
        mdot_air = fan_flow_m3s * rho_air
        delta_T = Q_coil_W / (mdot_air * CP_AIR + 1e-9)
        T_out_C = self.T_ambient_C + delta_T
        return (T_out_C + 273.15)

    def update_coil_temp(self, T_compressed_air_K, fan_flow_m3s, dt_s):
        """Update coil surface temperature."""
        # Heat input from hot compressed air passing through coil
        if fan_flow_m3s > 0:
            rho = 1.1
            mdot = fan_flow_m3s * rho
            Q_removed = self.UA_coil * (T_compressed_air_K - 273.15 - self.T_coil_surface_C)
        else:
            Q_removed = 0.0
        dT = Q_removed / (self.thermal_mass_cabinet) * dt_s
        self.T_coil_surface_C = self.T_coil_surface_C + dT * 0.1
        self.T_coil_surface_C = min(self.T_coil_surface_C,
                                     T_compressed_air_K - 273.15 - 5)


class NonReturnValve:
    """
    NRV (check valve) model.
    Models: cracking pressure, flow resistance, stuck-open fault.
    """
    def __init__(self, cracking_pressure_pa=1500, Cv=0.8):
        self.name               = "Non-Return Valve"
        self.P_crack            = cracking_pressure_pa
        self.Cv                 = Cv        # flow coefficient
        self.is_stuck_open      = False
        self.is_stuck_closed    = False
        self.fragment_lodged    = False

    def pressure_drop(self, P_upstream_pa, P_downstream_pa, mdot_kgs):
        """Pressure drop across NRV in forward flow. Pa."""
        if self.is_stuck_closed:
            return 1e6
        dP = P_upstream_pa - P_downstream_pa
        if dP < self.P_crack and not self.is_stuck_open:
            return self.P_crack     # valve closed, blocking flow
        # Open: minor loss
        rho = 1.5      # compressed air density estimate
        A   = 1.5e-4   # m²
        v   = mdot_kgs / (rho * A + 1e-9)
        return 0.5 * rho * v**2 / (self.Cv**2)


class StorageTank:
    """
    2L SS storage tank.
    State: pressure (Pa absolute), temperature (K), liquid water volume (m³).
    Physics: ideal gas law for pressure, energy balance for temperature.
    Includes: pressure rise/fall, condensate accumulation, tank weld fatigue.
    """
    def __init__(self):
        self.name           = "2L SS Storage Tank"
        self.volume_m3      = 0.002         # 2 litres
        self.P_pa           = P_ATM         # initial: atmospheric
        self.T_K            = 293.15        # initial: 20°C
        self.mass_air_kg    = (P_ATM * self.volume_m3) / (R_AIR * self.T_K)
        self.water_vol_m3   = 0.0           # condensate
        self.P_max_pa       = abs_pressure(3.5)     # PRV set pressure
        self.P_burst_pa     = abs_pressure(10.0)    # burst pressure (4× rated)
        self.fatigue_cycles = 0             # pressure cycles for weld fatigue
        self.weld_life_cycles = 200000      # ~200k pressure cycles to initiate crack
        self.T_wall_C       = 25.0          # tank wall temperature

    def update(self, mdot_in_kgs, mdot_out_kgs, T_in_K, Q_heat_loss_W, dt_s):
        """
        Update tank pressure and temperature.
        Mass conservation: dm/dt = mdot_in - mdot_out
        Energy: d(m*cv*T)/dt = mdot_in*h_in - mdot_out*h_out - Q_loss
        Using ideal gas: P = m*R*T/V
        """
        # Mass update
        dm = (mdot_in_kgs - mdot_out_kgs) * dt_s
        self.mass_air_kg = max(1e-6, self.mass_air_kg + dm)

        # Energy update (simplified: track temperature)
        cv_air = CP_AIR - R_AIR    # 718 J/(kg·K)
        h_in   = CP_AIR * T_in_K
        h_out  = CP_AIR * self.T_K
        dU     = ((mdot_in_kgs * h_in - mdot_out_kgs * h_out) - Q_heat_loss_W) * dt_s
        U      = cv_air * self.mass_air_kg * self.T_K
        U_new  = max(U + dU, cv_air * self.mass_air_kg * 250)
        self.T_K = U_new / (cv_air * self.mass_air_kg)

        # Pressure from ideal gas law
        P_new = self.mass_air_kg * R_AIR * self.T_K / self.volume_m3
        # Count pressure cycles for fatigue
        if P_new > abs_pressure(2.3) and self.P_pa < abs_pressure(2.3):
            self.fatigue_cycles += 1
        self.P_pa = P_new
        self.T_wall_C = self.T_K - 273.15 - 10     # wall slightly cooler

    @property
    def P_gauge_bar(self):
        return gauge_bar(self.P_pa)

    @property
    def weld_integrity(self):
        return min(1.0, self.fatigue_cycles / self.weld_life_cycles)


class OpenDrainFilter:
    """
    SMC Open Drain Filter — 1 micron.
    Models: pressure drop vs flow, solid loading, open drain bleed flow.
    Key result: drain bleed at 2.5 bar is physically quantified.
    """
    def __init__(self):
        self.name           = "Open Drain Filter (1 µm)"
        self.K_clean        = 1200          # Pa·s/m³
        self.loading        = 0.0           # 0=clean, 1=clogged
        self.clog_rate      = 1e-6          # loading per kg air
        self.drain_d_m      = 0.001         # 1mm orifice
        self.drain_Cd       = 0.65
        self.is_bypassed    = False         # reversed installation fault

    def pressure_drop(self, Q_m3s):
        if self.is_bypassed:
            return 0.0
        return self.K_clean * (1 + 6 * self.loading) * Q_m3s

    def drain_bleed_flow_kgs(self, P_upstream_pa, T_K=313.15):
        """Actual air bleed through open drain orifice — for overpressure analysis."""
        return orifice_mass_flow(P_upstream_pa, P_ATM, T_K,
                                  self.drain_d_m, self.drain_Cd)

    def drain_bleed_lpm(self, P_upstream_pa, T_K=313.15):
        return free_air_lpm(self.drain_bleed_flow_kgs(P_upstream_pa, T_K))

    def update_loading(self, mdot_kgs, dt_s):
        self.loading = min(1.0, self.loading + self.clog_rate * mdot_kgs * dt_s)


class WaterSeparator:
    """
    Cyclone water separator.
    Models: separation efficiency vs flow, bowl liquid level, float drain state.
    """
    def __init__(self):
        self.name           = "Water Separator"
        self.K_flow         = 800
        self.separation_eff = 0.95      # 95% water removal at design flow
        self.bowl_volume_m3 = 50e-6     # 50 ml bowl
        self.water_level    = 0.0       # fraction full 0–1
        self.drain_ok       = True      # float drain working

    def pressure_drop(self, Q_m3s):
        return self.K_flow * Q_m3s

    def update(self, moisture_kgs_per_s, dt_s):
        if self.drain_ok:
            # Drain keeps bowl at low level
            drain_rate = 1e-7   # kg/s condensate drain capacity
            net = moisture_kgs_per_s * self.separation_eff - drain_rate
        else:
            net = moisture_kgs_per_s * self.separation_eff
        delta_vol = net * dt_s / 1000   # water density ~1000 kg/m³
        self.water_level = max(0, min(1.0,
                              self.water_level + delta_vol / self.bowl_volume_m3))

    @property
    def is_overflowing(self):
        return self.water_level >= 1.0


class MistOilSeparator:
    """
    0.3-micron coalescing mist/oil separator.
    Models: saturation state, pressure drop increase with saturation.
    """
    def __init__(self):
        self.name           = "Mist/Oil Separator (0.3 µm)"
        self.K_clean        = 2500
        self.saturation     = 0.0
        self.sat_rate       = 3e-6
        self.is_ruptured    = False

    def pressure_drop(self, Q_m3s):
        if self.is_ruptured:
            return 0.0
        return self.K_clean * (1 + 12 * self.saturation) * Q_m3s

    def update(self, mdot_kgs, dt_s):
        self.saturation = min(1.0, self.saturation + self.sat_rate * mdot_kgs * dt_s)


class PressureSensor:
    """
    Pressure sensor with drift model, port blockage fault, noise model.
    """
    def __init__(self, name="Pressure Sensor", full_scale_bar=6.0):
        self.name           = name
        self.full_scale_pa  = full_scale_bar * 1e5
        self.drift_pa       = 0.0       # Pa zero-point drift
        self.drift_rate     = 0.0       # Pa/hour — set non-zero for drift fault
        self.is_blocked     = False
        self.noise_std_pa   = 150       # Pa RMS noise
        self._frozen_value  = None

    def read(self, true_pressure_pa, dt_s=1.0):
        """Returns sensor output including drift, noise, and faults."""
        if self.is_blocked:
            if self._frozen_value is None:
                self._frozen_value = true_pressure_pa
            return self._frozen_value

        # Advance drift
        self.drift_pa += self.drift_rate * dt_s / 3600

        reading = true_pressure_pa + self.drift_pa
        reading += np.random.normal(0, self.noise_std_pa)
        return reading

    @property
    def drift_bar(self):
        return self.drift_pa / 1e5


class PressureReliefValve:
    """
    PRV — spring-loaded, set at 3.2 bar gauge.
    Models: set pressure, hysteresis, stuck-closed fault.
    """
    def __init__(self, set_pressure_bar=3.2):
        self.name           = "Pressure Relief Valve"
        self.P_set_pa       = abs_pressure(set_pressure_bar)
        self.P_reseat_pa    = abs_pressure(set_pressure_bar - 0.15)  # hysteresis
        self.is_open        = False
        self.is_stuck_closed = False
        self.Cd             = 0.7
        self.orifice_d_m    = 0.006     # 6mm orifice

    def update(self, P_pa):
        if self.is_stuck_closed:
            self.is_open = False
            return
        if P_pa >= self.P_set_pa:
            self.is_open = True
        elif P_pa < self.P_reseat_pa:
            self.is_open = False

    def vent_flow_kgs(self, P_pa, T_K=350.0):
        """Mass flow vented when PRV is open."""
        if not self.is_open:
            return 0.0
        return orifice_mass_flow(P_pa, P_ATM, T_K, self.orifice_d_m, self.Cd)


# ─────────────────────────────────────────────────────────────────────────────
# 3. SYSTEM CLASS — CONNECTS ALL COMPONENTS
# ─────────────────────────────────────────────────────────────────────────────

class VentilatorCompressorSystem:
    """
    Complete system: HEPA → Silencer → Hose → Pump → CoilTank → NRV →
                     OpenDrainFilter → WaterSep → MistSep → Air OUT
    """
    def __init__(self):
        # Instantiate all components
        self.hepa           = HEPAFilter()
        self.silencer       = SilencerChamber()
        self.suction_hose   = FlexibleHose(0.15, 0.012, 'suction')
        self.pump           = ReciprocatingPump()
        self.fan1           = CoolingFan(1)
        self.fan2           = CoolingFan(2)
        self.thermal        = ThermalModel()
        self.discharge_hose = FlexibleHose(0.10, 0.010, 'discharge')
        self.nrv1           = NonReturnValve()      # pump outlet NRV
        self.tank           = StorageTank()
        self.prv            = PressureReliefValve(3.2)
        self.odf            = OpenDrainFilter()     # Open Drain Filter
        self.water_sep      = WaterSeparator()
        self.mist_sep       = MistOilSeparator()
        self.nrv2           = NonReturnValve()      # outlet NRV
        self.sensor_tank    = PressureSensor("Tank Pressure Sensor")
        self.sensor_outlet  = PressureSensor("Outlet Pressure Sensor")

        # Control parameters
        self.P_setpoint_pa  = abs_pressure(2.5)     # firmware target
        self.P_cutoff_sw_pa = abs_pressure(3.0)     # hardware pressure switch
        self.pump_on        = False
        self.hw_switch_ok   = True                  # hardware switch not bypassed

        # Demand model (ventilator breathing demand)
        self.breath_rate_bpm        = 15
        self.tidal_volume_L         = 0.5
        self.I_E_ratio              = 1/2           # inspiration:expiration = 1:2
        self.P_delivery_bar         = 1.5           # pressure at ventilator outlet

        # Accumulated data for plotting
        self.time_log       = []
        self.state_log      = {}
        self._init_log()

    def _init_log(self):
        keys = ['P_tank_bar', 'T_head_max_C', 'T_winding_C',
                'T_cooling_air_C', 'fan1_rpm', 'fan2_rpm',
                'pump_mdot_lpm', 'pump_current_A', 'pump_PWM',
                'hepa_loading', 'hepa_dP_mbar', 'odf_loading',
                'water_level', 'mist_sat', 'ring_wear',
                'sensor_reading_bar', 'drain_bleed_lpm',
                'P_prv_set_bar', 'prv_open', 'weld_integrity_tank',
                'total_dP_filtration_mbar']
        for k in keys:
            self.state_log[k] = []

    def ventilator_demand_flow(self, t_s):
        """
        Sinusoidal breath demand model.
        Returns required mass flow kg/s from the tank at time t.
        """
        breath_period = 60.0 / self.breath_rate_bpm
        t_in_breath = t_s % breath_period
        t_insp = breath_period / (1 + 1/self.I_E_ratio)
        if t_in_breath < t_insp:
            # Inspiration: draw from tank
            tidal_m3 = self.tidal_volume_L * 1e-3
            rho_delivery = abs_pressure(self.P_delivery_bar) / (R_AIR * 310)
            mdot_demand = (tidal_m3 * rho_delivery) / t_insp
        else:
            mdot_demand = 0.0
        return mdot_demand

    def firmware_control(self, dt_s):
        """
        Firmware pressure control loop.
        Uses sensor reading (with possible drift) — not true pressure.
        """
        P_sensed = self.sensor_tank.read(self.tank.P_pa, dt_s)
        P_sensed_bar = gauge_bar(P_sensed)

        # Hardware switch override
        if self.tank.P_pa >= self.P_cutoff_sw_pa and self.hw_switch_ok:
            self.pump.PWM = 0.0
            self.pump_on  = False
            return

        if P_sensed_bar < gauge_bar(self.P_setpoint_pa) - 0.1:
            self.pump.PWM = 1.0
            self.pump_on  = True
        elif P_sensed_bar > gauge_bar(self.P_setpoint_pa):
            self.pump.PWM = 0.0
            self.pump_on  = False

    def step(self, t_s, dt_s):
        """Advance system state by dt_s seconds at time t_s."""

        # ── Fan update
        self.fan1.update(dt_s)
        self.fan2.update(dt_s)
        total_fan_flow = self.fan1.flow_m3s + self.fan2.flow_m3s

        # ── Thermal: cooling air temperature arriving at pump heads
        Q_coil = self.thermal.UA_coil * max(0,
            self.pump.T_head_max - self.thermal.T_ambient_C)
        self.thermal.update_coil_temp(
            T_compressed_air_K=self.tank.T_K,
            fan_flow_m3s=total_fan_flow, dt_s=dt_s)
        T_cooling_air_K = self.thermal.cooling_air_temp_K(total_fan_flow, Q_coil)

        # ── Firmware control
        self.firmware_control(dt_s)

        # ── Pump operation
        P_inlet_pa = P_ATM    # suction side (atmospheric after HEPA losses)
        # Subtract HEPA and silencer losses from effective pump inlet
        # Approximate: pump operates at slightly sub-atmospheric inlet
        Q_pump_m3s_estimate = 20e-3 / 60  # rough volumetric estimate
        hepa_dP = self.hepa.pressure_drop(Q_pump_m3s_estimate)
        sil_dP  = self.silencer.pressure_drop(Q_pump_m3s_estimate)
        hose_dP = self.suction_hose.pressure_drop(Q_pump_m3s_estimate)
        P_pump_inlet_pa = max(P_ATM * 0.5, P_ATM - hepa_dP - sil_dP - hose_dP)

        mdot_pump = self.pump.mass_flow(P_pump_inlet_pa, self.tank.P_pa, T_cooling_air_K)
        T_pump_out_K = self.pump.outlet_temperature(T_cooling_air_K,
                                                      P_pump_inlet_pa, self.tank.P_pa)

        # ── Pump thermal update
        self.pump.update_thermal(P_pump_inlet_pa, self.tank.P_pa,
                                  T_cooling_air_K, dt_s)

        # ── PRV
        self.prv.update(self.tank.P_pa)
        mdot_prv_vent = self.prv.vent_flow_kgs(self.tank.P_pa, self.tank.T_K)

        # ── Ventilator demand
        mdot_demand = self.ventilator_demand_flow(t_s)

        # ── Filtration downstream (pressure drops, not controlling flow here)
        odf_dP   = self.odf.pressure_drop(Q_pump_m3s_estimate)
        wsep_dP  = self.water_sep.pressure_drop(Q_pump_m3s_estimate)
        mist_dP  = self.mist_sep.pressure_drop(Q_pump_m3s_estimate)
        total_filt_dP = odf_dP + wsep_dP + mist_dP

        # ── Open drain bleed at current tank pressure
        drain_bleed = self.odf.drain_bleed_flow_kgs(self.tank.P_pa, self.tank.T_K)

        # ── Tank update
        mdot_out_total = mdot_demand + mdot_prv_vent + drain_bleed
        Q_loss_tank = 10.0  # W natural cooling of tank to ambient
        self.tank.update(mdot_pump, mdot_out_total, T_pump_out_K, Q_loss_tank, dt_s)

        # ── Component state updates
        self.hepa.update_loading(mdot_pump, dt_s)
        self.odf.update_loading(mdot_pump, dt_s)
        self.water_sep.update(mdot_pump * 0.001, dt_s)   # 0.1% moisture fraction
        self.mist_sep.update(mdot_pump, dt_s)

        # ── HEPA fracture check (peak suction during pump stroke)
        peak_suction = hepa_dP * 1.8   # dynamic peak ~80% higher than mean
        self.hepa.check_fracture(peak_suction)

        # ── Pump hours
        if self.pump_on:
            self.pump.hours_run += dt_s / 3600

        # ── Log state
        self.time_log.append(t_s)
        l = self.state_log
        l['P_tank_bar'].append(self.tank.P_gauge_bar)
        l['T_head_max_C'].append(self.pump.T_head_max)
        l['T_winding_C'].append(self.pump.T_winding)
        l['T_cooling_air_C'].append(T_cooling_air_K - 273.15)
        l['fan1_rpm'].append(self.fan1.RPM)
        l['fan2_rpm'].append(self.fan2.RPM)
        l['pump_mdot_lpm'].append(free_air_lpm(mdot_pump))
        l['pump_current_A'].append(self.pump.current_draw(self.tank.P_pa))
        l['pump_PWM'].append(self.pump.PWM)
        l['hepa_loading'].append(self.hepa.loading * 100)
        l['hepa_dP_mbar'].append(hepa_dP / 100)
        l['odf_loading'].append(self.odf.loading * 100)
        l['water_level'].append(self.water_sep.water_level * 100)
        l['mist_sat'].append(self.mist_sep.saturation * 100)
        l['ring_wear'].append(self.pump.ring_wear * 100)
        l['sensor_reading_bar'].append(gauge_bar(
            self.sensor_tank.read(self.tank.P_pa, dt_s)))
        l['drain_bleed_lpm'].append(free_air_lpm(drain_bleed))
        l['P_prv_set_bar'].append(gauge_bar(self.prv.P_set_pa))
        l['prv_open'].append(float(self.prv.is_open))
        l['weld_integrity_tank'].append(self.tank.weld_integrity * 100)
        l['total_dP_filtration_mbar'].append(total_filt_dP / 100)

    def run(self, duration_s, dt_s=0.5, fault_schedule=None):
        """
        Run simulation for duration_s seconds.
        fault_schedule: list of (time_s, fault_fn) tuples
        """
        t = 0.0
        fault_idx = 0
        faults_sorted = sorted(fault_schedule or [], key=lambda x: x[0])

        while t <= duration_s:
            # Apply any scheduled faults
            while fault_idx < len(faults_sorted) and t >= faults_sorted[fault_idx][0]:
                faults_sorted[fault_idx][1](self)
                fault_idx += 1
            self.step(t, dt_s)
            t += dt_s

    def to_arrays(self):
        """Convert logs to numpy arrays for plotting."""
        t = np.array(self.time_log)
        arrays = {k: np.array(v) for k, v in self.state_log.items()}
        return t, arrays


# ─────────────────────────────────────────────────────────────────────────────
# 4. FAULT INJECTION FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────

def fault_fan_failure(sys):
    """Both fans seize."""
    sys.fan1.is_seized = True
    sys.fan2.is_seized = True
    print(f"  [FAULT] Fan seizure injected")

def fault_filter_clog(sys):
    """Accelerate HEPA and ODF clog 50× faster."""
    sys.hepa.clog_rate  *= 50
    sys.odf.clog_rate   *= 50
    print(f"  [FAULT] Accelerated filter clogging injected")

def fault_sensor_drift(sys):
    """Pressure sensor drifts +0.4 bar/hour (reads high — pump under-runs)."""
    sys.sensor_tank.drift_rate = 4000  # Pa/hour = 0.04 bar/hour
    print(f"  [FAULT] Sensor drift injected (+0.04 bar/hr)")

def fault_nrv_failure(sys):
    """NRV1 stuck open — back-flow possible."""
    sys.nrv1.is_stuck_open = True
    print(f"  [FAULT] NRV1 stuck-open injected")

def fault_prv_stuck_closed(sys):
    """PRV stuck closed — no overpressure relief."""
    sys.prv.is_stuck_closed = True
    # Also disable hardware switch to simulate worst-case double fault
    sys.hw_switch_ok = False
    print(f"  [FAULT] PRV stuck closed + hardware switch disabled (double fault)")


# ─────────────────────────────────────────────────────────────────────────────
# 5. STEADY-STATE ANALYSIS — COMPONENT CURVES
# ─────────────────────────────────────────────────────────────────────────────

def compute_steady_state_curves():
    """Generate steady-state performance curves for all components."""
    results = {}

    # ── 5a. Orifice flow vs pressure (drain bleed analysis)
    P_gauges = np.linspace(0, 4.0, 200)
    P_abs    = [abs_pressure(pg) for pg in P_gauges]
    odf_tmp  = OpenDrainFilter()
    drain_lpm = [odf_tmp.drain_bleed_lpm(p) for p in P_abs]

    # Pump output at same pressures for comparison
    pump_tmp = ReciprocatingPump()
    pump_tmp.PWM = 1.0
    pump_lpm = [free_air_lpm(pump_tmp.mass_flow(P_ATM, p, 313.15)) for p in P_abs]
    results['orifice'] = (P_gauges, drain_lpm, pump_lpm)

    # ── 5b. HEPA pressure drop vs flow at different loading states
    Q_range  = np.linspace(0, 40e-3/60, 100)     # 0 to 40 L/min in m³/s
    Q_lpm    = Q_range * 1e3 * 60
    hepa_tmp = HEPAFilter()
    dP_curves = {}
    for loading in [0.0, 0.25, 0.50, 0.75, 1.0]:
        hepa_tmp.loading = loading
        dP_curves[loading] = [hepa_tmp.pressure_drop(q)/100 for q in Q_range]
    results['hepa'] = (Q_lpm, dP_curves)

    # ── 5c. Pump performance curve (flow vs pressure) at different ring wear
    P_out_bar = np.linspace(0.5, 3.5, 100)
    P_out_pa  = [abs_pressure(p) for p in P_out_bar]
    pump_curves = {}
    for wear in [0.0, 0.25, 0.50, 0.75, 1.0]:
        pump_tmp2 = ReciprocatingPump()
        pump_tmp2.PWM = 1.0
        pump_tmp2.ring_wear = wear
        pump_curves[wear] = [free_air_lpm(pump_tmp2.mass_flow(P_ATM, p, 313.15))
                              for p in P_out_pa]
    results['pump_perf'] = (P_out_bar, pump_curves)

    # ── 5d. Filtration train total ΔP vs flow
    Q_lpm_filt = np.linspace(1, 40, 100)
    odf2  = OpenDrainFilter()
    ws2   = WaterSeparator()
    ms2   = MistOilSeparator()
    dP_clean = []
    dP_half  = []
    dP_clog  = []
    for ql in Q_lpm_filt:
        q = ql / (1000 * 60)
        odf2.loading = 0.0;  ms2.saturation = 0.0
        dP_clean.append((odf2.pressure_drop(q)+ws2.pressure_drop(q)+ms2.pressure_drop(q))/100)
        odf2.loading = 0.5;  ms2.saturation = 0.5
        dP_half.append((odf2.pressure_drop(q)+ws2.pressure_drop(q)+ms2.pressure_drop(q))/100)
        odf2.loading = 0.9;  ms2.saturation = 0.9
        dP_clog.append((odf2.pressure_drop(q)+ws2.pressure_drop(q)+ms2.pressure_drop(q))/100)
    results['filtration'] = (Q_lpm_filt, dP_clean, dP_half, dP_clog)

    # ── 5e. Polytropic compression temperature rise vs pressure ratio
    PR = np.linspace(1.0, 4.0, 100)
    T_out_35C = [(polytropic_temp(308.15, P_ATM, P_ATM*pr) - 273.15) for pr in PR]
    T_out_25C = [(polytropic_temp(298.15, P_ATM, P_ATM*pr) - 273.15) for pr in PR]
    results['compression_temp'] = (PR, T_out_35C, T_out_25C)

    # ── 5f. Tank fill time vs pump flow rate and tank volume
    volumes = [1.0, 2.0, 3.0, 5.0]   # litres
    flow_lpm = np.linspace(5, 35, 100)
    fill_times = {}
    for vol in volumes:
        # Simple fill: P final = P_atm * (1 + mass_added/(rho0*V))
        # time = (P_final - P_initial) * V / (Q_pump * P_atm) in seconds
        fill_times[vol] = [((2.5 * 1e5) * vol * 1e-3) / (q/60 * 1e-3 * P_ATM) * 60
                            for q in flow_lpm]
    results['fill_time'] = (flow_lpm, fill_times)

    return results


# ─────────────────────────────────────────────────────────────────────────────
# 6. SIMULATION RUNNER — RUN ALL SCENARIOS
# ─────────────────────────────────────────────────────────────────────────────

def run_all_simulations(duration_s=300, dt_s=0.5, T_ambient_C=25.0,
                         pump_PWM_init=1.0, fan_PWM=1.0,
                         breath_rate=15, tidal_vol_L=0.5,
                         hepa_init_loading=0.0):
    """Run baseline + all 5 fault scenarios. Return dict of results."""

    scenarios = {
        'Baseline (Normal Operation)': [],
        'Fan Failure (Thermal Runaway)': [(60, fault_fan_failure)],
        'Filter Clogging Progression': [(30, fault_filter_clog)],
        'Sensor Drift (Firmware Blind)': [(45, fault_sensor_drift)],
        'NRV Failure (Back-flow)': [(90, fault_nrv_failure)],
        'PRV Stuck + HW Switch Disabled': [(50, fault_prv_stuck_closed)],
    }

    all_results = {}
    for name, faults in scenarios.items():
        print(f"\n  Running: {name}")
        sys = VentilatorCompressorSystem()
        sys.thermal.T_ambient_C = T_ambient_C
        sys.pump.PWM = pump_PWM_init
        sys.fan1.PWM = fan_PWM
        sys.fan2.PWM = fan_PWM
        sys.breath_rate_bpm = breath_rate
        sys.tidal_volume_L  = tidal_vol_L
        sys.hepa.loading    = hepa_init_loading
        sys.run(duration_s, dt_s, faults)
        t, arrays = sys.to_arrays()
        all_results[name] = (t, arrays, sys)

    return all_results


# ─────────────────────────────────────────────────────────────────────────────
# 7. INTERACTIVE DASHBOARD
# ─────────────────────────────────────────────────────────────────────────────

class Dashboard:
    def __init__(self):
        self.duration_s    = 300
        self.dt_s          = 0.5
        self.T_ambient_C   = 25.0
        self.fan_PWM       = 1.0
        self.breath_rate   = 15
        self.tidal_vol_L   = 0.5
        self.hepa_loading  = 0.0
        self.results       = None
        self.ss_results    = None
        self.active_scenario = 'Baseline (Normal Operation)'
        self._build()

    def _build(self):
        plt.rcParams.update({
            'font.family': 'DejaVu Sans',
            'font.size': 9,
            'axes.titlesize': 10,
            'axes.labelsize': 9,
            'axes.spines.top': False,
            'axes.spines.right': False,
            'lines.linewidth': 1.8,
            'grid.alpha': 0.3,
            'grid.linestyle': '--',
        })

        self.fig = plt.figure(figsize=(22, 14), facecolor='#F8F9FA')
        self.fig.canvas.manager.set_window_title(
            'Medical Ventilator Compressor — System Simulation Dashboard')

        # ── TITLE BAR
        self.fig.text(0.01, 0.975,
            'MEDICAL VENTILATOR COMPRESSOR SYSTEM — DYNAMIC SIMULATION',
            fontsize=13, fontweight='bold', color='#1A1A2E', va='top')
        self.fig.text(0.01, 0.958,
            'G&M Tech 100RND-ED · 24VDC · 2L SS Tank · Copper Cooling Coil · SMC Filtration Train',
            fontsize=9, color='#555555', va='top')

        # ── LAYOUT: left controls, 3×4 main plots, bottom steady-state
        self.gs_main = gridspec.GridSpec(
            4, 5, left=0.17, right=0.98, top=0.94, bottom=0.28,
            hspace=0.55, wspace=0.38)

        self.gs_ss = gridspec.GridSpec(
            1, 5, left=0.17, right=0.98, top=0.24, bottom=0.03,
            hspace=0.4, wspace=0.4)

        self.gs_ctrl = gridspec.GridSpec(
            1, 1, left=0.01, right=0.155, top=0.94, bottom=0.03)

        # ── CONTROL PANEL
        self._build_controls()

        # ── MAIN PLOT AXES
        self.ax = {}
        titles = [
            ('P_tank',    0, 0, 'Tank Pressure (bar gauge)'),
            ('T_head',    0, 1, 'Pump Head Temperature (°C)'),
            ('T_cool',    0, 2, 'Cooling Air Temp (°C)'),
            ('fan_rpm',   0, 3, 'Fan Speed (RPM)'),
            ('pump_flow', 0, 4, 'Pump Delivery Flow (L/min)'),
            ('current',   1, 0, 'Motor Current Draw (A)'),
            ('hepa',      1, 1, 'HEPA Filter Loading (%)'),
            ('odf',       1, 2, 'ODF Loading & Water Level (%)'),
            ('mist',      1, 3, 'Mist Sep Saturation (%)'),
            ('ring',      1, 4, 'Piston Ring Wear (%)'),
            ('sensor',    2, 0, 'Pressure Sensor: True vs Reading (bar)'),
            ('drain',     2, 1, 'Open Drain Bleed Flow (L/min)'),
            ('prv',       2, 2, 'PRV Status & Tank Pressure (bar)'),
            ('filt_dP',   2, 3, 'Filtration Train ΔP (mbar)'),
            ('weld',      2, 4, 'Tank Weld Fatigue (%)'),
            ('thermal_stk',3,0, 'Thermal Stacking Effect (°C)'),
            ('pump_eff',  3, 1, 'Effective Pump Efficiency (%)'),
            ('risk',      3, 2, 'Multi-fault Risk Index'),
            ('energy',    3, 3, 'Cumulative Energy (kJ)'),
            ('hours',     3, 4, 'Pump Hours / Duty Cycle (%)'),
        ]
        for key, row, col, title in titles:
            ax = self.fig.add_subplot(self.gs_main[row, col])
            ax.set_title(title, pad=4, fontsize=9)
            ax.grid(True)
            ax.set_facecolor('#FAFAFA')
            self.ax[key] = ax

        # ── STEADY-STATE AXES
        ss_titles = ['Orifice Drain Flow vs Pump Output',
                     'HEPA ΔP vs Flow (loading states)',
                     'Pump Performance Curve (ring wear)',
                     'Compression Temperature Rise',
                     'Filtration Train ΔP vs Flow']
        self.ax_ss = []
        for i, t in enumerate(ss_titles):
            ax = self.fig.add_subplot(self.gs_ss[0, i])
            ax.set_title(t, pad=4, fontsize=9, color='#333333')
            ax.grid(True)
            ax.set_facecolor('#F5F5F5')
            self.ax_ss.append(ax)

        # Section label
        self.fig.text(0.01, 0.255, 'STEADY-STATE\nCOMPONENT\nCURVES',
                      fontsize=8, color='#555555', va='top',
                      ha='left', linespacing=1.6)

        self.fig.text(0.01, 0.945, 'DYNAMIC\nSIMULATION\nPLOTS',
                      fontsize=8, color='#555555', va='top',
                      ha='left', linespacing=1.6)

    def _build_controls(self):
        """Build slider panel on the left."""
        ctrl_ax = self.fig.add_axes([0.01, 0.03, 0.14, 0.88])
        ctrl_ax.set_facecolor('#EFEFEF')
        ctrl_ax.set_xticks([]); ctrl_ax.set_yticks([])
        for spine in ctrl_ax.spines.values():
            spine.set_edgecolor('#CCCCCC')
        ctrl_ax.text(0.5, 0.97, 'PARAMETERS', ha='center', va='top',
                     transform=ctrl_ax.transAxes, fontsize=9, fontweight='bold',
                     color='#1A1A2E')

        slider_specs = [
            ('duration',    'Duration (s)',       60, 600, 300,  0.86),
            ('T_ambient',   'Ambient Temp (°C)',  15,  45,  25,  0.78),
            ('fan_PWM',     'Fan PWM (%)',          0, 100, 100,  0.70),
            ('breath_rate', 'Breath Rate (bpm)',   8,  30,  15,  0.62),
            ('tidal_vol',   'Tidal Volume (mL)',  200,800, 500,  0.54),
            ('hepa_load',   'HEPA Init Load (%)', 0,  90,   0,  0.46),
        ]

        self.sliders = {}
        for key, label, vmin, vmax, vinit, ypos in slider_specs:
            ax_sl = self.fig.add_axes([0.015, ypos, 0.125, 0.022])
            sl = Slider(ax_sl, label, vmin, vmax, valinit=vinit,
                        color='#4A90D9', track_color='#D0E4F7')
            sl.label.set_fontsize(8)
            sl.valtext.set_fontsize(8)
            self.sliders[key] = sl

        # Scenario radio buttons
        self.fig.text(0.075, 0.415, 'FAULT SCENARIO',
                      ha='center', fontsize=8, fontweight='bold', color='#1A1A2E')
        ax_radio = self.fig.add_axes([0.012, 0.24, 0.135, 0.165])
        scenarios = ['Baseline', 'Fan Failure', 'Filter Clog',
                     'Sensor Drift', 'NRV Failure', 'PRV Stuck']
        self.radio = RadioButtons(ax_radio, scenarios,
                                   activecolor='#E74C3C')
        for lbl in self.radio.labels:
            lbl.set_fontsize(8)

        # Run button
        ax_btn = self.fig.add_axes([0.02, 0.175, 0.11, 0.045])
        self.btn_run = Button(ax_btn, '▶  RUN SIMULATION',
                               color='#2ECC71', hovercolor='#27AE60')
        self.btn_run.label.set_fontsize(9)
        self.btn_run.label.set_color('white')
        self.btn_run.on_clicked(self._on_run)

        # Status text
        self.status_ax = self.fig.add_axes([0.01, 0.13, 0.14, 0.04])
        self.status_ax.set_facecolor('#F0F0F0')
        self.status_ax.set_xticks([]); self.status_ax.set_yticks([])
        self.status_text = self.status_ax.text(0.5, 0.5, 'Ready to run',
            ha='center', va='center', fontsize=8, color='#555555',
            transform=self.status_ax.transAxes)

    def _on_run(self, event):
        self.status_text.set_text('Running simulation...')
        self.status_text.set_color('#E74C3C')
        self.fig.canvas.draw()

        # Read slider values
        duration_s  = int(self.sliders['duration'].val)
        T_amb       = self.sliders['T_ambient'].val
        fan_pwm     = self.sliders['fan_PWM'].val / 100.0
        brate       = int(self.sliders['breath_rate'].val)
        tvol        = self.sliders['tidal_vol'].val / 1000.0
        hepa_init   = self.sliders['hepa_load'].val / 100.0

        # Read selected scenario
        scenario_map = {
            'Baseline':    'Baseline (Normal Operation)',
            'Fan Failure': 'Fan Failure (Thermal Runaway)',
            'Filter Clog': 'Filter Clogging Progression',
            'Sensor Drift':'Sensor Drift (Firmware Blind)',
            'NRV Failure': 'NRV Failure (Back-flow)',
            'PRV Stuck':   'PRV Stuck + HW Switch Disabled',
        }
        sel = self.radio.value_selected
        self.active_scenario = scenario_map[sel]

        print(f"\n{'='*60}")
        print(f"RUNNING: {self.active_scenario}")
        print(f"{'='*60}")

        self.results = run_all_simulations(
            duration_s=duration_s, dt_s=0.5,
            T_ambient_C=T_amb, fan_PWM=fan_pwm,
            breath_rate=brate, tidal_vol_L=tvol,
            hepa_init_loading=hepa_init)

        self.ss_results = compute_steady_state_curves()

        self._plot_results()
        self.status_text.set_text(f'Done — {self.active_scenario[:28]}')
        self.status_text.set_color('#27AE60')
        self.fig.canvas.draw()

    def _plot_results(self):
        """Plot all simulation results onto the dashboard axes."""
        COLORS = {
            'Baseline (Normal Operation)':          '#2196F3',
            'Fan Failure (Thermal Runaway)':        '#F44336',
            'Filter Clogging Progression':          '#FF9800',
            'Sensor Drift (Firmware Blind)':        '#9C27B0',
            'NRV Failure (Back-flow)':              '#009688',
            'PRV Stuck + HW Switch Disabled':       '#E91E63',
        }

        # Clear all dynamic axes
        for ax in self.ax.values():
            ax.cla()
            ax.grid(True)
            ax.set_facecolor('#FAFAFA')

        # Determine which scenarios to plot
        # Always plot baseline; also plot active scenario if different
        to_plot = ['Baseline (Normal Operation)']
        if self.active_scenario not in to_plot:
            to_plot.append(self.active_scenario)

        for name in to_plot:
            if name not in self.results:
                continue
            t, arr, sys = self.results[name]
            c = COLORS[name]
            lbl = name.split('(')[0].strip()
            lw  = 2.2 if name == self.active_scenario else 1.2
            ls  = '-' if name == self.active_scenario else '--'
            alpha = 1.0 if name == self.active_scenario else 0.5

            kw = dict(color=c, linewidth=lw, linestyle=ls, alpha=alpha, label=lbl)

            # ── Row 0
            ax = self.ax['P_tank']
            ax.plot(t, arr['P_tank_bar'], **kw)
            ax.axhline(2.5, color='green', lw=1, ls=':', alpha=0.7, label='Setpoint 2.5 bar')
            ax.axhline(3.0, color='orange', lw=1, ls=':', alpha=0.7, label='HW Switch 3.0 bar')
            ax.axhline(3.2, color='red', lw=1, ls=':', alpha=0.7, label='PRV 3.2 bar')
            ax.set_ylabel('bar gauge'); ax.set_xlabel('Time (s)')

            self.ax['T_head'].plot(t, arr['T_head_max_C'], **kw)
            self.ax['T_head'].axhline(85, color='orange', lw=1, ls=':', alpha=0.6, label='Warning 85°C')
            self.ax['T_head'].axhline(105, color='red', lw=1, ls=':', alpha=0.6, label='Shutdown 105°C')
            self.ax['T_head'].set_ylabel('°C'); self.ax['T_head'].set_xlabel('Time (s)')

            self.ax['T_cool'].plot(t, arr['T_cooling_air_C'], **kw)
            self.ax['T_cool'].plot(t, np.full_like(t, self.sliders['T_ambient'].val),
                                    color='gray', lw=1, ls=':', alpha=0.5, label='Ambient')
            self.ax['T_cool'].set_ylabel('°C'); self.ax['T_cool'].set_xlabel('Time (s)')

            self.ax['fan_rpm'].plot(t, arr['fan1_rpm'], **kw)
            self.ax['fan_rpm'].set_ylabel('RPM'); self.ax['fan_rpm'].set_xlabel('Time (s)')

            self.ax['pump_flow'].plot(t, arr['pump_mdot_lpm'], **kw)
            self.ax['pump_flow'].set_ylabel('L/min (ANR)'); self.ax['pump_flow'].set_xlabel('Time (s)')

            # ── Row 1
            self.ax['current'].plot(t, arr['pump_current_A'], **kw)
            self.ax['current'].set_ylabel('Amperes'); self.ax['current'].set_xlabel('Time (s)')

            self.ax['hepa'].plot(t, arr['hepa_loading'], **kw)
            self.ax['hepa'].axhline(85, color='red', lw=1, ls=':', alpha=0.6, label='Replace 85%')
            self.ax['hepa'].set_ylabel('%'); self.ax['hepa'].set_xlabel('Time (s)')

            self.ax['odf'].plot(t, arr['odf_loading'], label=lbl+' ODF', color=c, lw=lw, ls=ls, alpha=alpha)
            self.ax['odf'].plot(t, arr['water_level'], label=lbl+' H₂O level',
                                 color=c, lw=lw*0.7, ls=':', alpha=alpha*0.8)
            self.ax['odf'].set_ylabel('%'); self.ax['odf'].set_xlabel('Time (s)')

            self.ax['mist'].plot(t, arr['mist_sat'], **kw)
            self.ax['mist'].axhline(80, color='red', lw=1, ls=':', alpha=0.6, label='Saturated 80%')
            self.ax['mist'].set_ylabel('%'); self.ax['mist'].set_xlabel('Time (s)')

            self.ax['ring'].plot(t, arr['ring_wear'], **kw)
            self.ax['ring'].axhline(70, color='orange', lw=1, ls=':', alpha=0.6, label='Replace 70%')
            self.ax['ring'].set_ylabel('%'); self.ax['ring'].set_xlabel('Time (s)')

            # ── Row 2
            self.ax['sensor'].plot(t, arr['P_tank_bar'], color=c, lw=lw, ls=ls,
                                    alpha=alpha, label=lbl+' True P')
            self.ax['sensor'].plot(t, arr['sensor_reading_bar'], color=c, lw=lw*0.7,
                                    ls=':', alpha=alpha, label=lbl+' Sensor reading')
            self.ax['sensor'].set_ylabel('bar gauge'); self.ax['sensor'].set_xlabel('Time (s)')

            self.ax['drain'].plot(t, arr['drain_bleed_lpm'], **kw)
            self.ax['drain'].set_ylabel('L/min (ANR)'); self.ax['drain'].set_xlabel('Time (s)')

            prv_open_line = arr['prv_open'] * arr['P_tank_bar']
            prv_open_line[arr['prv_open'] == 0] = np.nan
            self.ax['prv'].plot(t, arr['P_tank_bar'], **kw)
            self.ax['prv'].plot(t, prv_open_line, color='red', lw=3, alpha=0.9,
                                 label='PRV venting')
            self.ax['prv'].axhline(3.2, color='red', lw=1, ls=':', alpha=0.5)
            self.ax['prv'].set_ylabel('bar gauge'); self.ax['prv'].set_xlabel('Time (s)')

            self.ax['filt_dP'].plot(t, arr['total_dP_filtration_mbar'], **kw)
            self.ax['filt_dP'].set_ylabel('mbar'); self.ax['filt_dP'].set_xlabel('Time (s)')

            self.ax['weld'].plot(t, arr['weld_integrity_tank'], **kw)
            self.ax['weld'].axhline(80, color='red', lw=1, ls=':', alpha=0.6, label='Inspect >80%')
            self.ax['weld'].set_ylabel('% fatigue life used'); self.ax['weld'].set_xlabel('Time (s)')

            # ── Row 3
            # Thermal stacking: difference between cooling air and ambient
            T_amb_arr = np.full_like(t, self.sliders['T_ambient'].val)
            thermal_stack = arr['T_cooling_air_C'] - T_amb_arr
            self.ax['thermal_stk'].plot(t, thermal_stack, **kw)
            self.ax['thermal_stk'].axhline(0, color='gray', lw=0.8, alpha=0.5)
            self.ax['thermal_stk'].set_ylabel('ΔT above ambient (°C)')
            self.ax['thermal_stk'].set_xlabel('Time (s)')

            # Pump volumetric efficiency proxy
            pump_eff = np.clip(100 - arr['ring_wear'] * 0.8 -
                                arr['hepa_loading'] * 0.15, 0, 100)
            self.ax['pump_eff'].plot(t, pump_eff, **kw)
            self.ax['pump_eff'].set_ylabel('%'); self.ax['pump_eff'].set_xlabel('Time (s)')

            # Composite risk index (normalised, 0–100)
            risk = np.clip(
                arr['T_head_max_C'] / 105 * 30 +
                arr['ring_wear'] / 100 * 20 +
                arr['hepa_loading'] / 100 * 15 +
                arr['mist_sat'] / 100 * 15 +
                (np.abs(arr['P_tank_bar'] - arr['sensor_reading_bar']) / 0.5) * 20,
                0, 100)
            self.ax['risk'].plot(t, risk, **kw)
            self.ax['risk'].axhline(60, color='orange', lw=1, ls=':', alpha=0.6, label='Elevated risk')
            self.ax['risk'].axhline(80, color='red', lw=1, ls=':', alpha=0.6, label='High risk')
            self.ax['risk'].set_ylabel('Risk index (0–100)')
            self.ax['risk'].set_xlabel('Time (s)')

            # Cumulative energy
            power = arr['pump_current_A'] * 24.0
            energy_kJ = np.cumsum(power) * 0.5 / 1000
            self.ax['energy'].plot(t, energy_kJ, **kw)
            self.ax['energy'].set_ylabel('kJ'); self.ax['energy'].set_xlabel('Time (s)')

            # Duty cycle
            duty = np.convolve(arr['pump_PWM'], np.ones(20)/20, mode='same') * 100
            self.ax['hours'].plot(t, duty, **kw)
            self.ax['hours'].set_ylabel('Pump on-time (%)'); self.ax['hours'].set_xlabel('Time (s)')

        # Add legends to key axes
        for key in ['P_tank', 'T_head', 'sensor', 'prv', 'risk']:
            self.ax[key].legend(fontsize=7, loc='best', framealpha=0.7)

        # ── RESTORE TITLES
        titles_map = {
            'P_tank':    'Tank Pressure (bar gauge)',
            'T_head':    'Pump Head Temperature (°C)',
            'T_cool':    'Cooling Air Temp (°C)',
            'fan_rpm':   'Fan Speed (RPM)',
            'pump_flow': 'Pump Delivery Flow (L/min)',
            'current':   'Motor Current Draw (A)',
            'hepa':      'HEPA Filter Loading (%)',
            'odf':       'ODF Loading & Water Level (%)',
            'mist':      'Mist Sep Saturation (%)',
            'ring':      'Piston Ring Wear (%)',
            'sensor':    'Pressure Sensor: True vs Reading',
            'drain':     'Open Drain Bleed Flow (L/min)',
            'prv':       'PRV Status & Tank Pressure (bar)',
            'filt_dP':   'Filtration Train ΔP (mbar)',
            'weld':      'Tank Weld Fatigue (%)',
            'thermal_stk':'Thermal Stacking ΔT (°C above ambient)',
            'pump_eff':  'Effective Pump Efficiency (%)',
            'risk':      'Multi-fault Risk Index (0–100)',
            'energy':    'Cumulative Energy Input (kJ)',
            'hours':     'Pump Duty Cycle (%)',
        }
        for key, title in titles_map.items():
            self.ax[key].set_title(title, pad=4, fontsize=9)

        # ── STEADY STATE PLOTS
        self._plot_steady_state()

    def _plot_steady_state(self):
        if self.ss_results is None:
            return
        for ax in self.ax_ss:
            ax.cla()
            ax.grid(True)
            ax.set_facecolor('#F5F5F5')

        # ── SS Plot 0: Drain bleed vs pump output
        ax = self.ax_ss[0]
        P_g, drain_lpm, pump_lpm = self.ss_results['orifice']
        ax.fill_between(P_g, drain_lpm, pump_lpm, alpha=0.15, color='red',
                         label='Pump surplus (not vented)')
        ax.plot(P_g, drain_lpm, color='#E74C3C', lw=2, label='Drain bleed (1mm orifice)')
        ax.plot(P_g, pump_lpm, color='#2196F3', lw=2, label='Pump output')
        ax.axvline(2.5, color='green', ls='--', lw=1, alpha=0.7, label='Working pressure')
        ax.set_xlabel('Pressure (bar gauge)'); ax.set_ylabel('Flow (L/min ANR)')
        ax.set_title('Drain Bleed vs Pump Output', fontsize=9)
        ax.legend(fontsize=7); ax.set_xlim(0, 4)

        # ── SS Plot 1: HEPA ΔP
        ax = self.ax_ss[1]
        Q_lpm, dP_curves = self.ss_results['hepa']
        cmap = plt.cm.RdYlGn_r
        for i, (loading, dP) in enumerate(dP_curves.items()):
            c = cmap(loading)
            ax.plot(Q_lpm, dP, color=c, lw=1.8, label=f'{int(loading*100)}% loaded')
        ax.set_xlabel('Flow rate (L/min)'); ax.set_ylabel('ΔP (mbar)')
        ax.set_title('HEPA ΔP vs Flow', fontsize=9)
        ax.legend(fontsize=7, title='Loading', title_fontsize=7)

        # ── SS Plot 2: Pump performance curves
        ax = self.ax_ss[2]
        P_bar, pump_curves = self.ss_results['pump_perf']
        cmap2 = plt.cm.RdYlGn_r
        for i, (wear, flow) in enumerate(pump_curves.items()):
            c = cmap2(wear)
            ax.plot(P_bar, flow, color=c, lw=1.8, label=f'{int(wear*100)}% worn')
        ax.axvline(2.5, color='green', ls='--', lw=1, alpha=0.7)
        ax.set_xlabel('Outlet pressure (bar gauge)'); ax.set_ylabel('Flow (L/min ANR)')
        ax.set_title('Pump Performance (ring wear)', fontsize=9)
        ax.legend(fontsize=7, title='Ring wear', title_fontsize=7)

        # ── SS Plot 3: Compression temperature
        ax = self.ax_ss[3]
        PR, T35, T25 = self.ss_results['compression_temp']
        ax.plot(PR, T35, color='#F44336', lw=2, label='Inlet 35°C')
        ax.plot(PR, T25, color='#2196F3', lw=2, label='Inlet 25°C')
        ax.axvline(2.5/1.01325+1, color='green', ls='--', lw=1, alpha=0.7,
                    label='~2.5 bar outlet')
        ax.axhline(120, color='red', ls=':', lw=1, alpha=0.6, label='PTFE limit 120°C')
        ax.set_xlabel('Compression ratio P_out/P_in'); ax.set_ylabel('Outlet T (°C)')
        ax.set_title('Polytropic Compression Temp', fontsize=9)
        ax.legend(fontsize=7)

        # ── SS Plot 4: Filtration ΔP
        ax = self.ax_ss[4]
        Q_f, dP_cl, dP_hf, dP_cg = self.ss_results['filtration']
        ax.fill_between(Q_f, dP_cl, dP_cg, alpha=0.12, color='red')
        ax.plot(Q_f, dP_cl, color='#4CAF50', lw=2, label='Clean')
        ax.plot(Q_f, dP_hf, color='#FF9800', lw=2, label='50% loaded')
        ax.plot(Q_f, dP_cg, color='#F44336', lw=2, label='90% clogged')
        ax.set_xlabel('Flow rate (L/min)'); ax.set_ylabel('Total ΔP (mbar)')
        ax.set_title('Filtration Train ΔP', fontsize=9)
        ax.legend(fontsize=7)

        # Restore SS titles
        ss_titles = ['Drain Bleed vs Pump Output (Overpressure Analysis)',
                     'HEPA ΔP vs Flow (0–100% Loading)',
                     'Pump Flow vs Pressure (0–100% Ring Wear)',
                     'Polytropic Compression Outlet Temperature',
                     'Filtration Train ΔP (Clean → Clogged)']
        for ax, t in zip(self.ax_ss, ss_titles):
            ax.set_title(t, pad=4, fontsize=9)

    def show(self):
        # Run baseline immediately on launch
        print("\nInitial run — Baseline scenario...")
        self._on_run(None)
        plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# 8. MAIN
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("=" * 65)
    print("  MEDICAL VENTILATOR COMPRESSOR — SYSTEM SIMULATION")
    print("  G&M Tech 100RND-ED · 24VDC · 2L SS Tank · SMC Filtration")
    print("=" * 65)
    print("\nLaunching interactive dashboard...")
    print("Use the sliders on the left to change system parameters.")
    print("Select a fault scenario and click RUN SIMULATION.\n")

    db = Dashboard()
    db.show()