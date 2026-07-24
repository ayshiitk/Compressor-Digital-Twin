import numpy as np
import math

class AcousticSilencerChamber:
    """
    Component Model: Pneumatic Volume + Pipe-Inertance Buffer.

    -------------------------------------------------------------------
    WHY THIS VERSION EXISTS (stability note)
    -------------------------------------------------------------------
    The original implementation integrated the pipe-inertance momentum
    equation explicitly (RK4) with a quadratic (Borda-Carnot) drag term:

        dm_dot/dt = (A/L) * (P_up - P_chamber - dp_damp)

    For a short, wide pipe (25mm dia, 20mm long) the coefficient (A/L)
    is large, so the derivative is orders of magnitude bigger than the
    mass flows it is tracking. That makes the ODE numerically stiff:
    ANY explicit integrator (Euler, RK4, RK45, ...) will diverge to
    inf/NaN within a handful of steps unless dt is pushed down to the
    microsecond range or smaller -- impractical for multi-hour
    system-level simulations.

    This version instead solves the momentum equation with a
    SEMI-IMPLICIT (linearized backward-Euler) update: the quadratic
    drag term is linearized around the previous step's flow magnitude,
    turning the per-step update into a simple, bounded algebraic
    division instead of a stiff ODE integration:

        m_dot_new = (m_dot_old + dt*a) / (1 + dt*b*|m_dot_old|)

    This is UNCONDITIONALLY STABLE -- the denominator is always >= 1
    and grows with dt, so m_dot_new can never blow up, no matter how
    large or small dt_s is (microseconds to hours).

    The chamber mass balance (dm/dt = m_dot_in - m_dot_out) is then
    updated using this already-bounded m_dot_in, and the result is
    clamped to physically valid pressure/mass bounds as a final safety
    net against pathological inputs (e.g. a huge upstream pressure
    step combined with a very large dt).
    -------------------------------------------------------------------

    Public interface is unchanged from the original class, so it is a
    drop-in replacement:
        update_state(P_upstream_pa, m_dot_out_kg_s, dt_s) -> (pressure_pa, m_dot_in)
        get_gauge_pressure() -> Pa
    """

    def __init__(self, chamber_vol_liters = 290, inlet_pipe_dia_mm=11.0, inlet_pipe_len_mm=23.0):
        self.component_name = "SS Acoustic Silencer"
 
        # --- Thermodynamics Constants ---
        self.R_air = 287.05           # Gas constant for air (J/(kg*K))
        self.T_K = 293.15             # Isothermal temperature (K)
        self.P_atm = 101325.0         # Atmospheric pressure (Pa)

        # --- Geometries ---
        self.V_m3 = chamber_vol_liters / 1000.0                     # Chamber volume (m^3)
        self.A_pipe = math.pi * ((inlet_pipe_dia_mm / 2000.0) ** 2)  # Cross-sectional area (m^2)
        self.L_pipe = max(1e-4, inlet_pipe_len_mm / 1000.0)          # Length (m), guarded > 0
        self.K_expansion = 1.0        # Borda-Carnot loss coefficient for sudden expansion

        # --- State Variables ---
        self.pressure_pa = self.P_atm
        self.mass_kg = (self.pressure_pa * self.V_m3) / (self.R_air * self.T_K)
        self.m_dot_in = 0.0           # Current mass flow rate entering chamber (kg/s)

        # --- Absolute safety bounds ---
        # Deliberately generous relative to any realistic flow/pressure in this
        # system; they exist purely as a last line of defense against pathological
        # inputs (e.g. an hours-long dt combined with a huge pressure jump), not
        # as part of the normal operating physics.
        self.m_dot_max_kg_s = 0.05      # ~2500 NLPM ceiling (this system runs ~0.002 kg/s)
        self.pressure_min_pa = 1.0      # never allow zero/negative absolute pressure
        self.pressure_max_pa = 5.0e6    # 50 Bar ceiling

    def _compute_step(self, m_dot_in, mass_kg, pressure_pa, P_upstream_pa, m_dot_out_kg_s, dt_s):
        """
        Pure (non-mutating) computation of one timestep's (new_m_dot_in, new_mass_kg,
        new_pressure_pa) given an arbitrary starting state. Does not touch self.

        Solves momentum AND mass/pressure balance TOGETHER, implicitly, rather than
        solving momentum implicitly and then integrating mass explicitly afterward.
        The latter still lets the chamber overshoot within a single step: the
        momentum solve assumes P_chamber is fixed at its OLD value for the whole
        step, computes a large instantaneous flow consistent with that stale
        pressure, and only lets the pressure react on the NEXT step -- which is
        exactly the mechanism that produced a period-2 limit cycle (flow pegs high,
        which overshoots pressure, which then drives flow to near zero next step,
        which lets pressure sag again, repeat) once dt got large relative to this
        chamber's true (very fast) physical response time.

        Substituting the mass balance into the momentum equation makes P_chamber's
        dependence on the new flow explicit within the same implicit solve, which
        naturally damps this overshoot regardless of dt_s.
        """
        dt_s = max(1e-12, dt_s)
        P_upstream_pa = min(self.pressure_max_pa, max(self.pressure_min_pa, P_upstream_pa))
        rho_up = max(1e-6, P_upstream_pa / (self.R_air * self.T_K))

        D = self.A_pipe / self.L_pipe
        Cd_quad = self.K_expansion / (2.0 * rho_up * (self.A_pipe ** 2))
        b_coeff = D * Cd_quad

        # Pressure-per-unit-mass conversion for this chamber (Pa per kg)
        K_pm = (self.R_air * self.T_K) / self.V_m3

        # P_chamber_new = pressure_pa + dt*K_pm*(m_dot_new - m_dot_out_kg_s)
        # a_coeff_implicit = D*(P_upstream - P_chamber_new)
        #                  = D*(P_upstream - pressure_pa + dt*K_pm*m_dot_out_kg_s) - D*dt*K_pm*m_dot_new
        term_const = D * (P_upstream_pa - pressure_pa + dt_s * K_pm * m_dot_out_kg_s)

        # Full implicit equation:  A_lin*x + dt*b*x*|x| = c2
        #   where A_lin = 1 + dt^2*D*K_pm  (>=1, grows with dt^2: this is the extra
        #   self-damping from the chamber's own pressure response no longer being
        #   frozen for the whole step)
        A_lin = 1.0 + (dt_s ** 2) * D * K_pm
        c2 = m_dot_in + dt_s * term_const
        coef = dt_s * b_coeff

        if coef <= 0.0:
            m_dot_new = c2 / A_lin
        elif c2 >= 0.0:
            disc = A_lin ** 2 + 4.0 * coef * c2
            m_dot_new = (-A_lin + math.sqrt(disc)) / (2.0 * coef)
        else:
            disc = A_lin ** 2 - 4.0 * coef * c2
            m_dot_new = (A_lin - math.sqrt(disc)) / (2.0 * coef)
        m_dot_new = max(-self.m_dot_max_kg_s, min(self.m_dot_max_kg_s, m_dot_new))

        # ---- Mass / pressure update, now self-consistent with m_dot_new above ----
        new_mass_kg = mass_kg + (m_dot_new - m_dot_out_kg_s) * dt_s
        new_mass_kg = max(1e-9, new_mass_kg)
        new_pressure_pa = (new_mass_kg * self.R_air * self.T_K) / self.V_m3
        new_pressure_pa = min(self.pressure_max_pa, max(self.pressure_min_pa, new_pressure_pa))
        new_mass_kg = (new_pressure_pa * self.V_m3) / (self.R_air * self.T_K)

        return m_dot_new, new_mass_kg, new_pressure_pa

    def update_state(self, P_upstream_pa, m_dot_out_kg_s, dt_s):
        """
        Advances the acoustic chamber state by one time-step of size dt_s.
        Stable for ANY dt_s > 0 -- from microseconds to multi-hour steps.
        """
        m_dot_new, mass_new, pressure_new = self._compute_step(
            self.m_dot_in, self.mass_kg, self.pressure_pa, P_upstream_pa, m_dot_out_kg_s, dt_s
        )
        self.m_dot_in = m_dot_new
        self.mass_kg = mass_new
        self.pressure_pa = pressure_new
        return self.pressure_pa, self.m_dot_in

    def get_gauge_pressure(self):
        """Returns the pressure relative to atmosphere."""
        return self.pressure_pa - self.P_atm


# =====================================================================
# STABILITY VALIDATION -- same physics, wildly different dt_s
# =====================================================================
if __name__ == "__main__":
    upstream_p = 101325.0 + 8000.0   # realistic ~80 mBar restriction upstream
    pump_pulling_mass = 0.002        # kg/s, ~100 NLPM realistic pump draw

    print("Time(ms) | Inlet Flow (kg/s) | Chamber Pressure (Pa Gauge)")
    print("-" * 55)
    silencer = AcousticSilencerChamber(chamber_vol_liters=0.290)
    dt = 0.0001
    for step in range(1, 16):
        p_chamber, m_in = silencer.update_state(upstream_p, pump_pulling_mass, dt)
        print(f" {step * dt * 1000:5.1f}   |      {m_in:.6f}      |    {silencer.get_gauge_pressure():.2f}")

    print("\n=== Stability sweep across dt spanning 7 orders of magnitude ===")
    print(f"{'dt (s)':>10} | {'m_dot_in (kg/s)':>16} | {'Gauge P (Pa)':>14} | finite?")
    print("-" * 60)
    for dt_test in [1e-6, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 60.0, 3600.0]:
        s = AcousticSilencerChamber(chamber_vol_liters=0.290)
        p, m = 0.0, 0.0
        for _ in range(50):
            p, m = s.update_state(upstream_p, pump_pulling_mass, dt_test)
        ok = math.isfinite(p) and math.isfinite(m)
        print(f"{dt_test:>10g} | {m:>16.6f} | {s.get_gauge_pressure():>14.2f} | {ok}")