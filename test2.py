import numpy as np

class SmartCompressor_120RND_Empirical:
    """
    Component Model: G&M Tech 120RND-ED Double Head Pump.
    
    E3 + E4 FIX:
    Previously, head heat was computed as (P_shaft - P_isentropic) while gas outlet
    temperature was computed from the isentropic relation independently. This was
    internally inconsistent: the "extra" shaft work was being assigned to the head
    metal while the gas simultaneously exited at the ideal isentropic temperature --
    violating energy conservation.

    Corrected energy partition (all flows derived from the same eta_is):

        P_isentropic = m_dot * cp * T1 * (pr^((g-1)/g) - 1)   [minimum compression work]
        P_shaft_to_gas = P_isentropic / eta_is                  [actual work into gas]
        Q_head = P_shaft_to_gas - P_isentropic                  [irreversibility -> head metal]
        T2_actual = T1 + P_shaft_to_gas / (m_dot * cp)         [consistent discharge T]

    The actual discharge temperature is now higher than isentropic, and the head
    heat source is the exact complement. Both terms sum to P_shaft_to_gas, conserving
    energy within the compression stage.

    eta_is is a function of differential pressure, fit from empirical data or set
    as a reasonable constant (0.65-0.75 for small reciprocating compressors).
    """

    def __init__(self):
        self.component_name = "120RND-ED Empirical Compressor (E3+E4 Fixed)"

        # --- 1. Thermodynamics & Mass Properties ---
        self.gamma = 1.4
        self.rho_normal = 1.204     # kg/m^3 at 20 C, 1 bar (NTP)
        self.cp_air = 1005.0        # J/(kg.K)

        self.Cth_motor = 1513.4     # J/K  (steel + copper stator)
        self.Cth_head  = 1413.0     # J/K  (aluminium heads)

        # State variables
        self.T_ambient_k = 293.15
        self.T_motor_k   = 293.15
        self.T_head_k    = 293.15

        self.motor_limit_k          = 273.15 + 75.0
        self.motor_throttle_start_k = 273.15 + 65.0
        self.head_target_k          = 273.15 + 60.0

        # --- 2. Isentropic efficiency curve ---
        # eta_is as a function of differential pressure (bar).
        # For a small reciprocating compressor, eta_is typically falls with
        # rising pressure ratio due to valve losses and re-expansion work.
        # Replace these knot values with your measured data.
        #   dp_bar:  0     1     2     3     4     5     6     7
        #   eta_is:  0.72  0.71  0.70  0.69  0.67  0.65  0.63  0.60
        _dp  = np.array([0, 1, 2, 3, 4, 5, 6, 7], dtype=float)
        _eta = np.array([0.72, 0.71, 0.70, 0.69, 0.67, 0.65, 0.63, 0.60])
        self.eta_is_curve = np.polyfit(_dp, _eta, 2)

        # --- 3. Manufacturer Empirical Pneumatic Curves ---
        p_bar  = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        f_nlpm = np.array([133, 121, 108, 90, 79, 67, 60, 52])
        c_amps = np.array([9.9, 16.1, 20.8, 22.9, 24.0, 24.5, 24.9, 24.5])

        self.flow_curve    = np.polyfit(p_bar, f_nlpm, 2)
        self.current_curve = np.polyfit(p_bar, c_amps, 3)

        # --- 4. Run Auto-Calibration ---
        self.motor_Rth_curve, self.head_Rth_curve = self._auto_calibrate_thermal_resistance()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_eta_is(self, dp_bar: float) -> float:
        """
        Returns isentropic efficiency for a given differential pressure.
        Clamped to [0.40, 0.85] so polynomial extrapolation cannot give
        physically nonsensical values outside the calibration range.
        """
        eta = float(np.polyval(self.eta_is_curve, dp_bar))
        return max(0.40, min(0.85, eta))

    def _isentropic_power(self, m_dot: float, T_up_k: float, pr: float) -> float:
        """
        P_isentropic = m_dot * cp * T1 * (pr^((gamma-1)/gamma) - 1)
        Returns 0 if pr <= 1 (no compression work).
        """
        if pr <= 1.0 or m_dot <= 0.0:
            return 0.0
        return m_dot * self.cp_air * T_up_k * ((pr ** ((self.gamma - 1.0) / self.gamma)) - 1.0)

    def _compression_energy_split(
        self,
        m_dot: float,
        T_up_k: float,
        pr: float,
        dp_bar: float
    ):
        """
        Core fix for E3 + E4.

        Returns (P_isentropic, P_shaft_to_gas, Q_head_w, T_out_gas_k) where:

          P_isentropic   -- minimum (ideal) compression power [W]
          P_shaft_to_gas -- actual shaft work delivered to gas [W]
                         = P_isentropic / eta_is
          Q_head_w       -- irreversibility heat to head metal [W]
                         = P_shaft_to_gas - P_isentropic
          T_out_gas_k    -- actual discharge temperature [K]
                         = T1 + P_shaft_to_gas / (m_dot * cp)

        All four quantities are derived from the SAME eta_is, so:
          P_shaft_to_gas = P_isentropic + Q_head_w  (energy is conserved)
          T_out is consistent with Q_head.

        Old (broken) behaviour:
          Q_head  = P_shaft - P_isentropic          (used eta_mech only)
          T_out   = T1 * pr^((g-1)/g)               (isentropic, independent)
          => Q_head and T_out were computed from different assumptions,
             double-counting or losing energy.
        """
        if m_dot <= 0.0 or pr <= 1.0:
            return 0.0, 0.0, 0.0, T_up_k

        eta_is = self._get_eta_is(dp_bar)
        P_is   = self._isentropic_power(m_dot, T_up_k, pr)

        # Actual shaft work into the gas (always >= P_is because eta_is <= 1)
        P_shaft_to_gas = P_is / eta_is

        # Irreversibility: extra work that raises gas T above isentropic value
        Q_head_w = P_shaft_to_gas - P_is

        # Consistent actual discharge temperature
        T_out_gas_k = T_up_k + P_shaft_to_gas / (m_dot * self.cp_air)

        return P_is, P_shaft_to_gas, Q_head_w, T_out_gas_k

    # ------------------------------------------------------------------
    # Auto-calibration (unchanged except head loop uses fixed split)
    # ------------------------------------------------------------------

    def _auto_calibrate_thermal_resistance(self):
        """
        Ingests raw empirical data, calculates thermodynamic waste heat,
        extracts Rth, and returns polynomial curve fits.
        """
        # =============================================
        # PASTE RAW MOTOR TEST DATA HERE
        # =============================================
        raw_motor_pwm    = np.array([0.25, 0.50, 0.75, 1.00])
        raw_motor_amps   = np.array([6.0, 12.0, 18.0, 24.0])
        raw_motor_temp_c = np.array([45.0, 55.0, 62.0, 68.0])
        raw_motor_amb_c  = np.array([25.0, 25.0, 25.0, 25.0])

        p_elec_motor   = 24.0 * raw_motor_amps
        q_waste_motor  = p_elec_motor - (p_elec_motor * 0.90 * 0.85)
        calc_Rth_motor = (raw_motor_temp_c - raw_motor_amb_c) / q_waste_motor
        motor_fit      = np.polyfit(raw_motor_pwm, calc_Rth_motor, 2)

        # =============================================
        # PASTE RAW HEAD TEST DATA HERE
        # =============================================
        raw_head_blower_cfm     = np.array([0.0, 15.0, 30.0, 50.0])
        raw_head_amps           = np.array([24.0, 24.0, 24.0, 24.0])
        raw_head_press_bar_abs  = np.array([4.0, 4.0, 4.0, 4.0])
        raw_head_temp_c         = np.array([85.0, 72.0, 63.0, 55.0])
        raw_head_amb_c          = np.array([25.0, 25.0, 25.0, 25.0])

        calc_Rth_head = np.zeros(len(raw_head_blower_cfm))

        for i in range(len(raw_head_blower_cfm)):
            p_elec   = 24.0 * raw_head_amps[i]
            p_shaft  = p_elec * 0.90 * 0.85

            dp_bar   = max(0.0, raw_head_press_bar_abs[i] - 1.0)
            flow_nlpm = max(0.0, np.polyval(self.flow_curve, dp_bar))
            m_dot    = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            pr       = raw_head_press_bar_abs[i] / 1.0
            t_in_k   = raw_head_amb_c[i] + 273.15

            # E3+E4 FIX: use consistent energy split in calibration too.
            # The head heat source during the test was Q_head from the
            # corrected split, not (P_shaft - P_isentropic_ideal).
            _, P_shaft_to_gas, Q_head_w, _ = self._compression_energy_split(
                m_dot, t_in_k, pr, dp_bar
            )

            # Any shaft power not delivered to the gas goes to the head.
            # If the test was done at 100 % PWM, the full mechanical loss
            # (P_shaft - P_shaft_to_gas) also ends up as head heat.
            # Here P_shaft is the mechanical shaft power; P_shaft_to_gas
            # is what actually went into compressing the gas.
            q_waste_head_total = max(0.0, p_shaft - P_shaft_to_gas + Q_head_w)

            if q_waste_head_total > 0.0:
                calc_Rth_head[i] = (raw_head_temp_c[i] - raw_head_amb_c[i]) / q_waste_head_total
            else:
                calc_Rth_head[i] = 0.5  # fallback; should not occur with real data

        head_fit = np.polyfit(raw_head_blower_cfm, calc_Rth_head, 2)
        return motor_fit, head_fit

    # ------------------------------------------------------------------
    # Thermal derating
    # ------------------------------------------------------------------

    def _calculate_thermal_derate(self, requested_pwm: float) -> float:
        if self.T_motor_k <= self.motor_throttle_start_k:
            return requested_pwm
        if self.T_motor_k >= self.motor_limit_k:
            return 0.0
        temp_margin = self.motor_limit_k - self.motor_throttle_start_k
        temp_excess = self.T_motor_k - self.motor_throttle_start_k
        return requested_pwm * (1.0 - (temp_excess / temp_margin))

    # ------------------------------------------------------------------
    # Main state update
    # ------------------------------------------------------------------

    def update_system_state(
        self,
        P_up_pa: float,
        T_up_k: float,
        P_down_pa: float,
        req_pump_pwm: float,
        req_blower_cfm: float,
        dt_s: float
    ):
        """
        Advances compressor state by dt_s seconds.

        Returns
        -------
        m_dot_kg_s   : mass flow rate [kg/s]
        current_a    : motor current draw [A]
        T_out_gas_k  : actual (non-isentropic) discharge temperature [K]  <- E4 fixed
        actual_pwm   : PWM after thermal derating
        T_motor_k    : updated motor temperature [K]
        T_head_k     : updated head temperature [K]
        P_pneumatic  : isentropic compression power [W]  (for system-level bookkeeping)
        Q_head_w     : irreversibility heat to head [W]  <- new output, was hidden before
        """
        req_pump_pwm    = max(0.0, min(1.0, req_pump_pwm))
        req_blower_cfm  = max(0.0, req_blower_cfm)

        actual_pwm = self._calculate_thermal_derate(req_pump_pwm)

        dp_bar = max(0.0, P_down_pa - P_up_pa) / 1e5

        if dp_bar > 8.0 or actual_pwm == 0.0:
            m_dot_kg_s   = 0.0
            current_a    = 0.0
            P_pneumatic  = 0.0
            Q_head_w     = 0.0
            T_out_gas_k  = T_up_k
        else:
            flow_nlpm  = max(0.0, np.polyval(self.flow_curve,    dp_bar)) * actual_pwm
            m_dot_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            current_a  = max(0.0, np.polyval(self.current_curve, dp_bar)) * actual_pwm

            pr = P_down_pa / max(1.0, P_up_pa)

            # --- E3 + E4 FIX ---
            # Single call produces all four quantities from the same eta_is.
            # Previously T_out_gas used pr^((g-1)/g) (isentropic),
            # while Q_head used a different formula -- now both are consistent.
            P_pneumatic, P_shaft_to_gas, Q_head_w, T_out_gas_k = \
                self._compression_energy_split(m_dot_kg_s, T_up_k, pr, dp_bar)

        # --- Electrical / mechanical power (motor side, unchanged) ---
        P_elec   = 24.0 * current_a
        P_shaft  = P_elec * 0.90 * 0.85

        heat_motor_w = max(0.0, P_elec - P_shaft)

        # --- Head heat source (E3 fix) ---
        # Old: heat_head_w = P_shaft - P_pneumatic
        #      (used isentropic P_pneumatic as if eta_is = 1, always underestimated)
        # New: heat_head_w = Q_head_w from corrected split
        #      This is the actual irreversibility power heating the head metal.
        #      Mechanical losses (P_shaft - P_shaft_to_gas) are NOT included here;
        #      they are assumed to exit via the crankcase / motor frame rather than
        #      the cylinder head. Adjust this partition if your thermal survey shows
        #      otherwise (e.g. if the connecting-rod bearing heat reaches the head).
        heat_head_w = Q_head_w

        # --- Thermal resistance from auto-calibrated curves ---
        Rth_motor = max(0.05, np.polyval(self.motor_Rth_curve, actual_pwm))
        Rth_head  = max(0.05, np.polyval(self.head_Rth_curve,  req_blower_cfm))

        # --- Newton cooling ---
        cooling_motor_w = (self.T_motor_k - self.T_ambient_k) / Rth_motor
        cooling_head_w  = (self.T_head_k  - self.T_ambient_k) / Rth_head

        # --- Integrate temperatures ---
        self.T_motor_k += ((heat_motor_w - cooling_motor_w) / self.Cth_motor) * dt_s
        self.T_head_k  += ((heat_head_w  - cooling_head_w)  / self.Cth_head)  * dt_s

        return (
            m_dot_kg_s,
            current_a,
            T_out_gas_k,   # actual discharge T (E4 fixed)
            actual_pwm,
            self.T_motor_k,
            self.T_head_k,
            P_pneumatic,
            Q_head_w,      # new: irreversibility power to head
        )


# ==========================================
# Test Execution
# ==========================================
if __name__ == "__main__":
    comp = SmartCompressor_120RND_Empirical()

    p_in  = 100000.0   # 1 bar abs
    p_out = 400000.0   # 4 bar abs
    t_in  = 293.15     # 20 C

    print("=" * 90)
    print("Empirical Compressor Simulation — E3 + E4 Fixed")
    print("  Pressure ratio : 4:1   |   Inlet: 1 bar, 20 C   |   100% PWM, 25 CFM blower")
    print("=" * 90)
    print(f"{'Time(s)':>7} | {'Flow(kg/s)':>10} | {'T_motor(C)':>10} | {'T_head(C)':>9} "
          f"| {'T_gas_out(C)':>12} | {'P_is(W)':>8} | {'Q_head(W)':>10} | {'eta_is':>7}")
    print("-" * 90)

    dt = 1.0
    for step in range(3001):
        result = comp.update_system_state(
            P_up_pa=p_in, T_up_k=t_in, P_down_pa=p_out,
            req_pump_pwm=1.0, req_blower_cfm=25.0, dt_s=dt
        )
        m_dot, amps, t_gas, act_pwm, t_mot, t_head, P_is, Q_head = result

        if step % 100 == 0:
            dp  = (p_out - p_in) / 1e5
            eta = comp._get_eta_is(dp)
            print(
                f"{step:7.1f} | {m_dot:10.5f} | {t_mot-273.15:10.2f} | "
                f"{t_head-273.15:9.2f} | {t_gas-273.15:12.2f} | "
                f"{P_is:8.1f} | {Q_head:10.1f} | {eta:7.3f}"
            )

            if act_pwm < 1.0:
                print(f"  -> THERMAL THROTTLE ACTIVE: pump power reduced to {act_pwm*100:.1f}%")

    print()
    print("Sanity check at final step:")
    dp   = (p_out - p_in) / 1e5
    pr   = p_out / p_in
    eta  = comp._get_eta_is(dp)
    flow = max(0.0, np.polyval(comp.flow_curve, dp))
    mdot = ((flow / 1000.0) / 60.0) * comp.rho_normal
    P_is_check = mdot * comp.cp_air * t_in * ((pr ** ((comp.gamma-1)/comp.gamma)) - 1.0)
    P_actual   = P_is_check / eta
    Q_h_check  = P_actual - P_is_check
    T_out_check = t_in + P_actual / (mdot * comp.cp_air)
    print(f"  eta_is          = {eta:.4f}")
    print(f"  m_dot           = {mdot*1000:.4f} g/s")
    print(f"  P_isentropic    = {P_is_check:.2f} W")
    print(f"  P_shaft_to_gas  = {P_actual:.2f} W   (= P_is / eta_is)")
    print(f"  Q_head          = {Q_h_check:.2f} W   (= P_shaft_to_gas - P_is)")
    print(f"  P_is + Q_head   = {P_is_check + Q_h_check:.2f} W   <- must equal P_shaft_to_gas")
    print(f"  T_out_gas       = {T_out_check - 273.15:.2f} C   (actual, not isentropic)")
    print(f"  T_out_isentropic= {t_in * (pr**((comp.gamma-1)/comp.gamma)) - 273.15:.2f} C   (for reference)")