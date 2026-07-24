"""
CopperCoolingCoil_TwinFan
==========================
Component model: helical copper cooling coil wrapped around a tank, cooled by
two axial blowers, housed in a square chamber.

Geometry:
    Housing : 148.3mm square chamber containing a 117.3mm OD tank
    Coil    : 136mm mean diameter, 8mm ID / 10mm OD copper tube, close-wound
    Fans    : 2x SUNON PF97332BX blowers in parallel, axial flow along the coil
              stack (air flows top -> bottom; compressed gas flows bottom -> top,
              i.e. COUNTER-CURRENT)

Model history / physics included (see inline notes for each):
    1. Fan operating point solved from an actual fan curve (free-delivery CFM +
       shutoff static pressure) intersected with a system resistance curve,
       instead of an assumed flat "efficiency" fraction.
    2. Flow is split between two parallel paths across the chamber cross-section:
       - through the coil annulus (high resistance, threading past every turn)
       - around the open square corners next to the round coil bundle
         (near-zero resistance)
       Only the through-coil stream is credited with cooling the gas; the
       corner-bypass stream stays cold and simply dilutes the exhaust.
    3. External convection uses a row-corrected crossflow correlation (the coil
       is a multi-turn tube bank, not an isolated cylinder).
    4. The gas and the through-coil air are both marched along the coil length
       as a genuine counter-flow heat exchanger (RK4 + shooting method), rather
       than assuming the external air sits at a constant ambient temperature.

KNOWN OPEN ITEM (unresolved as of this version):
    Bench data shows this model still under-predicts the outlet gas temperature
    by a fairly constant ~3-4C across all tested flow rates (i.e. it still
    over-predicts cooling by a roughly constant amount, not a rate-dependent
    amount). Motor/compressor waste heat has been ruled out as the cause (the
    motor is thermally isolated from the coil's air circuit). Candidates still
    to investigate: contact/mounting resistance between coil and tank, the
    close-wound turn-pitch assumption, the assumed fan shutoff pressure (not
    from the exact PF97332BX datasheet), or a systematic overestimate in the
    internal (h_in) or external (h_out) correlations themselves. Do not treat
    this model as validated until that gap is closed against more bench data.

ASSUMPTIONS TO VERIFY AGAINST YOUR HARDWARE / DATASHEETS:
    - self.fan_shutoff_pa: taken from the closest published Sunon datasheet for
      this fan frame/bearing family, NOT the exact PF97332BX. Replace if you
      have the real number.
    - self.turn_pitch: assumes a close-wound coil (turns touching). If there is
      a real axial gap between turns, update this and the row-spacing physics.
    - self.K_minor / self.K_bypass: engineering-estimate minor loss coefficients
      for the through-coil and corner-bypass paths respectively.
"""

import math


class CopperCoolingCoil_TwinFan:

    def __init__(self):
        # --- Coil physical dimensions ---
        self.D_in = 0.008            # Internal diameter (8mm)
        self.D_out = 0.010           # Outer diameter (10mm)
        self.D_mean = 0.136          # Mean coil diameter (136mm)

        # --- Chamber enclosure geometry ---
        self.W_chamber = 0.1483      # Square chamber width (148.3mm)
        self.D_tank = 0.1173         # Inner tank OD (117.3mm)

        # --- Fan performance (SUNON PF97332BX x2) ---
        self.cfm_per_fan = 53.6
        self.fan_free_flow_m3s = self.cfm_per_fan * 0.000471947   # per-fan free-air delivery
        self.fan_shutoff_pa = 1300.0    # ASSUMPTION - verify vs actual datasheet

        # --- Coil winding assumption ---
        self.turn_pitch = self.D_out    # ASSUMPTION - close-wound (turns touching)

        # --- Minor loss coefficients ---
        self.K_minor = 1.3      # entrance/exit loss, through-coil path
        self.K_bypass = 0.4     # entrance/exit loss, open corner bypass path

        # --- Material / fluid constants ---
        self.k_copper = 401.0
        self.R_air = 287.05
        self.cp_air = 1009.0
        self.rho_ambient = 1.293

    # ------------------------------------------------------------------
    # Air properties
    # ------------------------------------------------------------------
    def _get_air_properties(self, T_k):
        """Temperature-dependent air properties (Sutherland's law for viscosity)."""
        T_c = T_k - 273.15
        mu = 1.716e-5 * ((T_k / 273.15) ** 1.5) * ((273.15 + 110.4) / (T_k + 110.4))
        k_fluid = 0.0241 + (7.3e-5 * T_c)
        Pr = (mu * self.cp_air) / k_fluid
        return mu, k_fluid, Pr

    # ------------------------------------------------------------------
    # Tube-bank row correction (Zukauskas / Incropera, aligned arrangement)
    # ------------------------------------------------------------------
    def _row_correction_factor(self, N):
        table = {1: 0.70, 2: 0.80, 3: 0.86, 4: 0.90, 5: 0.92, 6: 0.93, 7: 0.94,
                 8: 0.95, 9: 0.96, 10: 0.97, 13: 0.98, 16: 0.99, 20: 1.00}
        keys = sorted(table.keys())
        if N <= keys[0]:
            return table[keys[0]]
        if N >= keys[-1]:
            return 1.00
        for i in range(len(keys) - 1):
            k0, k1 = keys[i], keys[i + 1]
            if k0 <= N <= k1:
                t = (N - k0) / (k1 - k0)
                return table[k0] + t * (table[k1] - table[k0])
        return 1.00

    # ------------------------------------------------------------------
    # Chamber geometry: through-coil annulus vs. open corner bypass
    # ------------------------------------------------------------------
    def _geometry(self):
        total_chamber_area = self.W_chamber ** 2
        tank_area = (math.pi / 4.0) * self.D_tank ** 2
        net_free_area = total_chamber_area - tank_area

        coil_envelope_r = (self.D_mean + self.D_out) / 2.0
        coil_envelope_area = math.pi * coil_envelope_r ** 2
        annulus_area = coil_envelope_area - tank_area   # through-coil path
        bypass_area = net_free_area - annulus_area       # open square corners
        return net_free_area, annulus_area, bypass_area

    # ------------------------------------------------------------------
    # Fan operating point: two parallel resistance paths vs. the fan curve
    # ------------------------------------------------------------------
    def _solve_fan_operating_point(self, N_turns):
        net_free_area, annulus_area, bypass_area = self._geometry()

        ring_frontal_area = math.pi * self.D_mean * self.D_out
        sigma = min(ring_frontal_area / annulus_area, 0.95)   # blockage ratio within the annulus
        epsilon = 1.0 - sigma

        # Sudden contraction/expansion loss coefficient (rises as blockage -> 1)
        K_row = (1.0 / epsilon) ** 2 - 1.0
        K_A = N_turns * K_row + self.K_minor
        K_B = self.K_bypass

        rho = self.rho_ambient
        R_A = K_A * 0.5 * rho / (annulus_area ** 2)
        R_B = K_B * 0.5 * rho / (bypass_area ** 2)

        # Parallel combination for dP = R*Q^2 type resistances
        inv_sqrt_Req = 1.0 / math.sqrt(R_A) + 1.0 / math.sqrt(R_B)
        R_eq = 1.0 / (inv_sqrt_Req ** 2)

        Qf = self.fan_free_flow_m3s
        Psh = self.fan_shutoff_pa

        # Two fans in parallel, quadratic fan curve: dP = Psh*(1-(Q/(2Qf))^2)
        # Closed-form intersection with dP = R_eq * Q^2:
        denom = R_eq + Psh / (4.0 * Qf ** 2)
        Q_total = math.sqrt(Psh / denom)

        dP = R_eq * Q_total ** 2
        Q_A = math.sqrt(dP / R_A)   # through-coil flow
        Q_B = math.sqrt(dP / R_B)   # corner-bypass flow

        v_face_A = Q_A / annulus_area
        v_max = v_face_A / epsilon   # local velocity squeezing past a turn

        return v_face_A, v_max, Q_A, Q_B, Q_total, sigma

    # ------------------------------------------------------------------
    # Internal + external convection coefficients
    # ------------------------------------------------------------------
    def _calculate_heat_transfer_coefficients(self, m_dot_kg_s, P_in_pa, T_mean_k, T_amb_k, N_turns):
        # Internal (gas side, helical tube)
        mu_in, k_in, Pr_in = self._get_air_properties(T_mean_k)
        area_in = math.pi * (self.D_in / 2.0) ** 2
        rho_in = P_in_pa / (self.R_air * T_mean_k)
        v_in = m_dot_kg_s / (rho_in * area_in)

        Re_in = (rho_in * v_in * self.D_in) / mu_in
        Dean = Re_in * math.sqrt(self.D_in / self.D_mean)

        Nu_straight = 0.023 * (Re_in ** 0.8) * (Pr_in ** 0.4)
        Nu_helical = Nu_straight * (1.0 + 3.5 * (self.D_in / self.D_mean))
        h_in = (Nu_helical * k_in) / self.D_in

        # External (air side, row-corrected crossflow over the through-coil stream)
        mu_out, k_out, Pr_out = self._get_air_properties(T_amb_k)  # chamber air assumed slightly above ambient
        v_face_A, v_max, Q_A, Q_B, Q_total, sigma = self._solve_fan_operating_point(N_turns)

        Re_out = (self.rho_ambient * v_max * self.D_out) / mu_out
        Nu_out_isolated = 0.3 + (0.62 * (Re_out ** 0.5) * (Pr_out ** (1.0 / 3.0))) / \
            ((1.0 + (0.4 / Pr_out) ** (2.0 / 3.0)) ** 0.25) * \
            ((1.0 + (Re_out / 282000.0) ** (5.0 / 8.0)) ** (4.0 / 5.0))

        C2 = self._row_correction_factor(N_turns)
        Nu_out = Nu_out_isolated * C2
        h_out = (Nu_out * k_out) / self.D_out

        return h_in, h_out, Re_in, Dean, rho_in, v_in, v_face_A, v_max, Q_A, Q_B, Q_total

    # ------------------------------------------------------------------
    # Counter-flow temperature marching (RK4 + shooting method)
    # ------------------------------------------------------------------
    def _counterflow_profile(self, T_in_k, T_amb_k, C_gas, C_air, UA_per_meter, L, n_steps=200):
        """
        x=0 is the gas inlet / air outlet (bottom). x=L is the gas outlet / air
        inlet (top). Gas flows +x, air flows -x (enters fresh at x=L).
        Both T_gas(x) and T_air(x) are solved simultaneously.
        """
        
        dx = L / n_steps

        def derivs(Tg, Ta):
            q = UA_per_meter * (Tg - Ta)
            return -q / C_gas, -q / C_air

        def march(T_air0):
            Tg, Ta = T_in_k, T_air0
            Tgs = [Tg]; Tas = [Ta]
            for _ in range(n_steps):
                k1g, k1a = derivs(Tg, Ta)
                k2g, k2a = derivs(Tg + 0.5 * dx * k1g, Ta + 0.5 * dx * k1a)
                k3g, k3a = derivs(Tg + 0.5 * dx * k2g, Ta + 0.5 * dx * k2a)
                k4g, k4a = derivs(Tg + dx * k3g, Ta + dx * k3a)
                Tg = Tg + (dx / 6.0) * (k1g + 2 * k2g + 2 * k3g + k4g)
                Ta = Ta + (dx / 6.0) * (k1a + 2 * k2a + 2 * k3a + k4a)
                Tgs.append(Tg); Tas.append(Ta)
            return Tgs, Tas

        # Shooting method: bisect on the unknown air-exit temperature at x=0
        # so that the computed air temperature at x=L lands on the known ambient inlet.
        lo, hi = T_amb_k, T_in_k
        Tgs, Tas = march(0.5 * (lo + hi))
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            Tgs, Tas = march(mid)
            if Tas[-1] > T_amb_k:
                hi = mid
            else:
                lo = mid
        Tgs, Tas = march(0.5 * (lo + hi))
        return Tgs, Tas

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def analyze_coil(self, m_dot_kg_s, P_in_pa, T_in_k, T_amb_k, selected_length=3.36):
        """
        Returns a dict:
            T_out_C          - compressed gas outlet temperature (top of coil)
            T_mid_C          - compressed gas temperature at coil midpoint
            T_air_exit_C     - through-coil air exhaust temperature (bottom)
            P_out_pa         - gas outlet pressure (accounts for internal friction dP)
            delta_p_pa       - internal pressure drop
            diagnostics      - dict of intermediate values (h_in, h_out, C_gas,
                                C_air, UA_per_meter, Q_A_cfm, Q_B_cfm, N_turns, Re_in, Dean)
        """
        T_chamber_k = T_amb_k + 5.0  # Assume chamber air is at ambient temperature + 5C

        if m_dot_kg_s < 1e-9:
            return T_in_k, P_in_pa, 0.0, 0.0

        target_approach = 1.0
        T_mean_guess = (T_in_k + (T_chamber_k + target_approach)) / 2.0
        N_turns = max(selected_length / (math.pi * self.D_mean), 1.0)

        (h_in, h_out, Re_in, Dean, rho_in, v_in,
         v_face_A, v_max, Q_A, Q_B, Q_total) = self._calculate_heat_transfer_coefficients(
            m_dot_kg_s, P_in_pa, T_mean_guess, T_chamber_k, N_turns
        )
        h_in = max(h_in, 5.0)
        h_out = max(h_out, 5.0)

        U_out = 1.0 / ((1.0 / h_out) + (self.D_out / (self.D_in * h_in)))
        UA_per_meter = U_out * math.pi * self.D_out

        C_gas = m_dot_kg_s * self.cp_air
        m_dot_A = self.rho_ambient * Q_A
        C_air = max(m_dot_A * self.cp_air, 1e-6)

        Tgs, Tas = self._counterflow_profile(T_in_k, T_chamber_k, C_gas, C_air, UA_per_meter, selected_length)

        T_out_k = Tgs[-1]
        T_mid_k = Tgs[len(Tgs) // 2]
        T_air_exit_k = Tas[0]

        # Internal pressure drop (Dean-corrected friction factor, bench-fitted coefficients)
        f_straight = 0.316 / (Re_in ** 0.25) if Re_in > 4000 else 64.0 / Re_in
        f_coiled = f_straight * (1.0 + 0.2425 * (Dean ** 0.2370))
        delta_p_pa = f_coiled * (selected_length / self.D_in) * (0.5 * rho_in * v_in ** 2)
        P_out_pa = P_in_pa - delta_p_pa

        return T_out_k, P_out_pa, delta_p_pa, m_dot_kg_s/self.rho_ambient*60.0*1000.0
            

        return {
            'T_out_C': T_out_k - 273.15,
            'T_mid_C': T_mid_k - 273.15,
            'T_air_exit_C': T_air_exit_k - 273.15,
            'P_out_pa': P_out_pa,
            'delta_p_pa': delta_p_pa,
            'diagnostics': {
                'h_in': h_in, 'h_out': h_out, 'C_gas': C_gas, 'C_air': C_air,
                'UA_per_meter': UA_per_meter, 'Q_A_cfm': Q_A / 0.000471947,
                'Q_B_cfm': Q_B / 0.000471947, 'N_turns': N_turns,
                'Re_in': Re_in, 'Dean': Dean,
            }
        }


# # =========================================================================
# # VALIDATION AGAINST BENCH DATA
# # =========================================================================
# if __name__ == "__main__":
#     coil_solver = CopperCoolingCoil_TwinFan()
#     rho_std = 1.293
#     P_atm_bar = 1.01325

#     # (Back-pressure gauge bar, Flow LPM, T_coil_inlet C, T_coil_midpoint C, T_coil_outlet C, T_room C)
#     bench_data = [
#         (2.50, 60.0, 66.0, 37.7, 34.4, 27.3),
#         (2.53, 48.0, 56.0, 34.0, 33.2, 27.1),
#         (2.50, 40.0, 54.0, 34.9, 31.6, 27.0),
#         (2.50, 25.0, 47.0, 33.1, 29.8, 27.1),
#         (2.50, 51.0, 62.5, 34.4, 32.3, 27.1),
#         (2.50, 48.0, 59.0, 36.4, 32.9, 27.3),
#         (2.50, 40.0, 56.5, 36.4, 33.5, 27.5),
#         (2.50, 25.0, 49.0, None, 31.3, 27.6),
#     ]

#     print("-" * 90)
#     print(f"{'Flow':>6} {'Tin':>6} {'Mid(meas)':>10} {'Mid(pred)':>10} {'Out(meas)':>10} {'Out(pred)':>10}")
#     print("-" * 90)
#     for backP, flow_lpm, Tin, Tmid_m, Tout_m, Troom in bench_data:
#         mass_flow = flow_lpm * rho_std / (1000 * 60)
#         pin_abs_pa = (backP + P_atm_bar) * 1e5
#         result = coil_solver.analyze_coil(mass_flow, pin_abs_pa, Tin + 273.15, Troom + 273.15)
#         tmid_str = f"{Tmid_m:10.1f}" if Tmid_m is not None else "       - "
#         print(f"{flow_lpm:6.1f} {Tin:6.1f} {tmid_str} {result['T_mid_C']:10.1f} {Tout_m:10.1f} {result['T_out_C']:10.1f}")



if __name__ == "__main__":
    coil_solver = CopperCoolingCoil_TwinFan()

    # (Flow in SLPM, Inlet Pressure in BAR GAUGE, Inlet Temp in CELSIUS)
    test_points = [
        (51, 2.5, 52.4),
        (52, 2.5, 49.6),
        (48, 2.5, 56.0),
        (60.0, 2.5, 66),
        # (52.9, 2.92, 60.0),
        # (28.9, 1.10, 45.0),
    ]

    ambient_room = 273.15 + 27.0           # K

    rho_std = 1.293                        # kg/m³ at STP
    P_atm_bar = 1.01325                    # Atmospheric pressure

    print("-" * 65)
    print(f"{'Flow':>8} {'Pin(abs)':>10} {'ΔP(bar)':>10} {'Tin(C)':>8} {'Tout(C)':>8} {'Tamb(C)':>8}") 
    print("-" * 65)

    for flow_slpm, pin_gauge_bar, tin_celsius in test_points:

        # Convert SLPM to kg/s
        mass_flow = flow_slpm * rho_std / (1000 * 60)

        # Convert Gauge -> Absolute pressure
        pin_abs_bar = pin_gauge_bar + P_atm_bar
        pin_abs_pa = pin_abs_bar * 1e5
        
        # Convert Inlet Temp -> Kelvin
        temperature_inlet = tin_celsius + 273.15

        # Run model (unused return variables are ignored)
        Tout, Pout_pa, dP_pa, flow_out = coil_solver.analyze_coil(
            mass_flow,
            pin_abs_pa,
            temperature_inlet,
            ambient_room
        )

        # 1. Pressure Conversions to Bar
        dP_bar = dP_pa / 1e5
        
        # 2. Temperature Conversions to Celsius
        Tout_celsius = Tout - 273.15
        Tamb_celsius = ambient_room - 273.15

        # Printed as a single formatted string to ensure perfect column alignment
        print(f"{flow_slpm:8.1f} {pin_abs_bar:10.3f} {dP_bar:10.4f} {tin_celsius:8.1f} {Tout_celsius:8.1f} {Tamb_celsius:8.1f}")