import numpy as np
import math

class IntakeHose_HiPoFlex:
    """
    Full Intake Path Model: Muffling Chamber -> Arc Tube -> HiPoFlex Hose -> Pump Head Inlet

    Path breakdown (per updated routing):
      1. 12.7mm ID rigid/semi-rigid arc tube from muffling chamber, 70mm arc length,
         135-degree sweep, connecting DIRECTLY (straight, no elbow) into the hose inlet.
      2. Sudden contraction loss at the 12.7mm -> 9mm ID step-down (this exists even
         though the joint itself has no elbow -- any abrupt ID reduction has a loss).
      3. HiPoFlex Transparent Braided Silicone Hose: 9x16mm, 600mm length, 3x 90-deg
         sweeping bends.
      4. Exit elbow fitting into the pump head inlet (same part/K-factor as the
         discharge-side elbow, per user confirmation both pump-head connections
         use identical fittings).

    Calculates combined fluid dynamic pressure drop and thermodynamic heat loss.
    Heat transfer (NTU) is modeled only across the silicone HiPoFlex hose section,
    since the upstream arc tube's material/wall properties are not yet specified
    and intake air is assumed close to ambient temperature already -- flag if you
    want thermal modeling added for that segment too.
    """
    def __init__(self):
        # ============================================================
        # 1. Upstream Arc Tube (Muffling Chamber -> Hose Inlet)
        # ============================================================
        self.D_arc = 0.0127          # Arc tube ID in meters (12.7 mm)
        self.L_arc = 0.070           # Arc tube flow length in meters (70 mm arc length)
        self.bend_angle_arc_deg = 135.0  # Sweep angle of the arc tube

        # ASSUMPTION: reference K-factor for a smooth 90-deg bend at the r/d implied
        # by this arc's geometry (r/d ~= 2.3 here). Typical published range for a
        # smooth bend at this r/d is ~0.15-0.30; 0.22 used as a mid-range estimate.
        # Verify against vendor/fitting data (e.g. Crane TP-410, Miller charts) if available.
        self.K90_bend_reference = 0.22
        # Simple linear angle scaling (K_theta = K90 * theta/90) -- a common quick
        # engineering approximation, not an exact chart lookup.
        self.K_bend_arc = self.K90_bend_reference * (self.bend_angle_arc_deg / 90.0)

        # ============================================================
        # 2. HiPoFlex Hose (unchanged geometry)
        # ============================================================
        self.L = 0.600              # Length in meters (600 mm)
        self.D_in = 0.009           # Inner Diameter in meters (9 mm)
        self.D_out = 0.016          # Outer Diameter in meters (16 mm)

        self.num_90_bends = 3       # Three 90-degree sweeping turns in the hose
        self.K_per_bend = 0.25      # Minor loss for a sweeping 50mm radius bend

        # Sudden contraction loss at the 12.7mm -> 9mm step-down, evaluated on the
        # downstream (smaller-bore) dynamic pressure: K = 0.5*(1 - A2/A1)
        area_ratio = (self.D_in / self.D_arc) ** 2
        self.K_contraction = 0.5 * (1.0 - area_ratio)

        # Exit elbow into the pump head inlet -- SAME physical fitting as the
        # discharge-side inlet elbow (K matched per user confirmation: K=1.2)
        self.K_elbow_fitting = 0.90

        # ============================================================
        # 3. Material & Thermal Properties
        # ============================================================
        self.k_silicone = 0.25      # Thermal conductivity of silicone (W/m*K)
        self.h_out = 7.0            # Natural convection coefficient, still room air (W/m2*K)

        # ============================================================
        # 4. Static Air Properties
        # ============================================================
        self.R_air = 287.05         # Gas constant for air (J/kg*K)
        self.cp_air = 1009.0        # Specific heat of air (J/kg*K)

    def _get_dynamic_air_properties(self, T_k):
        # Sutherland's Law for Dynamic Viscosity (mu)
        mu_ref = 1.716e-5
        T_ref = 273.15
        S = 110.4
        mu = mu_ref * ((T_k / T_ref) ** 1.5) * ((T_ref + S) / (T_k + S))

        # Linear approximation for Thermal Conductivity of Air (k_air)
        T_celsius = T_k - 273.15
        k_air = 0.0241 + (7.3e-5 * T_celsius)

        # Dynamic Prandtl Number (Pr = mu * Cp / k)
        Pr = (mu * self.cp_air) / k_air

        return mu, k_air, Pr

    @staticmethod
    def _friction_factor(Re):
        # Blasius Equation for Smooth Tubes (turbulent), laminar fallback otherwise
        if Re > 4000:
            return 0.316 / (Re ** 0.25)
        else:
            return max(0.02, 64.0 / max(1, Re))

    def calculate_hose_state(self, m_dot_kg_s, P_in_pa, T_in_k, T_amb_k):
        """
        Passes air from the muffling chamber through the arc tube, contraction,
        HiPoFlex hose, and exit elbow into the pump head, tracking pressure
        progressively through each segment (compressibility-aware, stepwise).
        """
        if m_dot_kg_s <= 0.0001:
            return P_in_pa, T_amb_k, 0.0

        mu_air, k_air, Pr_air = self._get_dynamic_air_properties(T_in_k)

        # ==========================================================
        # SEGMENT 1: Arc Tube (12.7mm ID, 70mm, 135-deg sweep)
        # ==========================================================
        area_arc = math.pi * (self.D_arc / 2.0) ** 2
        rho_arc = P_in_pa / (self.R_air * T_in_k)
        v_arc = m_dot_kg_s / (rho_arc * area_arc)
        Re_arc = (rho_arc * v_arc * self.D_arc) / mu_air
        f_arc = self._friction_factor(Re_arc)

        dyn_q_arc = 0.5 * rho_arc * v_arc ** 2
        dP_friction_arc = f_arc * (self.L_arc / self.D_arc) * dyn_q_arc
        dP_bend_arc = self.K_bend_arc * dyn_q_arc

        P_after_arc = P_in_pa - dP_friction_arc - dP_bend_arc

        # ==========================================================
        # SEGMENT 2: Sudden Contraction (12.7mm -> 9mm), straight joint
        # ==========================================================
        area_in = math.pi * (self.D_in / 2.0) ** 2
        rho_contraction = P_after_arc / (self.R_air * T_in_k)
        v_hose_inlet = m_dot_kg_s / (rho_contraction * area_in)
        dyn_q_contraction = 0.5 * rho_contraction * v_hose_inlet ** 2
        dP_contraction = self.K_contraction * dyn_q_contraction

        P_after_contraction = P_after_arc - dP_contraction

        # ==========================================================
        # SEGMENT 3: HiPoFlex Hose (9x16mm, 600mm, 3x sweeping bends)
        # ==========================================================
        rho_in = P_after_contraction / (self.R_air * T_in_k)
        velocity = m_dot_kg_s / (rho_in * area_in)
        Re = (rho_in * velocity * self.D_in) / mu_air
        f = self._friction_factor(Re)

        dyn_pressure = 0.5 * rho_in * velocity ** 2
        delta_p_straight = f * (self.L / self.D_in) * dyn_pressure
        delta_p_bends = (self.num_90_bends * self.K_per_bend) * dyn_pressure

        P_before_elbow = P_after_contraction - delta_p_straight - delta_p_bends

        # ==========================================================
        # SEGMENT 4: Exit Elbow into Pump Head Inlet
        # ==========================================================
        rho_exit = P_before_elbow / (self.R_air * T_in_k)
        v_exit = m_dot_kg_s / (rho_exit * area_in)
        dyn_q_exit = 0.5 * rho_exit * v_exit ** 2
        delta_p_elbow = self.K_elbow_fitting * dyn_q_exit

        P_out_pa = P_before_elbow - delta_p_elbow

        # ==========================================================
        # PART 2: THERMODYNAMICS (Heat Loss via NTU) -- HiPoFlex hose only
        # ==========================================================
        Nu_in = 0.023 * (Re ** 0.8) * (Pr_air ** 0.3)
        h_in = (Nu_in * k_air) / self.D_in
        R_in = 1.0 / (h_in * math.pi * self.D_in * self.L)

        R_wall = math.log(self.D_out / self.D_in) / (2.0 * math.pi * self.k_silicone * self.L)
        R_out = 1.0 / (self.h_out * math.pi * self.D_out * self.L)

        R_total = R_in + R_wall + R_out
        UA = 1.0 / R_total

        C_min = m_dot_kg_s * self.cp_air
        NTU = UA / C_min

        T_out_k = T_amb_k + (T_in_k - T_amb_k) * math.exp(-NTU)
        heat_loss_watts = C_min * (T_in_k - T_out_k)

        return P_out_pa, T_out_k, heat_loss_watts


# =====================================================================
# SIMULATION EXECUTION (TEST BLOCK)
# =====================================================================
if __name__ == "__main__":
    hose = IntakeHose_HiPoFlex()

    # NOTE: Updated to realistic INTAKE-side conditions (pre-compression, near
    # ambient) -- the original test block used 4.0 Bar / 95C, which are
    # discharge-side (post-compression, hot) conditions. Adjust as needed if
    # your muffling chamber runs at a measurable sub-atmospheric draw or a
    # different ambient temp.
    test_m_dot = 0.002            # approx 90 NLPM flow
    test_press_pa = 101325.0      # ~1.0 Bar Absolute (atmospheric, pre-pump)
    test_temp_k = 273.15 + 28.0   # ~28C, near-ambient intake air
    test_amb_k = 273.15 + 25.0    # 25C still room air

    p_final, t_final, watts_lost = hose.calculate_hose_state(
        m_dot_kg_s=test_m_dot,
        P_in_pa=test_press_pa,
        T_in_k=test_temp_k,
        T_amb_k=test_amb_k
    )

    print("=== Intake Path Physics Results (Muffler -> Arc -> Hose -> Pump) ===")
    print(f"Muffler Chamber Out: {test_press_pa/100000.0:.5f} Bar, {test_temp_k-273.15:.1f} °C")
    print(f"Pressure Dropped:    {(test_press_pa - p_final)/100.0:.2f} mBar (Arc+Contraction+Hose+Elbow)")
    print(f"Pump Inlet Pres:     {p_final/100000.0:.5f} Bar")
    print("-" * 43)
    print(f"K_bend_arc (135deg): {hose.K_bend_arc:.3f}")
    print(f"K_contraction:       {hose.K_contraction:.3f}")
    print(f"Temp Dropped:        {(test_temp_k - t_final):.2f} °C")
    print(f"Pump Inlet Temp:     {t_final - 273.15:.1f} °C")
    print(f"Radiated Heat:       {watts_lost:.2f} Watts")