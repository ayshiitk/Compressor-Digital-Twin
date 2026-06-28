import math

class CopperCoolingCoil_TwinFan:
    """
    Component Model: Helical Copper Cooling Coil wrapped around a tank
    Housing: 148.3mm Square Chamber containing a 117.3mm OD Tank
    Coil: 136mm Mean Diameter, 8mm ID, 10mm OD Copper Tube
    Actuation: 2x SUNON PF97332BX Blowers in parallel crossflow
    """
    def __init__(self):
        # 1. Coil Physical Dimensions
        self.D_in = 0.008           # Internal Diameter (8mm)
        self.D_out = 0.010          # Outer Diameter (10mm)
        self.D_mean = 0.136         # Mean Coil Diameter (136mm)
        
        # 2. Chamber Enclosure Geometry
        self.W_chamber = 0.1483     # Width of square chamber (148.3mm)
        self.D_tank = 0.1173        # Outer diameter of the inner tank (117.3mm)
        
        # 3. Fan Performance Properties (SUNON PF97332BX x 2)
        # Max free delivery is 53.6 CFM per fan. Under static restriction of the 
        # tight coil bundle, we assume the pair operates at ~65% free airflow capacity.
        self.cfm_per_fan = 53.6
        self.estimated_efficiency = 0.5
        self.total_cooling_vol_flow = (2.0 * self.cfm_per_fan * self.estimated_efficiency) * 0.000471947 # Convert CFM to m3/s
        
        # 4. Material and Fluid Constants
        self.k_copper = 401.0       # High thermal conductivity of copper (W/m*K)
        self.R_air = 287.05         # Gas constant for air (J/kg*K)
        self.cp_air = 1009.0        # Specific heat of air (J/kg*K)
        self.rho_ambient = 1.293   # Ambient air density at room temp (kg/m3)

    def _get_air_properties(self, T_k):
        """Calculates temperature-dependent properties of air."""
        T_c = T_k - 273.15
        mu = 1.716e-5 * ((T_k / 273.15)**1.5) * ((273.15 + 110.4) / (T_k + 110.4)) # Sutherland's Law
        k_fluid = 0.0241 + (7.3e-5 * T_c)
        Pr = (mu * self.cp_air) / k_fluid
        return mu, k_fluid, Pr

    def _calculate_heat_transfer_coefficients(self, m_dot_kg_s, P_in_pa, T_mean_k, T_amb_k):
        """Calculates internal helical convection (hin) and external crossflow convection (hout)."""
        # --- Internal Fluid Dynamics & Convection ---
        mu_in, k_in, Pr_in = self._get_air_properties(T_mean_k)
        area_in = math.pi * (self.D_in / 2.0)**2
        rho_in = P_in_pa / (self.R_air * T_mean_k)
        v_in = m_dot_kg_s / (rho_in * area_in)
        
        Re_in = (rho_in * v_in * self.D_in) / mu_in
        # Dean Number for Helical Flow
        Dean = Re_in * math.sqrt(self.D_in / self.D_mean)
        
        # Internal Nusselt Number via Schmidt Helical Correction
        Nu_straight = 0.023 * (Re_in**0.8) * (Pr_in**0.4)
        Nu_helical = Nu_straight * (1.0 + 3.5 * (self.D_in / self.D_mean))
        h_in = (Nu_helical * k_in) / self.D_in

        # --- External Crossflow Convection (Chamber Airflow) ---
        mu_out, k_out, Pr_out = self._get_air_properties(T_amb_k)
        
        # Compute net free cross-sectional area of the chamber
        total_chamber_area = self.W_chamber**2
        tank_obstruction = (math.pi / 4.0) * (self.D_tank**2)
        net_free_area = total_chamber_area - tank_obstruction
        
        # External Air Velocity crossing the coils
        v_out = self.total_cooling_vol_flow / net_free_area
        Re_out = (self.rho_ambient * v_out * self.D_out) / mu_out
        
        # Churchill-Bernstein correlation for crossflow over a cylinder
        Nu_out = 0.3 + (0.62 * (Re_out**0.5) * (Pr_out**(1.0/3.0))) / \
                 ((1.0 + (0.4 / Pr_out)**(2.0/3.0))**0.25) * \
                 ((1.0 + (Re_out / 282000.0)**(5.0/8.0))**(4.0/5.0))
        h_out = (Nu_out * k_out) / self.D_out
        
        return h_in, h_out, Re_in, Dean, rho_in, v_in

    def analyze_coil(self, m_dot_kg_s, P_in_pa, T_in_k, T_amb_k, selected_length=3.36):
        """
        Computes the required length to hit a 1°C approach to ambient, 
        actual absolute gas outlet pressure, and ambient cooling air outlet temperature.
        """
        # Target threshold: 1 Degree above ambient
        target_approach = 1.0
        T_target_out_k = T_amb_k + target_approach
        T_mean_guess = (T_in_k + T_target_out_k) / 2.0
        
        # Compute convective strengths
        h_in, h_out, Re_in, Dean, rho_in, v_in = self._calculate_heat_transfer_coefficients(
            m_dot_kg_s, P_in_pa, T_mean_guess, T_amb_k
        )
        
        # --- ADD THESE TWO LINES TO PREVENT DIVIDE BY ZERO ---
        h_in = max(h_in, 5.0)   # Minimum 5 W/m2K for natural static convection
        h_out = max(h_out, 5.0) # In case the cooling fans are ever at 0 RPM

        # Existing code:
        U_out = 1.0 / ((1.0 / h_out) + (self.D_out / (self.D_in * h_in)))
        UA_per_meter = U_out * math.pi * self.D_out
        
        # --- Task 1: Find Required Length for 1°C Approach ---
        if T_in_k <= T_target_out_k:
            return T_in_k, T_amb_k - 273.15, P_in_pa  # No cooling needed; return inputs as outputs
          
        C_min = m_dot_kg_s * self.cp_air
        if C_min < 1e-6:   # ← ADD THIS GUARD
            return T_in_k, T_amb_k, P_in_pa

        NTU_req = -math.log(target_approach / (T_in_k - T_amb_k))
        L_required = (NTU_req * C_min) / UA_per_meter
        
        # --- Task 2: Evaluate Performance based on Selected Length ---
        L_eval = selected_length if selected_length is not None else L_required
        
        # Final Temperature Output Calculation for the compressed gas
        NTU_eval = (UA_per_meter * L_eval) / C_min
        T_final_k = T_amb_k + (T_in_k - T_amb_k) * math.exp(-NTU_eval)
        
        # --- Task 3: Ambient Cooling Air Temperature Rise ---
        # Total heat rate rejected from the compressed gas (Watts)
        watts_extracted = C_min * (T_in_k - T_final_k)
        # Mass flow rate of ambient cooling air moved by the fans
        m_dot_ambient = self.rho_ambient * self.total_cooling_vol_flow
        # Temperature increase of ambient air
        delta_T_ambient = watts_extracted / (m_dot_ambient * self.cp_air)
        T_ambient_out_celsius = (T_amb_k + delta_T_ambient) - 273.15
        
        # Friction Factor with Coiled Helical Correction (UPDATED)
        f_straight = 0.316 / (Re_in**0.25) if Re_in > 4000 else 64.0 / Re_in
        f_coiled = f_straight * (1.0 + 0.2425 * (Dean**0.2370)) # <-- Replaced 0.11 and 0.49
        
        # Pressure Drop calculation
        delta_p_pa = f_coiled * (L_eval / self.D_in) * (0.5 * rho_in * v_in**2)
        
        # Actual absolute outlet pressure of compressed gas
        P_out_pa = P_in_pa - delta_p_pa
        return T_final_k, T_ambient_out_celsius, P_out_pa
        # return T_final_k, T_ambient_out_celsius, P_out_pa, delta_p_pa, m_dot_kg_s/self.rho_ambient*60.0*1000.0, Re_in, Dean
    

# =====================================================================
# SYSTEM EVALUATION
# =====================================================================
if __name__ == "__main__":
    coil_solver = CopperCoolingCoil_TwinFan()

    # (Flow in SLPM, Inlet Pressure in BAR GAUGE)
    test_points = [
        (99.2, 0.40),
        (81.4, 1.86),
        (74.0, 2.47),
        (63.5, 2.02),
        (52.9, 2.92),
        (28.9, 1.10),
    ]

    temperature_inlet = 273.15 + 93.5      # K
    ambient_room = 273.15 + 25.0           # K

    rho_std = 1.293                        # kg/m³ at STP
    P_atm_bar = 1.01325                    # Atmospheric pressure

    print("-" * 110)
    print(f"{'Flow':>8} {'Pin(g)':>10} {'Pin(abs)':>10} {'Pout(abs)':>12} {'ΔP':>10} {'Tout':>10} {'Re':>10} {'Dean':>10}") 
    print("-" * 110)

    for flow_slpm, pin_gauge_bar in test_points:

        # Convert SLPM to kg/s
        mass_flow = flow_slpm * rho_std / (1000 * 60)

        # Convert Gauge -> Absolute pressure
        pin_abs_bar = pin_gauge_bar + P_atm_bar
        pin_abs_pa = pin_abs_bar * 1e5

        # Run model
        Tout, Tamb_out, Pout_pa, dP_pa, flow_out, Re_in, Dean = coil_solver.analyze_coil(
            mass_flow,
            pin_abs_pa,
            temperature_inlet,
            ambient_room
        )

        # Convert back to bar
        Pout_abs_bar = Pout_pa / 1e5
        dP_bar = dP_pa / 1e5

        print(f"{flow_slpm:8.1f}"
              f"{pin_gauge_bar:10.2f}"
              f"{pin_abs_bar:10.3f}"
              f"{Pout_abs_bar:12.3f}"
              f"{dP_bar:10.4f}"
              f"{Re_in:10.2f}"
              f"{Dean :10.2f}")