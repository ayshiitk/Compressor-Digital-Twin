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
        self.estimated_efficiency = 0.65
        self.total_cooling_vol_flow = (2.0 * self.cfm_per_fan * self.estimated_efficiency) * 0.000471947 # Convert CFM to m3/s
        
        # 4. Material and Fluid Constants
        self.k_copper = 401.0       # High thermal conductivity of copper (W/m*K)
        self.R_air = 287.05         # Gas constant for air (J/kg*K)
        self.cp_air = 1009.0        # Specific heat of air (J/kg*K)
        self.rho_ambient = 1.204    # Ambient air density at room temp (kg/m3)

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

    def analyze_coil(self, m_dot_kg_s, P_in_pa, T_in_k, T_amb_k, selected_length=None):
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
        
        # Overall thermal resistance based on outer surface area (neglecting copper wall resistance)
        U_out = 1.0 / ((1.0 / h_out) + (self.D_out / (self.D_in * h_in)))
        UA_per_meter = U_out * math.pi * self.D_out
        
        # --- Task 1: Find Required Length for 1°C Approach ---
        C_min = m_dot_kg_s * self.cp_air
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
        
        # Friction Factor with Coiled Helical Correction
        f_straight = 0.316 / (Re_in**0.25) if Re_in > 4000 else 64.0 / Re_in
        f_coiled = f_straight * (1.0 + 0.11 * (Dean**0.49))
        
        # Pressure Drop calculation
        delta_p_pa = f_coiled * (L_eval / self.D_in) * (0.5 * rho_in * v_in**2)
        
        # Actual absolute outlet pressure of compressed gas
        P_out_pa = P_in_pa - delta_p_pa
        P_out_bar = P_out_pa / 100000.0  # Convert to Bar absolute
        
        excess_length = L_eval - L_required if selected_length is not None else 0.0
        
        return {
            "L_required_meters": L_required,
            "T_final_celsius": T_final_k - 273.15,
            "T_ambient_out_celsius": T_ambient_out_celsius,
            "pressure_drop_mbar": delta_p_pa / 100.0,
            "P_out_bar": P_out_bar,
            "excess_length_meters": excess_length
        }

# =====================================================================
# SYSTEM EVALUATION
# =====================================================================
if __name__ == "__main__":
    coil_solver = CopperCoolingCoil_TwinFan()
    
    # Boundary Conditions arriving from the previous discharge hose state
    mass_flow = 0.00196          # 91 NLPM
    pressure_inlet = 398500.0    # ~3.98 Bar absolute (accounting for hose drop)
    temperature_inlet = 273.15 + 93.5 # 93.5°C entering from the silicone hose
    ambient_room = 273.15 + 25.0 # 25°C Room Air
    
    # SCENARIO A: Calculate what the system mathematically needs
    ideal_results = coil_solver.analyze_coil(mass_flow, pressure_inlet, temperature_inlet, ambient_room)
    
    print("=== OPTIMUM COOLING COIL DESIGN PARAMETERS ===")
    print(f"Required Length to hit 1°C of Ambient: {ideal_results['L_required_meters']:.3f} meters")
    print(f"Compressed Air Exit Temp:              {ideal_results['T_final_celsius']:.2f} °C")
    print(f"Actual Gas Outlet Pressure:            {ideal_results['P_out_bar']:.3f} Bar absolute")
    print(f"Pressure Drop:                 {ideal_results['pressure_drop_mbar']:.1f} mBar")
    print(f"Exhaust Ambient Air Temp (to pump):    {ideal_results['T_ambient_out_celsius']:.2f} °C")
    print("-" * 55)
    
    # SCENARIO B: If you pick a standard off-the-shelf physical length
    custom_length = 3.36
    custom_results = coil_solver.analyze_coil(mass_flow, pressure_inlet, temperature_inlet, ambient_room, selected_length=custom_length)
    
    print(f"=== PERFORMANCE ANALYSIS FOR SELECTED LENGTH ({custom_length}m) ===")
    print(f"Actual Compressed Air Exit Temp:       {custom_results['T_final_celsius']:.2f} °C")
    print(f"Actual Gas Outlet Pressure:            {custom_results['P_out_bar']:.3f} Bar absolute")
    print(f"Total Pressure Drop Over Coil:          {custom_results['pressure_drop_mbar']:.1f} mBar")
    print(f"Exhaust Ambient Air Temp (to pump):    {custom_results['T_ambient_out_celsius']:.2f} °C")
    print(f"Calculated Excess Length Used:          {custom_results['excess_length_meters']:.3f} meters")