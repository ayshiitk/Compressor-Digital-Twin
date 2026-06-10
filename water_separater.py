import numpy as np

# ==========================================
# 1. PSYCHROMETRIC BRIDGE (Condensation Model)
# ==========================================
def calculate_coil_condensation(T_amb_C, RH_amb, P_amb_pa, T_coil_C, P_coil_pa, m_dot_air_kg_s):
    """
    Calculates the exact phase-change of water vapor into liquid droplets
    between the compressor intake (ambient) and the cooling coil exhaust.
    """
    # Antoine Equation Constants for Water
    A = 8.07131
    B = 1730.63
    C = 233.426
    
    # --- STAGE 1: COMPRESSOR INTAKE (Ambient Room Conditions) ---
    P_sat_amb_pa = (10 ** (A - B / (T_amb_C + C))) * 133.322
    P_vapor_amb_pa = P_sat_amb_pa * RH_amb
    omega_in = 0.622 * (P_vapor_amb_pa / (P_amb_pa - P_vapor_amb_pa))
    m_dot_water_in_kg_s = m_dot_air_kg_s * omega_in
    
    # --- STAGE 2: COOLING COIL EXHAUST (Compressed & Cooled) ---
    P_sat_coil_pa = (10 ** (A - B / (T_coil_C + C))) * 133.322
    omega_max_coil = 0.622 * (P_sat_coil_pa / (P_coil_pa - P_sat_coil_pa))
    
    # --- STAGE 3: PHASE CHANGE (Condensation) ---
    if omega_in > omega_max_coil:
        m_dot_vapor_out_kg_s = m_dot_air_kg_s * omega_max_coil
        m_dot_liquid_out_kg_s = m_dot_air_kg_s * (omega_in - omega_max_coil)
    else:
        m_dot_vapor_out_kg_s = m_dot_water_in_kg_s
        m_dot_liquid_out_kg_s = 0.0 # Air is dry enough; no liquid forms
        
    return {
        "water_intake_mg_s": m_dot_water_in_kg_s * 1000000.0,
        "vapor_passed_mg_s": m_dot_vapor_out_kg_s * 1000000.0,
        "liquid_condensed_mg_s": m_dot_liquid_out_kg_s * 1000000.0
    }

# ==========================================
# 2. WATER SEPARATOR MODEL
# ==========================================
class SMC_AFG20_WaterSeparator:
    """
    Digital Twin Component: SMC AFG20-F02-J-D Water Separator
    """
    def __init__(self, valve_closed=False):
        self.component_name = "SMC AFG20 Water Separator"
        self.R_air = 287.05      
        self.rho_normal = 1.204  
        self.eta_max = 0.99      
        self.Q_critical_lpm = 8.0 
        self.k_shape = 2.0       
        self.beta_inertial = 8.5e7 
        
        # Leakage Configuration ("J" Option)
        self.drain_valve_closed = valve_closed 
        self.drain_hole_area_m2 = np.pi * ((1.8 / 1000.0 / 2)**2) 
        self.Cd_drain = 0.62     

    def calculate_efficiency(self, Q_actual_lpm):
        if Q_actual_lpm <= 0: return 0.0
        decay_factor = (Q_actual_lpm / self.Q_critical_lpm) ** self.k_shape
        efficiency = self.eta_max * (1.0 - np.exp(-decay_factor))
        return max(0.0, min(self.eta_max, efficiency))

    def calculate_leakage(self, P_in_pa, T_in_k, P_atm_pa=101325.0):
        if self.drain_valve_closed: return 0.0 
        pr = P_atm_pa / P_in_pa
        gamma = 1.4
        critical_ratio = (2 / (gamma + 1)) ** (gamma / (gamma - 1))
        
        if pr <= critical_ratio:
            m_dot_leak = self.Cd_drain * self.drain_hole_area_m2 * P_in_pa * np.sqrt(gamma / (self.R_air * T_in_k)) * (2 / (gamma + 1)) ** ((gamma + 1) / (2 * (gamma - 1)))
        else:
            m_dot_leak = self.Cd_drain * self.drain_hole_area_m2 * P_in_pa * np.sqrt((2 * gamma / (gamma - 1)) / (self.R_air * T_in_k) * (pr ** (2/gamma) - pr ** ((gamma + 1)/gamma)))
        return m_dot_leak

    # def update_state(self, m_dot_in_kg_s, P_in_pa, T_in_k, liquid_water_in_mg_s):
    #     m_dot_leak_kg_s = self.calculate_leakage(P_in_pa, T_in_k)
    #     m_dot_effective = max(0.0, m_dot_in_kg_s - m_dot_leak_kg_s)
        
    #     rho = P_in_pa / (self.R_air * T_in_k)
    #     dp_inertial = (self.beta_inertial * m_dot_effective**2) / rho if rho > 0 else 0.0
    #     P_out_pa = max(0.0, P_in_pa - dp_inertial)
        
    #     Q_actual_lpm = (m_dot_effective / rho) * 60000.0 if rho > 0 else 0.0
    #     eta_current = self.calculate_efficiency(Q_actual_lpm)
        
    #     water_captured_mg_s = liquid_water_in_mg_s * eta_current
    #     water_escaped_mg_s = liquid_water_in_mg_s - water_captured_mg_s
        
    #     return {
    #         "P_out_pa": P_out_pa,
    #         "dp_pa": dp_inertial,
    #         "m_dot_out_kg_s": m_dot_effective,
    #         "m_dot_leak_kg_s": m_dot_leak_kg_s,
    #         "Q_actual_lpm": Q_actual_lpm,
    #         "efficiency": eta_current,
    #         "water_captured_mg_s": water_captured_mg_s,
    #         "water_escaped_mg_s": water_escaped_mg_s
    #     }
    
    def update_state(self, m_dot_in_kg_s, P_in_pa, T_in_k, liquid_water_in_mg_s):
        """
        Processes a single simulation time step through the separator node.
        """
        # 1. Inlet Density
        rho_in = P_in_pa / (self.R_air * T_in_k)
        
        # 2. Aerodynamic Pressure Drop 
        # ALL incoming air passes through the deflector vanes, so we use m_dot_in_kg_s
        if rho_in > 0:
            dp_inertial = (self.beta_inertial * m_dot_in_kg_s**2) / rho_in
        else:
            dp_inertial = 0.0
            
        # The pressure actually inside the bowl, after passing the swirl vanes
        P_bowl_pa = max(0.0, P_in_pa - dp_inertial)
        
        # 3. Leakage Check 
        # The air leaks from the bottom of the bowl, driven by P_bowl_pa
        m_dot_leak_kg_s = self.calculate_leakage(P_bowl_pa, T_in_k)
        
        # 4. Effective Output Mass Flow (What actually makes it to the OUT port)
        m_dot_effective = max(0.0, m_dot_in_kg_s - m_dot_leak_kg_s)
        
        # 5. Moisture Removal Efficiency
        # The centrifugal vortex is driven by the TOTAL incoming volumetric flow hitting the vanes
        if rho_in > 0:
            Q_actual_in_m3_s = m_dot_in_kg_s / rho_in
            Q_actual_lpm = Q_actual_in_m3_s * 60000.0 
        else:
            Q_actual_lpm = 0.0
            
        eta_current = self.calculate_efficiency(Q_actual_lpm)
        
        # 6. Calculate water captured vs escaped downstream
        water_captured_mg_s = liquid_water_in_mg_s * eta_current
        water_escaped_mg_s = liquid_water_in_mg_s - water_captured_mg_s
        
        return {
            "P_out_pa": P_bowl_pa,                  # Output pressure to feed next component
            "dp_pa": dp_inertial,                   # True pressure drop across the spin vanes
            "m_dot_out_kg_s": m_dot_effective,      # Air mass surviving the leak
            "m_dot_leak_kg_s": m_dot_leak_kg_s,     # Air lost to the 1.8mm hole
            "Q_actual_lpm": Q_actual_lpm,           # The true internal volumetric spin flow
            "efficiency": eta_current,              # Dynamic centrifugal capture rate
            "water_captured_mg_s": water_captured_mg_s, # Liquid stripped out
            "water_escaped_mg_s": water_escaped_mg_s    # Liquid passing downstream
        }




# ==========================================
# 3. SYSTEM INTEGRATION TEST (Nodal Coupling)
# ==========================================
if __name__ == "__main__":
    print("=== DIGITAL TWIN: WEATHER TO WATER SEPARATOR ===")
    
    # 1. Define Ambient Weather (e.g., Hot & Humid)
    T_ambient = 35.0         # 35 deg C
    RH_ambient = 0.95        # 95% Humidity
    P_ambient = 101325.0     # 1 Bar Atmos
    
    # 2. Define Cooling Coil Outputs (Fed into Separator)
    mass_flow = 0.002        # ~100 NLPM from compressor
    P_coil_out = 400000.0    # 4 Bar Absolute (~3 Bar Gauge)
    T_coil_out = 36.0        # 36 deg C (cooled compressed air)
    
    # STEP A: Run the Psychrometric Condensation Model
    weather_results = calculate_coil_condensation(
        T_amb_C=T_ambient, 
        RH_amb=RH_ambient, 
        P_amb_pa=P_ambient, 
        T_coil_C=T_coil_out, 
        P_coil_pa=P_coil_out, 
        m_dot_air_kg_s=mass_flow
    )
    
    condensed_liquid = weather_results['liquid_condensed_mg_s']
    print(f"\n[WEATHER SENSOR STAGE]")
    print(f" -> Ambient Humidity pulled into compressor: {weather_results['water_intake_mg_s']:.1f} mg/s")
    print(f" -> Forced Condensation in Cooling Coil:     {condensed_liquid:.1f} mg/s")
    
    # STEP B: Run the Separator using the condensed liquid calculated above
    separator_leaking = SMC_AFG20_WaterSeparator(valve_closed=False)
    
    sep_results = separator_leaking.update_state(
        m_dot_in_kg_s=mass_flow, 
        P_in_pa=P_coil_out, 
        T_in_k=T_coil_out + 273.15, 
        liquid_water_in_mg_s=condensed_liquid  # <-- THIS IS THE DATA HANDOFF
    )
    
    print(f"\n[WATER SEPARATOR STAGE]")
    print(f" -> Aerodynamic Pressure Drop: {sep_results['dp_pa']/100:.2f} mBar")
    print(f" -> 1.8mm Unvalved Air Leak:   {(sep_results['m_dot_leak_kg_s']/1.204)*60000:.1f} NLPM lost to room")
    print(f" -> Catch Efficiency:          {sep_results['efficiency']*100:.2f}%")
    
    # STEP C: Handoff to the next component (The Mist Separator)
    print(f"\n[HANDOFF TO MIST SEPARATOR]")
    print(f" -> Aerosols Escaping (1% missed): {sep_results['water_escaped_mg_s']:.2f} mg/s")
    print(f" -> Invisible Vapor Passing:       {weather_results['vapor_passed_mg_s']:.1f} mg/s")