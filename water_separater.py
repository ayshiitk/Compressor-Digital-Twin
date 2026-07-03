import numpy as np
import math
class SMC_AFG20_WaterSeparator:
    """
    Digital Twin Component: SMC AFG20-F02-J-D Water Separator
    Now includes a fully integrated psychrometric condensation model.
    """
    def __init__(self, valve_closed=False):
        self.component_name = "SMC AFG20 Water Separator"
        self.R_air = 287.05      
        self.rho_normal = 1.204  
        
        # Manufacturer Specs (SMC AFG20-D)
        self.eta_max = 0.99             # 99% water droplet removal ratio
        self.Q_max_nlpm = 1000.0        # Max air flow capacity: 1,000 l/min (ANR)
        self.Q_activation_nlpm = 15.0   # Flow needed to spin the vortex effectively
        self.k_shape = 2.0       
        # self.beta_inertial = 8.5e7 
        self.beta_inertial = 2.78e9   # calibrated from 90 SLPM measured point
        
        # Leakage Configuration ("J" Option)
        self.drain_valve_closed = valve_closed 

        # Leakage Configuration
        self.drain_valve_closed = valve_closed 
        
        # ---------------------------------------------------------
        # SERIES ORIFICE PHYSICS (1.8mm bowl -> 1.0mm restrictor)
        # ---------------------------------------------------------
        d1_bowl_m = 1.8 / 1000.0        # Built-in bowl hole
        d2_restrictor_m = 1.0 / 1000.0  # Added pipe restrictor
        
        A1 = math.pi * (d1_bowl_m / 2.0)**2
        A2 = math.pi * (d2_restrictor_m / 2.0)**2
        
        # Calculate the equivalent aerodynamic area of both holes in series
        # self.drain_hole_area_m2 = (A1 * A2) / math.sqrt(A1**2 + A2**2)
        # self.drain_hole_area_m2 = A2

        A_theoretical =  2.165e-7
        self.drain_hole_area_m2 = A_theoretical
        
        self.Cd_drain = 1 # Standard discharge coefficient
 

    def _calculate_condensation(self, T_amb_C, RH_amb, P_amb_pa, T_coil_C, P_coil_pa, m_dot_total_kg_s):
        """
        Calculates the exact phase-change of water vapor into liquid droplets.
        Corrected to isolate dry air mass for 100% thermodynamic accuracy.
        """
        # Antoine Equation Constants for Water
        A, B, C = 8.07131, 1730.63, 233.426
        
        # --- STAGE 1: INTAKE (Ambient Room Conditions) ---
        P_sat_amb_pa = (10 ** (A - B / (T_amb_C + C))) * 133.322
        P_vapor_amb_pa = P_sat_amb_pa * RH_amb
        omega_in = 0.622 * (P_vapor_amb_pa / (P_amb_pa - P_vapor_amb_pa))
        
        # Isolate true dry air mass (Thermodynamic Fix)
        m_dot_dry_air = m_dot_total_kg_s / (1.0 + omega_in)
        m_dot_water_in_kg_s = m_dot_dry_air * omega_in
        
        # --- STAGE 2: COIL EXHAUST (Compressed & Cooled) ---





        # ---------------------------------------------------------
        # THERMODYNAMIC CLAMP: Protect the Antoine Equation
        # The Antoine formula is only valid between 0°C and 100°C.
        # This prevents math explosions if the air temp spikes for 1 millisecond.
        # ---------------------------------------------------------
        T_safe_C = max(0.0, min(100.0, T_coil_C))
        
        # Calculate Saturation Pressure (P_sat) using the clamped temperature
        # 133.322 converts mmHg to Pascals
        P_sat_coil_pa = (10 ** (A - B / (T_safe_C + C))) * 133.322



        # P_sat_coil_pa = (10 ** (A - B / (T_coil_C + C))) * 133.322
        # omega_max_coil = 0.622 * (P_sat_coil_pa / (P_coil_pa - P_sat_coil_pa))

        # Safely calculate the dry air pressure (preventing division by zero)
        P_dry_air_pa = max(1.0, P_coil_pa - P_sat_coil_pa)
        
        # Calculate maximum humidity ratio
        omega_max_coil = 0.622 * (P_sat_coil_pa / P_dry_air_pa)
        
        # --- STAGE 3: PHASE CHANGE (Condensation) ---
        if omega_in > omega_max_coil:
            m_dot_vapor_out_kg_s = m_dot_dry_air * omega_max_coil
            m_dot_liquid_out_kg_s = m_dot_dry_air * (omega_in - omega_max_coil)
        else:
            m_dot_vapor_out_kg_s = m_dot_water_in_kg_s
            m_dot_liquid_out_kg_s = 0.0 # Air is dry enough; no liquid forms
            
        return m_dot_liquid_out_kg_s * 1000000.0, m_dot_vapor_out_kg_s * 1000000.0, m_dot_water_in_kg_s * 1000000.0

    def _calculate_efficiency(self, Q_nlpm):
        """
        Calculates droplet removal based on flow rate. 
        Drops off if flow is too weak to create a vortex, caps at 99% up to 1000 NLPM.
        """
        if Q_nlpm <= 0: 
            return 0.0
        # Flow decay logic: Needs a minimum flow (activation) to spin the water out
        decay_factor = (Q_nlpm / self.Q_activation_nlpm) ** self.k_shape
        efficiency = self.eta_max * (1.0 - np.exp(-decay_factor))
        
        # If pushed beyond 1000 NLPM (SMC Max Rating), efficiency would plummet due to flooding
        if Q_nlpm > self.Q_max_nlpm:
            efficiency *= (self.Q_max_nlpm / Q_nlpm)
            
        return max(0.0, min(self.eta_max, efficiency))

    # def calculate_leakage(self, P_in_pa, T_in_k, P_atm_pa=101325.0):
    #     """Handles choked (sonic) vs subsonic leakage through the drain hole."""
    #     if self.drain_valve_closed or P_in_pa <= P_atm_pa: 
    #         return 0.0 

    #     # if self.drain_valve_closed: return 0.0 
    #     pr = P_atm_pa / P_in_pa
    #     gamma = 1.4
    #     critical_ratio = (2 / (gamma + 1)) ** (gamma / (gamma - 1))
        
    #     if pr <= critical_ratio:
    #         m_dot_leak = self.Cd_drain * self.drain_hole_area_m2 * P_in_pa * math.sqrt(gamma / (self.R_air * T_in_k)) * (2 / (gamma + 1)) ** ((gamma + 1) / (2 * (gamma - 1)))
    #     else:
    #         m_dot_leak = self.Cd_drain * self.drain_hole_area_m2 * P_in_pa * math.sqrt((2 * gamma / (gamma - 1)) / (self.R_air * T_in_k) * (pr ** (2/gamma) - pr ** ((gamma + 1)/gamma)))
    #     return m_dot_leak
    
    def _calculate_leakage(self, P_in_pa, T_in_k, P_atm_pa=101325.0):
        """Calculates the continuous air purge escaping through the restrictor."""
        
        # SAFETY CHECK: No leaking if valve is closed OR if system is in a vacuum!
        if self.drain_valve_closed or P_in_pa <= P_atm_pa:
            return 0.0 
            
        pr = P_atm_pa / P_in_pa
        gamma = 1.4
        critical_ratio = (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))
        
        # Calculate Mass Flow Rate leaking through the equivalent restrictor area
        if pr <= critical_ratio:
            # Choked (Sonic) Flow
            m_dot_leak = self.Cd_drain * self.drain_hole_area_m2 * P_in_pa * math.sqrt(gamma / (self.R_air * T_in_k)) * (2.0 / (gamma + 1.0)) ** ((gamma + 1.0) / (2.0 * (gamma - 1.0)))
        else:
            # Subsonic Flow
            m_dot_leak = self.Cd_drain * self.drain_hole_area_m2 * P_in_pa * math.sqrt((2.0 * gamma / (gamma - 1.0)) / (self.R_air * T_in_k) * (pr ** (2.0/gamma) - pr ** ((gamma + 1.0)/gamma)))
            
        return m_dot_leak

    def update_state(self, t, m_dot_in_kg_s, P_in_pa, T_in_k, T_amb_C, RH_amb, P_amb_pa=101325.0):
        """
        Master update function. Feed it the compressor output and room weather, 
        and it handles condensation, leakage, flow rates, and water separation.
        """
        # 1. PSYCHROMETRIC PHASE CHANGE (Condense the water)
        T_coil_C = T_in_k - 273.15
        liquid_water_in_mg_s, vapor_passed_mg_s, water_intake_mg_s = self._calculate_condensation(
            T_amb_C, RH_amb, P_amb_pa, T_coil_C, P_in_pa, m_dot_in_kg_s
        )

        # 2. AERODYNAMIC & LEAKAGE CALCULATIONS
        rho_in = P_in_pa / (self.R_air * T_in_k)
        
        # Calculate Normal Liters Per Minute (ANR) for the SMC specification
        Q_nlpm = (m_dot_in_kg_s / self.rho_normal) * 60000.0 if m_dot_in_kg_s > 0 else 0.0

        if rho_in > 0:
            dp_inertial = (self.beta_inertial * m_dot_in_kg_s**2) / rho_in
        else:
            dp_inertial = 0.0
            
        P_bowl_pa = max(0.0, P_in_pa - dp_inertial)
        m_dot_leak_kg_s = self._calculate_leakage(P_bowl_pa, T_in_k, P_amb_pa)
        leak_nlpm = (m_dot_leak_kg_s / 1.204) * 60000.0
        # if t % 100 ==0:
        #     print(f"Leakage Mass Flow_nlpm_water: {leak_nlpm:.6f} nlpm at P_out_water: {P_bowl_pa/100000:.3f} Bar abs, T_in_water: {T_in_k-273.15:.1f} °C")

        m_dot_effective = max(0.0, m_dot_in_kg_s - m_dot_leak_kg_s)
        
        # 3. CENTRIFUGAL WATER SEPARATION
        eta_current = self._calculate_efficiency(Q_nlpm)
        water_captured_mg_s = liquid_water_in_mg_s * eta_current
        water_escaped_mg_s = liquid_water_in_mg_s - water_captured_mg_s
        
        return P_bowl_pa, dp_inertial, m_dot_effective, water_escaped_mg_s, vapor_passed_mg_s
    

        # return {
        #     "P_out_pa": P_bowl_pa,                  
        #     "dp_pa": dp_inertial,                   
        #     "m_dot_out_kg_s": m_dot_effective,      
        #     "m_dot_leak_kg_s": m_dot_leak_kg_s,     
        #     "Q_nlpm": Q_nlpm,                       # Updated to output NLPM for SMC specs
        #     "efficiency": eta_current,              
        #     "water_intake_mg_s": water_intake_mg_s,
        #     "water_captured_mg_s": water_captured_mg_s, 
        #     "water_escaped_liquid_mg_s": water_escaped_mg_s,
        #     "water_escaped_vapor_mg_s": vapor_passed_mg_s
        # }

# ==========================================
# SYSTEM INTEGRATION TEST
# ==========================================
if __name__ == "__main__":
    print("=== DIGITAL TWIN: UNIFIED WATER SEPARATOR ===")
    
    # Instantiate the unified model
    separator = SMC_AFG20_WaterSeparator(valve_closed=False)
    
    # Variables passed from the Compressor/Tank and Room Sensors
    mass_flow = 0.002        # ~100 NLPM air flow
    P_coil_out = 400000.0    # 4 Bar Absolute
    T_coil_out = 36.0        # 36 deg C air hitting the separator
    T_ambient = 35.0         # 35 deg C Room
    RH_ambient = 0.95        # 95% Humidity Room
    
    # Single function call to process everything!
    results = separator.update_state(
        m_dot_in_kg_s = mass_flow,
        P_in_pa = P_coil_out,
        T_in_k = T_coil_out + 273.15,
        T_amb_C = T_ambient,
        RH_amb = RH_ambient
    )
    
    print("\n[FLUID DYNAMICS]")
    print(f" -> Flow Rate: {results['Q_nlpm']:.1f} NLPM")
    print(f" -> Pressure Drop: {results['dp_pa']/100:.2f} mBar")
    print(f" -> Valve Leak: {(results['m_dot_leak_kg_s']/1.204)*60000:.1f} NLPM")
    
    print("\n[PSYCHROMETRICS & SEPARATION]")
    print(f" -> Total Humidity Ingested: {results['water_intake_mg_s']:.1f} mg/s")
    print(f" -> Centrifugal Catch Rate:  {results['efficiency']*100:.2f}%")
    print(f" -> Liquid Water Captured:   {results['water_captured_mg_s']:.1f} mg/s")
    print(f" -> Droplets Escaping:       {results['water_escaped_liquid_mg_s']:.2f} mg/s")
    print(f" -> Vapor Escaping:          {results['water_escaped_vapor_mg_s']:.1f} mg/s")