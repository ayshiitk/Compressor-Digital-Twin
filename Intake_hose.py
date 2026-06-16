import numpy as np
import math

class IntakeHose_HiPoFlex:
    """
    Component Model: HiPoFlex Transparent Braided Silicone Hose
    Dimensions: 9x16mm, 600mm Length, 3x 90-degree bends.
    Calculates combined fluid dynamic pressure drop and thermodynamic heat loss.
    """
    def __init__(self):
        # 1. Hose Physical Dimensions
        self.L = 0.600             # Length in meters (600 mm)
        self.D_in = 0.009          # Inner Diameter in meters (9 mm)
        self.D_out = 0.016         # Outer Diameter in meters (16 mm)
        
        # 2. Material & Flow Geometry
        self.k_silicone = 0.25     # Thermal conductivity of silicone (W/m*K)
        self.num_90_bends = 3      # Three 90-degree turns
        self.K_per_bend = 0.25     # Minor loss coefficient for a sweeping 50mm radius bend
        self.h_out = 7.0           # Natural convection coefficient for still room air (W/m2*K)
        
        # 3. Static Air Properties
        self.R_air = 287.05        # Gas constant for air (J/kg*K)
        self.cp_air = 1009.0       # Specific heat of air (J/kg*K)

    def _get_dynamic_air_properties(self, T_k):
        """
        Dynamically calculates air viscosity, thermal conductivity, and Prandtl number 
        based on the exact live temperature of the air.
        """
        # Sutherland's Law for Dynamic Viscosity (mu)
        mu_ref = 1.716e-5
        T_ref = 273.15
        S = 110.4
        mu = mu_ref * ((T_k / T_ref)**1.5) * ((T_ref + S) / (T_k + S))
        
        # Linear approximation for Thermal Conductivity of Air (k_air)
        T_celsius = T_k - 273.15
        k_air = 0.0241 + (7.3e-5 * T_celsius)
        
        # Dynamic Prandtl Number (Pr = mu * Cp / k)
        Pr = (mu * self.cp_air) / k_air
        
        return mu, k_air, Pr

    def calculate_hose_state(self, m_dot_kg_s, P_in_pa, T_in_k, T_amb_k):
        """
        Passes hot compressed air through the 600mm hose, calculating exactly 
        how much pressure is destroyed and how much heat escapes.
        """
        # If the compressor is completely off, return ambient/static conditions
        if m_dot_kg_s <= 0.0001:
            return P_in_pa, T_amb_k

        # Fetch live air properties based on incoming temperature
        mu_air, k_air, Pr_air = self._get_dynamic_air_properties(T_in_k)

        # ==========================================
        # PART 1: FLUID DYNAMICS (Pressure Drop)
        # ==========================================
        # Step 1: Flow Velocity
        area_in = math.pi * (self.D_in / 2.0)**2
        rho_in = P_in_pa / (self.R_air * T_in_k)  # Density at 4 Bar is ~4x normal
        velocity = m_dot_kg_s / (rho_in * area_in)
        
        # Step 2: Reynolds Number (Turbulence)
        Re = (rho_in * velocity * self.D_in) / mu_air
        
        # Step 3: Friction Factor (Blasius Equation for Smooth Silicone Tubes)
        if Re > 4000:
            f = 0.316 / (Re**0.25)
        else:
            f = max(0.02, 64.0 / max(1, Re)) # Fallback if flow is slow/laminar
            
        # Step 4: Pressure Drop (Major + Minor Losses)
        dyn_pressure = 0.5 * rho_in * velocity**2
        
        delta_p_straight = f * (self.L / self.D_in) * dyn_pressure
        delta_p_bends = (self.num_90_bends * self.K_per_bend) * dyn_pressure
        
        P_out_pa = P_in_pa - (delta_p_straight + delta_p_bends)

        # ==========================================
        # PART 2: THERMODYNAMICS (Heat Loss via NTU)
        # ==========================================
        # Hurdle 1: Internal Convection Resistance (Dittus-Boelter Equation)
        Nu_in = 0.023 * (Re**0.8) * (Pr_air**0.3)
        h_in = (Nu_in * k_air) / self.D_in
        R_in = 1.0 / (h_in * math.pi * self.D_in * self.L)
        
        # Hurdle 2: Wall Conduction Resistance (Cylindrical Thick Wall)
        R_wall = math.log(self.D_out / self.D_in) / (2.0 * math.pi * self.k_silicone * self.L)
        
        # Hurdle 3: External Convection Resistance (Still Air / Natural Convection)
        R_out = 1.0 / (self.h_out * math.pi * self.D_out * self.L)
        
        # Total Resistance & Overall Heat Transfer Coefficient (U*A)
        R_total = R_in + R_wall + R_out
        UA = 1.0 / R_total
        
        # Exponential Decay (NTU Method)
        C_min = m_dot_kg_s * self.cp_air
        NTU = UA / C_min
        
        # Final Temperature hitting the cooling coil
        T_out_k = T_amb_k + (T_in_k - T_amb_k) * math.exp(-NTU)
        
        # Total Watts of heat radiated into the still room air
        heat_loss_watts = C_min * (T_in_k - T_out_k)

        return P_out_pa, T_out_k 
    


# =====================================================================
# SIMULATION EXECUTION (TEST BLOCK)
# =====================================================================
# if __name__ == "__main__":
#     # Initialize the finalized hose module
#     hose = DischargeHose_HiPoFlex()
    
#     # Simulating the air leaving the compressor head
#     test_m_dot = 0.002           # approx 90  NLPM flow
#     test_press_pa = 400000.0     # 4.0 Bar Absolute
#     test_temp_k = 273.15 + 95.0  # 95°C hot gas
#     test_amb_k = 273.15 + 25.0   # 25°C still room air
    
#     # Run the physics engine
#     p_final, t_final, watts_lost = hose.calculate_hose_state(
#         m_dot_kg_s=test_m_dot, 
#         P_in_pa=test_press_pa, 
#         T_in_k=test_temp_k, 
#         T_amb_k=test_amb_k
#     )
    
#     # Print the physical realities
#     print("=== HiPoFlex 600mm Hose Physics Results ===")
#     print(f"Compressor Output:  {test_press_pa/100000.0:.2f} Bar, {test_temp_k-273.15:.1f} °C")
#     print(f"Pressure Dropped:   {(test_press_pa - p_final)/100.0:.1f} mBar (Destroyed by friction/bends)")
#     print(f"Coil Arrival Pres:  {p_final/100000.0:.3f} Bar")
#     print("-" * 43)
#     print(f"Temp Dropped:       {(test_temp_k - t_final):.1f} °C (Insulated by 3.5mm wall/still air)")
#     print(f"Coil Arrival Temp:  {t_final - 273.15:.1f} °C")
#     print(f"Radiated Heat:      {watts_lost:.1f} Watts")