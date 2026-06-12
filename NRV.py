import math

class SMC_AKH10_NRV:
    """
    Component Model: SMC AKH10-00 Non-Return / Check Valve
    Fittings: 10mm One-touch push-in type
    Governing Law: ISO 6358 Pneumatic Fluid Power Flow Standard
    """
    def __init__(self):
        # Official SMC Manufacturer Specifications
        self.C_sonic_conductance = 4.8      # dm3/(s*bar)
        self.b_critical_pressure_ratio = 0.5 # Subsonic/Sonic threshold index
        self.P_crack_pa = 5000.0            # Cracking Pressure: 0.005 MPa = 50 mBar
        self.rho_anr = 1.204                # Standard reference air density (kg/m3)

    def calculate_valve_state(self, m_dot_kg_s, P_in_pa, T_in_k):
        """
        Calculates the absolute outlet pressure (Bar) and the total pressure drop (mBar)
        across the valve using the inverse ISO 6358 pneumatic standard equations.
        """
        # If there is no forward mass flow, the check valve remains closed
        if m_dot_kg_s <= 0.00001:
            return P_in_pa / 100000.0, 0.0

        P_in_bar = P_in_pa / 100000.0
        rho_anr_dm3 = self.rho_anr / 1000.0
        
        # 1. Calculate maximum sonic mass flow capacity boundary (Choked Limit)
        m_dot_max = (self.C_sonic_conductance * P_in_bar * rho_anr_dm3 * 
                     math.sqrt(293.15 / T_in_k))
        
        # 2. Map the flow regime via ISO 6358 standard
        if m_dot_kg_s >= m_dot_max:
            # Flow has reached Mach 1 (Choked)
            P_out_restriction_pa = P_in_pa * self.b_critical_pressure_ratio
        else:
            # Flow is subsonic (Unchoked)
            flow_ratio = m_dot_kg_s / m_dot_max
            pressure_ratio = self.b_critical_pressure_ratio + (1.0 - self.b_critical_pressure_ratio) * math.sqrt(1.0 - flow_ratio**2)
            P_out_restriction_pa = P_in_pa * pressure_ratio

        # 3. Factor in the mechanical spring cracking pressure restriction
        P_out_pa = P_out_restriction_pa - self.P_crack_pa
        
        # Formulate final output telemetry
        delta_p_mbar = (P_in_pa - P_out_pa) / 100.0
        # P_out_bar = P_out_pa / 100000.0

        return P_out_pa, delta_p_mbar, T_in_k


# =====================================================================
# STANDALONE MODULE TEST BLOCK
# =====================================================================
if __name__ == "__main__":
    nrv = SMC_AKH10_NRV()
    
    # Test conditions (E.g., 100 NLPM flow, 4.0 Bar abs inlet, 25°C temperature)
    test_mass_flow = 0.002       # kg/s
    test_p_in_pa = 400000.0      # Pascals (4.0 Bar absolute)
    test_t_in_k = 273.15 + 25.0  # Kelvin
    
    p_out, dp_mbar = nrv.calculate_valve_state(test_mass_flow, test_p_in_pa, test_t_in_k)
    
    print("=== STANDALONE SMC AKH10 NRV MODULE TEST ===")
    print(f"Inlet Pressure  : {test_p_in_pa / 100000.0:.3f} Bar absolute")
    print(f"Pressure Drop   : {dp_mbar:.1f} mBar")
    print(f"Outlet Pressure : {p_out:.3f} Bar absolute")