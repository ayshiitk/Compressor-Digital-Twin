import math

class SMC_AF20_Filter:
    """
    Component Model: SMC AF20-F02-J-D 5-Micron Particulate Filter
    Governing Physics: Darcy-Forchheimer Porous Media Flow Equation & Choked Flow Leakage
    
    Features an integrated Cramer's Rule solver to dynamically compute 
    viscous and inertial structural coefficients from raw datasheet coordinates.
    """
    def __init__(self, point1_lpm = 300, point1_dp_mbar = 200, point2_lpm = 600, point2_dp_mbar = 700, p_cal_gauge_mpa=0.25, valve_closed=False):
        # Universal Constants
        self.R_air = 287.05               # Specific gas constant for air (J/kg*K)
        self.rho_anr = 1.204              # Standard air density baseline (kg/m3)
        self.T_cal_k = 293.15             # Datasheet standard calibration temperature (20°C)
        
        # Leakage Configuration ("J" Option Auto-Drain)
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
        A_theoretical =  2.165e-7
        self.drain_hole_area_m2 = A_theoretical

        self.Cd_drain = 1  # Standard discharge coefficient

        # Self-Calibrate coefficients from chart anchor inputs
        self.alpha_viscous, self.beta_inertial = self._execute_factory_calibration(
            point1_lpm, point1_dp_mbar, point2_lpm, point2_dp_mbar, p_cal_gauge_mpa
        )

    def _get_sutherland_viscosity(self, T_k):
        """Calculates temperature-dependent dynamic viscosity using Sutherland's Law."""
        return 1.716e-5 * ((T_k / 273.15)**1.5) * ((273.15 + 110.4) / (T_k + 110.4))

    def _execute_factory_calibration(self, q1, dp1, q2, dp2, p_gauge_mpa):
        """
        Solves the 2x2 simultaneous matrix equation system using Cramer's Rule
        to isolate alpha (viscous) and beta (inertial) coefficients from datasheet points.
        """
        # 1. Establish localized thermodynamic calibration states
        p_abs_pa = (p_gauge_mpa * 1000000.0) + 101325.0
        rho_cal = p_abs_pa / (self.R_air * self.T_cal_k)
        mu_cal = self._get_sutherland_viscosity(self.T_cal_k)
        
        # 2. Convert standard volumetric parameters to SI Mass Flows and Pascals
        m_dot1 = (q1 * self.rho_anr) / 60000.0
        dp1_pa = dp1 * 100.0
        
        m_dot2 = (q2 * self.rho_anr) / 60000.0
        dp2_pa = dp2 * 100.0
        
        # 3. Formulate the Matrix terms: [X]*alpha + [Y]*beta = [dP]
        X1 = mu_cal * m_dot1
        Y1 = (m_dot1 ** 2) / rho_cal
        
        X2 = mu_cal * m_dot2
        Y2 = (m_dot2 ** 2) / rho_cal
        
        # 4. Apply Cramer's Rule Determinants
        D_determinant = (X1 * Y2) - (X2 * Y1)
        
        if abs(D_determinant) < 1e-15:
            raise ValueError("Calibration Matrix Singular. Choose distinct data points from the curve.")
            
        alpha = ((dp1_pa * Y2) - (dp2_pa * Y1)) / D_determinant
        beta = ((X1 * dp2_pa) - (X2 * dp1_pa)) / D_determinant
        
        return alpha, beta

    # def _calculate_leakage(self, P_in_pa, T_in_k, P_atm_pa=101325.0):
    #     """Handles choked (sonic) vs subsonic leakage through the bowl drain hole."""
    #     if self.drain_valve_closed or P_in_pa <= P_atm_pa: 
    #         return 0.0 
            
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

    def calculate_filter_state(self, t,  m_dot_kg_s, P_in_pa, T_in_k, P_atm_pa=101325.0):
        """
        Evaluates current flow criteria to output total component pressure drop,
        leakage mass flow, and downstream delivery mass flow/pressure.
        """
        # If no flow and tank is empty, exit cleanly
        if m_dot_kg_s <= 1e-7 and P_in_pa <= P_atm_pa:
            return P_in_pa, 0.0, 0.0, 0.0, 0.0

        # 1. Calculate live operational state properties
        mu = self._get_sutherland_viscosity(T_in_k)
        rho = P_in_pa / (self.R_air * T_in_k)

        # 2. Apply Darcy-Forchheimer separation (All incoming air passes through the element)
        dp_viscous_pa = self.alpha_viscous * mu * m_dot_kg_s
        dp_inertial_pa = (self.beta_inertial * (m_dot_kg_s**2)) / rho

        total_dp_pa = dp_viscous_pa + dp_inertial_pa
        total_dp_mbar = total_dp_pa / 100.0
        
        # Calculate pressure inside the bowl after passing through the element
        P_out_pa = max(P_atm_pa, P_in_pa - total_dp_pa)

        # 3. Calculate bowl leakage (driven by the pressure after the element drop)
        m_dot_leak_kg_s = self._calculate_leakage(P_out_pa, T_in_k, P_atm_pa)
        leak_nlpm = (m_dot_leak_kg_s / 1.204) * 60000.0
        # if t % 100 ==0:
        #     print(f"Leakage Mass Flow_nlpm: {leak_nlpm:.6f} kg/s at P_out: {P_out_pa/100000:.3f} Bar abs, T_in: {T_in_k-273.15:.1f} °C")

        # 4. Calculate actual surviving mass flow to send downstream
        m_dot_out_kg_s = max(0.0, m_dot_kg_s - m_dot_leak_kg_s)

        return P_out_pa, total_dp_mbar, dp_viscous_pa / 100.0, dp_inertial_pa / 100.0, m_dot_out_kg_s
        # return P_out_pa, total_dp_mbar, dp_viscous_pa / 100.0, dp_inertial_pa / 100.0, m_dot_out_kg_s, m_dot_leak_kg_s

# =====================================================================
# CALIBRATED PERFORMANCE RUNNER
# =====================================================================
if __name__ == "__main__":
    # Input the exact raw coordinates derived from the 0.3 MPa curve in your datasheet
    # Point 1: 300 LPM @ 200 mBar (0.02 MPa)
    # Point 2: 600 LPM @ 700 mBar (0.07 MPa)
    filter_unit = SMC_AF20_Filter(
        point1_lpm=300.0, point1_dp_mbar=200.0,
        point2_lpm=600.0, point2_dp_mbar=700.0,
        p_cal_gauge_mpa=0.3,
        valve_closed=False  # Simulating the unvalved 1.8mm leak
    )
    
    print("=====================================================================")
    print("      SMC AF20 DARCY-FORCHHEIMER AUTO-CALIBRATION RESULTS            ")
    print("=====================================================================")
    print(f"Computed Viscous Matrix Alpha Factor (1/m): {filter_unit.alpha_viscous:.4e}")
    print(f"Computed Inertial Tortuosity Beta Factor (1/m): {filter_unit.beta_inertial:.4e}")
    print("=====================================================================\n")
    
    # Run simulation test case for your 100 NLPM target system layout
    mass_flow_gas = 0.002         # 100 NLPM flow rate
    p_inlet_pa = 387400.0         # Absolute pressure entering the filter housing
    t_inlet_k = 273.15 + 26.8     # Cooled air entering at 26.8°C
    
    p_out_pa, total_dp, dp_v, dp_i, m_dot_out, m_dot_leak = filter_unit.calculate_filter_state(mass_flow_gas, p_inlet_pa, t_inlet_k)
    
    # Convert masses back to NLPM for easy reading
    inlet_nlpm = (mass_flow_gas / filter_unit.rho_anr) * 60000.0
    out_nlpm = (m_dot_out / filter_unit.rho_anr) * 60000.0
    leak_nlpm = (m_dot_leak / filter_unit.rho_anr) * 60000.0
    
    print("=======================================================")
    print("           SIMULATED CURRENT FILTER STATE              ")
    print("=======================================================")
    print(f"Inlet Line Pressure         : {p_inlet_pa / 100000.0:.3f} Bar absolute")
    print(f"Operational Gas Temperature : {t_inlet_k - 273.15:.1f} °C")
    print("-" * 55)
    print(f"Viscous Shear Loss Element  : {dp_v:.3f} mBar")
    print(f"Inertial Turbulent Path Loss: {dp_i:.3f} mBar")
    print(f"TOTAL CLEAN ASSEMBLY DROP   : {total_dp:.3f} mBar")
    print("-" * 55)
    print(f"Absolute Pressure Leaving   : {p_out_pa / 100000.0:.4f} Bar absolute")
    print("-" * 55)
    print(f"Flow Requested (Inlet)      : {inlet_nlpm:.1f} NLPM")
    print(f"Lost through 1.8mm Leak     : {leak_nlpm:.1f} NLPM")
    print(f"Surviving Flow (Delivered)  : {out_nlpm:.1f} NLPM")
    print("=======================================================")