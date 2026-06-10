import math

class SMC_AF20_Filter:
    """
    Component Model: SMC AF20-F02-J-D 5-Micron Particulate Filter
    Governing Physics: Darcy-Forchheimer Porous Media Flow Equation
    
    Features an integrated Cramer's Rule solver to dynamically compute 
    viscous and inertial structural coefficients from raw datasheet coordinates.
    """
    def __init__(self, point1_lpm, point1_dp_mbar, point2_lpm, point2_dp_mbar, p_cal_gauge_mpa=0.3):
        # Universal Constants
        self.R_air = 287.05               # Specific gas constant for air (J/kg*K)
        self.rho_anr = 1.204              # Standard air density baseline (kg/m3)
        self.T_cal_k = 293.15             # Datasheet standard calibration temperature (20°C)
        
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

    def calculate_filter_state(self, m_dot_kg_s, P_in_pa, T_in_k):
        """
        Evaluates current flow criteria to output total component pressure drop (mBar)
        and subsequent downstream delivery pressure (Bar absolute).
        """
        if m_dot_kg_s <= 1e-7:
            return P_in_pa / 100000.0, 0.0, 0.0, 0.0

        # Calculate live operational state properties
        mu = self._get_sutherland_viscosity(T_in_k)
        rho = P_in_pa / (self.R_air * T_in_k)

        # Apply Darcy-Forchheimer separation
        dp_viscous_pa = self.alpha_viscous * mu * m_dot_kg_s
        dp_inertial_pa = (self.beta_inertial * (m_dot_kg_s**2)) / rho

        total_dp_pa = dp_viscous_pa + dp_inertial_pa
        total_dp_mbar = total_dp_pa / 100.0
        
        P_out_bar = (P_in_pa - total_dp_pa) / 100000.0

        return P_out_bar, total_dp_mbar, dp_viscous_pa / 100.0, dp_inertial_pa / 100.0


# =====================================================================
# CALIBRATED PERFORMANCE RUNNER
# =====================================================================
if __name__ == "__main__":
    # Input the exact raw coordinates derived from the 0.3 MPa curve in image_375003.png
    # Point 1: 300 LPM @ 200 mBar (0.02 MPa)
    # Point 2: 600 LPM @ 700 mBar (0.07 MPa)
    filter_unit = SMC_AF20_Filter(
        point1_lpm=300.0, point1_dp_mbar=200.0,
        point2_lpm=600.0, point2_dp_mbar=700.0,
        p_cal_gauge_mpa=0.3
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
    
    p_out, total_dp, dp_v, dp_i = filter_unit.calculate_filter_state(mass_flow_gas, p_inlet_pa, t_inlet_k)
    
    print("=======================================================")
    print("          SIMULATED CURRENT VENTILATOR STATE           ")
    print("=======================================================")
    print(f"Inlet Line Pressure         : {p_inlet_pa / 100000.0:.3f} Bar absolute")
    print(f"Operational Gas Temperature : {t_inlet_k - 273.15:.1f} °C")
    print("-" * 55)
    print(f"Viscous Shear Loss Element  : {dp_v:.3f} mBar")
    print(f"Inertial Turbulent Path Loss: {dp_i:.3f} mBar")
    print(f"TOTAL CLEAN ASSEMBLY DROP   : {total_dp:.3f} mBar")
    print("-" * 55)
    print(f"Absolute Pressure Leaving   : {p_out:.4f} Bar absolute")
    print("=======================================================")