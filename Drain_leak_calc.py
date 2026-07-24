import math

class RestrictorFlowSolver:
    """
    A Fanno-flow and isentropic expansion solver for predicting mass flow 
    and volumetric flow (NLPM) through short tube restrictors.
    """
    def __init__(self, calibration_factor=.88, contraction_multiplier=0.3):
        # Fluid Constants (Air)
        self.k = 1.4
        self.R = 287.05
        self.mu = 1.81e-5
        self.P_normal = 101325.0
        self.T_normal = 273.15
        
        # System Constants (SI Units)
        self.Downstream_P_pa = 101325.0  # Pa, atmospheric pressure downstream
        self.D_upstream_mm = 6.0         # Upstream tube ID (mm)
        
        # Empirical Tuning Parameters (Based on physical test data)
        self.calibration_factor = calibration_factor
        self.contraction_multiplier = contraction_multiplier

    def _calculate_fanno_right_side(self, M1):
        """Internal helper: Calculates right side of Fanno equation."""
        term1 = (1 - M1**2) / (self.k * M1**2)
        term2 = ((self.k + 1) / (2 * self.k)) * math.log(((self.k + 1) * M1**2) / (2 + (self.k - 1) * M1**2))
        return term1 + term2

    def _solve_inlet_mach(self, fanno_parameter, tolerance=1e-6):
        """Internal helper: Bisection search for Inlet Mach Number."""
        low_M = 0.001 
        high_M = 0.999
        
        for _ in range(100):
            mid_M = (low_M + high_M) / 2.0
            calculated_fanno = self._calculate_fanno_right_side(mid_M)
            
            if calculated_fanno > fanno_parameter:
                low_M = mid_M
            else:
                high_M = mid_M
                
            if abs(calculated_fanno - fanno_parameter) < tolerance:
                break
                
        return mid_M

    def calculate_flow(self, D_restrictor_mm, P1_pa, T1_K, L_restrictor_mm=5.0):
        """
        Executes the iterative solver for the given geometric and operating parameters.
        Returns mass flow (kg/s) and Volumetric flow (NLPM).
        """
        # SAFETY GUARD: a 0mm (or negative) restrictor diameter has no physical
        # meaning here and previously caused a ZeroDivisionError deep in the
        # Fanno-flow solver below, silently killing the whole simulation.
        # Treat it as "orifice closed" -> zero leak flow.
        if D_restrictor_mm <= 0:
            return 0.0, 0.0

        # Unit Conversions
        D1 = self.D_upstream_mm / 1000.0
        D2 = D_restrictor_mm / 1000.0
        L = L_restrictor_mm / 1000.0
        
        # Safety clamp: Absolute pressure cannot physically be lower than downstream atmospheric
        # This completely prevents negative pressure ratios and the complex number TypeError
        P1 = max(P1_pa, self.Downstream_P_pa + 1.0)
        P2 = self.Downstream_P_pa
        
        A2 = (math.pi * D2**2) / 4.0
        beta = D2 / D1

        # Iterative Solver Setup
        f_guess = 0.025
        m_dot = 0.0
        effective_choking_P1_pa = 0.0
        is_choked = False
        
        for _ in range(6):
            # A. Fanno Analysis
            fanno_param = f_guess * (L / D2)
            M1 = self._solve_inlet_mach(fanno_param)
            
            # Dimensionless Pressure Ratios
            P1_to_Pstar = (1 / M1) * math.sqrt((self.k + 1) / (2 + (self.k - 1) * M1**2))
            P0_to_P1 = (1 + ((self.k - 1) / 2) * M1**2) ** (self.k / (self.k - 1))
            P0_to_Pstar = P0_to_P1 * P1_to_Pstar
            
            # Determine Choking Threshold in Pascals
            effective_choking_P1_pa = self.Downstream_P_pa * P0_to_Pstar
            is_choked = P1 >= effective_choking_P1_pa
            
            # B. Total Resistance (K)
            K_contraction = self.contraction_multiplier * (1 - beta**2)
            K_friction = f_guess * (L / D2)
            K_exit = 1.0
            K_total = K_contraction + K_friction + K_exit
            Cd = 1.0 / math.sqrt(K_total)
            
            # C. Mass Flow
            if is_choked:
                term1 = self.k / (self.R * T1_K)
                term2 = (2 / (self.k + 1)) ** ((self.k + 1) / (self.k - 1))
                m_dot = Cd * A2 * P1 * math.sqrt(term1 * term2)
            else:
                pr = P2 / P1
                term1 = (2 * self.k) / (self.R * T1_K * (self.k - 1))
                term2 = (pr ** (2 / self.k)) - (pr ** ((self.k + 1) / self.k))
                if term2 < 0: term2 = 0 
                m_dot = Cd * A2 * P1 * math.sqrt(term1 * term2)
                
            # D. Update Friction Factor
            Re = (4 * m_dot) / (math.pi * D2 * self.mu)
            if Re < 2300:
                f_guess = 64.0 / max(Re, 1)
            else:
                f_guess = 0.3164 / (Re ** 0.25)
                
        # Final Conversions & Calibration Application
        m_dot = m_dot * self.calibration_factor
        rho_normal = self.P_normal / (self.R * self.T_normal)
        NLPM = (m_dot / rho_normal) * 60000.0
        
        return m_dot, NLPM

# ==========================================
# TEST PARAMETERS & EXECUTION
# ==========================================
if __name__ == "__main__":
    # 1. Instantiate the solver
    solver = RestrictorFlowSolver(calibration_factor=1.14, contraction_multiplier=0.3)

    # 2. Define geometry and operating conditions
    restrictor_ID = 0.45   # mm
    
    # Updated to input Pascals natively for the test block
    upstream_pressure_pa = (2.2+ 1.0) * 101325.0  # 3.2 atm absolute converted to Pa
    temperature = 273.15 + 25 # K

    # 3. Run Simulation
    m_dot, nlpm = solver.calculate_flow(
        restrictor_ID, 
        upstream_pressure_pa,
        temperature
    )

    # 4. Print Results
    print(f"Mass Flow Rate: {m_dot:.6f} kg/s")
    print(f"Volume Flow Rate: {nlpm:.2f} NLPM\n")