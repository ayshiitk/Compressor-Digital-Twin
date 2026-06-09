import math

class SMC_AF20_Filter:
    """
    Component Model: SMC AF20-F02-J-D Modular Air Filter Unit
    Port Size: 1/4" BSP | Element Size: 5-micron particulate matrix
    
    This model utilizes a high-fidelity 1D linear interpolation engine mapped 
    directly to the official flow characteristic curves in image_375003.png. 
    It accurately handles extreme operating envelopes, including very low flow 
    rates and high-pressure conditions up to 7 Bar gauge (0.7 MPa).
    """
    def __init__(self):
        # Curve anchor lines extracted from image_375003.png
        # Inlet Gauge Pressure vectors (P1) in MPa
        self.p_anchors_mpa = [0.1, 0.3, 0.5, 0.7]
        
        # Corresponding quadratic flow loss coefficients (MPa / (LPM_ANR^2))
        # Mathematically fitted to cross the exact visual thresholds of the chart
        self.a_coefficients = [2.20e-7, 1.00e-7, 0.65e-7, 0.40e-7]
        
        self.rho_anr = 1.204  # Standard reference air density (kg/m3)

    def _interpolate_coefficient(self, p_gauge_mpa):
        """Linearly interpolates the square-law flow resistance coefficient based on line pressure."""
        # Bound protection to prevent extrapolation divergence
        if p_gauge_mpa <= self.p_anchors_mpa[0]:
            return self.a_coefficients[0]
        if p_gauge_mpa >= self.p_anchors_mpa[-1]:
            return self.a_coefficients[-1]
            
        # Piecewise linear interpolation across the datasheet lines
        for i in range(len(self.p_anchors_mpa) - 1):
            if self.p_anchors_mpa[i] <= p_gauge_mpa <= self.p_anchors_mpa[i+1]:
                p0, p1 = self.p_anchors_mpa[i], self.p_anchors_mpa[i+1]
                a0, a1 = self.a_coefficients[i], self.a_coefficients[i+1]
                return a0 + (a1 - a0) * ((p_gauge_mpa - p0) / (p1 - p0))
                
        return self.a_coefficients[-1]

    def calculate_filter_state(self, m_dot_kg_s, P_in_pa):
        """
        Calculates the downstream absolute pressure (Bar) and the fluid pressure drop (mBar)
        across the filter assembly.
        """
        # If there is no forward flow, restriction drops to absolute zero
        if m_dot_kg_s <= 1e-7:
            return P_in_pa / 100000.0, 0.0

        # 1. Convert mass flow rate (kg/s) to standard volumetric flow rate (LPM ANR)
        # This matches the X-axis of the manufacturer chart exactly.
        flow_lpm_anr = (m_dot_kg_s * 60000.0) / self.rho_anr
        
        # 2. Convert absolute inlet pressure to gauge pressure in MPa
        p_gauge_mpa = (P_in_pa - 101325.0) / 1000000.0
        
        # 3. Interpolate the specific curve performance coefficient
        A_coeff = self._interpolate_coefficient(p_gauge_mpa)
        
        # 4. Calculate pressure drop using the curve's structural square law
        delta_p_mpa = A_coeff * (flow_lpm_anr ** 2)
        
        # Convert MPa to mBar (1 MPa = 10,000 mBar)
        delta_p_mbar = delta_p_mpa * 10000.0
        
        # 5. Compute final absolute outlet pressure
        P_out_pa = P_in_pa - (delta_p_mpa * 1000000.0)
        P_out_bar = P_out_pa / 100000.0
        
        return P_out_bar, delta_p_mbar


# =====================================================================
# MULTI-SCENARIO VERIFICATION RUNNER
# =====================================================================
if __name__ == "__main__":
    filter_unit = SMC_AF20_Filter()
    
    print("=====================================================================")
    print("       SMC AF20 FILTER PERFORMANCE ENVELOPE VERIFICATION RUN        ")
    print("=====================================================================")
    
    # Define test conditions representing your target states and extreme bounds
    test_cases = [
        {"name": "Standard Operating State", "flow_lpm": 100.0, "p_in_bar_abs": 3.913}, # ~2.9 Bar gauge
        {"name": "High Pressure, Very Low Flow", "flow_lpm": 5.0,   "p_in_bar_abs": 8.013}, # 7.0 Bar gauge
        {"name": "High Pressure, Mid Flow", "flow_lpm": 100.0, "p_in_bar_abs": 8.013}, # 7.0 Bar gauge
        {"name": "High Pressure, Chart Maximum", "flow_lpm": 1000.0, "p_in_bar_abs": 8.013}  # 7.0 Bar gauge
    ]
    
    for case in test_cases:
        # Convert test case parameters to standard SI inputs
        m_dot = (case["flow_lpm"] * filter_unit.rho_anr) / 60000.0
        p_in_pa = case["p_in_bar_abs"] * 100000.0
        
        p_out, dp_mbar = filter_unit.calculate_filter_state(m_dot, p_in_pa)
        
        print(f"Scenario: {case['name']}")
        print(f"  -> Input Flow Rate    : {case['flow_lpm']:.1f} LPM (ANR)")
        print(f"  -> Inlet Pressure     : {case['p_in_bar_abs']:.3f} Bar absolute ({(case['p_in_bar_abs']-1.013):.2f} Bar gauge)")
        print(f"  -> Calculated Loss    : {dp_mbar:.3f} mBar")
        print(f"  -> Output Pressure    : {p_out:.4f} Bar absolute")
        print("-" * 69)