import numpy as np

class ZF111_HEPA_Filter:
    """
    Component Model: ZF-111 Intake HEPA Filter.
    Utilizes empirical datasheet points to fit a non-linear pressure drop curve,
    accounting for the severe inertial losses of the 3/8" NPT connector at high flows.
    """
    def __init__(self):
        self.component_name = "ZF-111 HEPA Filter"
        self.housing_material = "ABS"
        self.media = "Hydrophobic Glass fiber"
        self.connector = "3/8 NPT"
        
        # Datasheet Performance Points [Flow in L/min, Resistance in Pa]
        # Including (0,0) as a physical baseline
        self.known_flows_lpm = np.array([0, 30, 60, 90])
        self.known_resistance_pa = np.array([0, 71, 1120, 3236])
        # self.known_flows_lpm       = np.array([0,  30,   50,   60,   70,   85,   100])
        # self.known_resistance_pa   = np.array([0, 2000, 3000, 4000, 5000, 6000, 6000])
        
        # Generate 2nd-order polynomial coefficients [C1, C2, C3]
        self.flow_curve_coeffs = np.polyfit(self.known_flows_lpm, self.known_resistance_pa, 2)
        
        # State variables
        self.loading = 0.0          # 0.0 = New, 1.0 = Clogged
        self.is_fractured = False
        self.clog_rate = 1e-7       # Arbitrary loading increment per time-step
        
        # Reciprocating pump suction pulse threshold (Pa gauge)
        self.fracture_threshold_pa = 5000 

    def pressure_drop(self, flow_lpm):
        """
        Calculates the exact pressure drop (Pa) for a given flow rate (L/min) 
        based on the datasheet curve fit and current filter loading state.
        """
        if self.is_fractured:
            return 0.0  # Media ruptured; no flow resistance
            
        # Calculate clean pressure drop using the empirical polynomial curve
        dp_clean = np.polyval(self.flow_curve_coeffs, flow_lpm)
        
        # Ensure pressure drop doesn't dip below zero due to polynomial fit anomalies near 0
        dp_clean = max(0.0, dp_clean)
        
        # Apply loading penalty. 
        # Assuming a fully clogged filter (1.0) increases resistance by a factor of 5.
        dp_loaded = dp_clean * (1.0 + 4.0 * self.loading)
        
        return dp_loaded

    def update_loading(self, flow_lpm, dt_s):
        """Advances the loading state based on flow volume over time."""
        volume_passed_liters = flow_lpm * (dt_s / 60.0)
        self.loading = min(1.0, self.loading + (self.clog_rate * volume_passed_liters))

    def check_pulse_fracture(self, peak_suction_pa):
        """Simulates catastrophic failure if reciprocating pump pulls too hard."""
        if peak_suction_pa > self.fracture_threshold_pa:
            self.is_fractured = True
            print(f"WARNING: {self.component_name} media fractured due to excess suction pulse!")

# --- Unit Test ---
if __name__ == "__main__":
    hepa = ZF111_HEPA_Filter()
    
    print(f"Testing {hepa.component_name} Model:")
    test_flows = [15, 30, 60, 90, 100]
    
    for q in test_flows:
        dp = hepa.pressure_drop(q)
        print(f"Flow: {q:3} L/min | Resistance: {dp:.2f} Pa")