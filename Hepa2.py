import numpy as np

class ZF111_HEPA_Filter:
    """
    Component Model: ZF-111 Intake HEPA Filter.
    Stateless model calculating pressure drop dynamically based on usage hours.
    Supports variable flow data points and dynamic clamping per age dataset.
    """
    def __init__(self, usage_hours):
        self.usage_hours = usage_hours
        self.component_name = "ZF-111 HEPA Filter"
        self.housing_material = "ABS"
        self.media = "Hydrophobic Glass fiber"
        self.connector = "3/8 NPT"
        
        # --- DATASET TABLE ---
        # Map the usage hours to a tuple of (Flows_Array, Resistances_Array).
        # You can now have completely different flow test points for each age.
        self.performance_data_by_hours = {
            0: (
                np.array([0, 30, 60, 90]),        # Flows for 0 hours
                np.array([0, 71, 1120, 3236])     # Resistances for 0 hours
            ),  
            500: (
                np.array([0, 20, 50, 80, 100]),   # Different flow points for 500 hours
                np.array([0, 80, 1500, 3800, 4800])
            ), 
            1000: (
                np.array([0, 25, 50, 75]),        # Only tested up to 75 L/min for 1000 hours
                np.array([0, 300, 2500, 5500])  
            ),
            1500: (
                np.array([0, 25, 50, 75]),        # Only tested up to 75 L/min for 1000 hours
                np.array([0, 300, 2500, 5500])  
            )
        }

        # Cache sorted hours for quick lookup during simulation steps
        self.available_hours = sorted(self.performance_data_by_hours.keys())
        
        # --- PRE-CALCULATE COEFFICIENTS ---
        # Generate polynomials and cache the maximum valid flow for each dataset.
        self.coeffs_by_hours = {}
        self.max_valid_flow_by_hours = {}
        
        for hours, (flows, resistances) in self.performance_data_by_hours.items():
            # Ensure arrays are sorted by flow (just in case they are entered out of order)
            sort_idx = np.argsort(flows)
            sorted_flows = flows[sort_idx]
            sorted_resistances = resistances[sort_idx]
            
            # Fit the curve using the specific flow points for this age
            self.coeffs_by_hours[hours] = np.polyfit(sorted_flows, sorted_resistances, 2)
            
            # Store the highest flow point in this specific dataset to cap extrapolation
            self.max_valid_flow_by_hours[hours] = sorted_flows[-1]

    def pressure_drop(self, flow_lpm):
        """
        Calculates the exact pressure drop (Pa) for a given flow rate (L/min) 
        and specific usage hours.
        """
        # 1. Determine which dataset to use based on the requested hours
        selected_hours = self.available_hours[0]
        for h in self.available_hours:
            if self.usage_hours >= h:
                selected_hours = h
            else:
                break
                
        # 2. Retrieve the pre-calculated coefficients and the maximum valid flow
        active_coeffs = self.coeffs_by_hours[selected_hours]
        max_flow_limit = self.max_valid_flow_by_hours[selected_hours]

        # 3. Clamp flow to the max limit of THIS SPECIFIC dataset
        flow_lpm_for_curve = min(flow_lpm, max_flow_limit)

        # 4. Calculate pressure drop using the empirical polynomial curve
        dp = np.polyval(active_coeffs, flow_lpm_for_curve)
        
        # 5. Ensure pressure drop doesn't dip below zero
        return max(0.0, dp)

# --- Unit Test / Implementation Example ---
if __name__ == "__main__":
    hepa = ZF111_HEPA_Filter()
    
    print(f"Testing {hepa.component_name} Dynamic Function Calls:\n")
    
    test_cases = [
        {"flow": 60, "hours": 0},
        {"flow": 60, "hours": 500},
        {"flow": 95, "hours": 0},    # Capped at 90 L/min internally
        {"flow": 95, "hours": 500},  # Valid up to 100 L/min, won't cap
        {"flow": 95, "hours": 1000}, # Capped at 75 L/min internally
    ]
    
    for case in test_cases:
        q = case["flow"]
        h = case["hours"]
        dp = hepa.pressure_drop(flow_lpm=q, usage_hours=h)
        print(f"Input -> Flow: {q:2} L/min | Hours: {h:4} || Result -> DP: {dp:.2f} Pa")