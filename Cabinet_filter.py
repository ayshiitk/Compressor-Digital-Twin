import numpy as np

def calculate_filter_pressure_drop(flow_rate_cfm, width_mm=91.0, height_mm=91.0):
    """
    Calculates the pressure drop across a louvered IP54 filter assembly.
    
    Parameters:
    flow_rate_cfm (float): Volumetric flow rate in Cubic Feet per Minute.
    width_mm (float): Cut-out width in millimeters (Default JSAV-10: 91mm).
    height_mm (float): Cut-out height in millimeters (Default JSAV-10: 91mm).
    
    Returns:
    float: Total pressure drop in Pascals (Pa).
    """
    # --- 1. Physical Constants ---
    rho_air = 1.225    # Density of air at standard conditions (kg/m^3)
    mu_air = 1.81e-5   # Dynamic viscosity of air (Pa*s)
    
    # --- 2. Area Calculations ---
    # Convert mm to meters
    area_m2 = (width_mm / 1000) * (height_mm / 1000)
    
    # The plastic louvers block part of the opening. 
    # We estimate a 70% open area for the JSAV-10 grill.
    effective_area_m2 = area_m2 * 0.70 
    
    # --- 3. Filter Media Properties (Estimated for IP54) ---
    filter_thickness_m = 0.002   # Estimated thickness: 2.0 mm
    permeability_k = 2.5e-9      # Permeability of dense non-woven polyester (m^2)
    inertial_coeff = 15.0        # Form drag coefficient for louvers/grill
    
    # --- 4. Fluid Dynamics ---
    # Convert CFM to cubic meters per second (m^3/s)
    flow_rate_m3s = flow_rate_cfm * 0.0004719474
    
    # Calculate superficial velocity (v = Q / A)
    velocity = flow_rate_m3s / effective_area_m2
    
    # Darcy-Forchheimer Equation
    # Viscous term (linear): Flow through the fabric pores
    dp_viscous = (mu_air * filter_thickness_m / permeability_k) * velocity
    
    # Inertial term (quadratic): Flow colliding with the plastic louvers
    dp_inertial = 0.5 * rho_air * inertial_coeff * (velocity**2)
    
    # Total pressure drop
    total_dp_pa = dp_viscous + dp_inertial
    
    return total_dp_pa

# ==========================================
# Input your known Flow Rate here
# ==========================================
my_flow_rate_cfm = 25.0  # Example: 25 CFM

# Calculate and print the result
pressure_drop = calculate_filter_pressure_drop(my_flow_rate_cfm)

print(f"Flow Rate: {my_flow_rate_cfm} CFM")
print(f"Estimated Pressure Drop: {pressure_drop:.2f} Pascals")
# Note: To convert Pascals to mmH2O (a common fan spec), divide by 9.806
print(f"Equivalent in mmH2O: {(pressure_drop / 9.806):.2f} mmH2O")