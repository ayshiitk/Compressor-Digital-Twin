import numpy as np

class AcousticSilencerChamber:
    """
    Component Model: 2nd-Order Acoustic Dampener & Volume Buffer.
    Models fluid inertia (L), sudden expansion damping (R), and volume capacitance (C)
    to accurately simulate the attenuation of high-frequency pneumatic pulses.
    """
    def __init__(self, chamber_vol_liters=.5640, inlet_pipe_dia_mm=9.0, inlet_pipe_len_mm=23.0):
        self.component_name = "SS Acoustic Silencer"
        
        # --- Thermodynamics Constants ---
        self.R_air = 287.05           # Gas constant for air (J/(kg*K))
        self.T_K = 293.15             # Isothermal temperature (K)
        self.P_atm = 101325.0         # Atmospheric pressure (Pa)
        
        # --- Geometries ---
        self.V_m3 = chamber_vol_liters / 1000.0 # ID - 114.3 , Length - 55.0 mm, Volume - 564 mL
        self.A_pipe = np.pi * ((inlet_pipe_dia_mm / 2000.0) ** 2) # Cross-sectional area (m^2)
        self.L_pipe = inlet_pipe_len_mm / 1000.0                  # Length (m)
        self.K_expansion = 1.0        # Borda-Carnot loss coefficient for sudden expansion
        
        # --- State Variables ---
        self.pressure_pa = self.P_atm # Chamber pressure
        self.mass_kg = (self.pressure_pa * self.V_m3) / (self.R_air * self.T_K)
        self.m_dot_in = 0.0           # Current mass flow rate entering chamber (kg/s)
    

    def update_state(self, P_upstream_pa, m_dot_out_kg_s, dt_s):
        """
        Advances the chamber state using stable macro-physics.
        """
        # Calculate upstream air density
        rho_up = P_upstream_pa / (self.R_air * self.T_K)
        
        # 1. Stable Resistance (Damping)
        # Instead of integrating acceleration (which explodes at dt=0.01),
        # we calculate the exact velocity required to balance the pressure difference.
        dp_available = P_upstream_pa - self.pressure_pa
        
        if dp_available >= 0:
            # Normal Flow: Room -> Silencer
            velocity = np.sqrt((2.0 * dp_available) / (self.K_expansion * rho_up))
            self.m_dot_in = rho_up * self.A_pipe * velocity
        else:
            # Reverse Flow (Prevents math crashes if pressure spikes)
            velocity = np.sqrt((2.0 * abs(dp_available)) / (self.K_expansion * rho_up))
            self.m_dot_in = -rho_up * self.A_pipe * velocity
            
        # Mathematical Safety Clamp: 
        # Prevent the flow from over-filling the chamber in a single 0.01s frame
        max_mass_change = (abs(dp_available) * self.V_m3) / (self.R_air * self.T_K)
        self.m_dot_in = np.clip(self.m_dot_in, -max_mass_change/dt_s, max_mass_change/dt_s)

        # 2. Volume Buffer (Capacitance)
        # Integrate net mass flow to update total mass in chamber
        net_mass_flow = self.m_dot_in - m_dot_out_kg_s
        self.mass_kg += net_mass_flow * dt_s
        self.mass_kg = max(1e-9, self.mass_kg) # Prevent physical impossibilities
        
        # 3. Update internal pressure isothermally
        self.pressure_pa = (self.mass_kg * self.R_air * self.T_K) / self.V_m3
        
        return self.pressure_pa, self.m_dot_in


    # def update_state(self, P_upstream_pa, m_dot_out_kg_s, dt_s):
        """
        Advances the acoustic chamber state by one time-step.
        
        Inputs:
        P_upstream_pa  : Pressure directly after the HEPA filter (Pa absolute)
        m_dot_out_kg_s : Mass flow rate pulled out by the compressor pump (kg/s)
        dt_s           : Solver time-step (seconds)
        """
        # Calculate upstream air density
        rho_up = P_upstream_pa / (self.R_air * self.T_K)
        
        # 1. Acoustic Damping (Resistance)
        # Prevent division by zero and handle flow direction math safely
        velocity = self.m_dot_in / (rho_up * self.A_pipe) if rho_up > 0 else 0
        dp_damp = self.K_expansion * 0.5 * rho_up * (velocity ** 2)
        
        # Directional sign for drag (resists current flow direction)
        dp_damp = np.copysign(dp_damp, self.m_dot_in)
        
        # 2. Fluid Inertia (Inductance)
        # Calculate acceleration of the air column
        # d(m_dot)/dt = (A / L) * (Driving Pressure - Resisting Pressure)
        force_balance_pa = P_upstream_pa - self.pressure_pa - dp_damp
        m_dot_derivative = (self.A_pipe / self.L_pipe) * force_balance_pa
        
        # Integrate acceleration to update incoming mass flow
        self.m_dot_in += m_dot_derivative * dt_s
        
        # 3. Volume Buffer (Capacitance)
        # Integrate net mass flow to update total mass in chamber
        net_mass_flow = self.m_dot_in - m_dot_out_kg_s
        self.mass_kg += net_mass_flow * dt_s
        self.mass_kg = max(1e-9, self.mass_kg) # Prevent physical impossibilities
        
        # Update internal pressure isothermally
        self.pressure_pa = (self.mass_kg * self.R_air * self.T_K) / self.V_m3
        
        return self.pressure_pa, self.m_dot_in

    def get_gauge_pressure(self):
        """Returns the pressure relative to atmosphere."""
        return self.pressure_pa - self.P_atm

# --- Unit Test ---
# if __name__ == "__main__":
#     silencer = AcousticSilencerChamber()
    
#     # Simulate a sudden 3000 Pa suction pulse from the pump, 
#     # while the HEPA filter upstream remains at atmospheric pressure.
#     upstream_p = 101325.0 
#     pump_pulling_mass = 0.005 # kg/s
#     dt = 0.0001                # 1 ms time-step required for acoustic frequencies
    
#     print("Time(ms) | Inlet Flow (kg/s) | Chamber Pressure (Pa Gauge)")
#     print("-" * 55)
#     for step in range(1, 16):
#         p_chamber, m_in = silencer.update_state(upstream_p, pump_pulling_mass, dt)
#         print(f" {step * dt * 1000:5.1f}   |      {m_in:.6f}      |    {silencer.get_gauge_pressure():.2f}")