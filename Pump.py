import numpy as np

class SmartCompressor_120RND_Empirical:
    """
    Component Model: G&M Tech 120RND-ED Double Head Pump.
    Features auto-calibration: Ingests raw physical test data (Temps, Amps, Pressures)
    and automatically calculates dynamic Thermal Resistance (Rth) curves on initialization.
    """
    def __init__(self):
        self.component_name = "120RND-ED Empirical Compressor"
        
        # --- 1. Thermodynamics & Mass Properties ---
        self.gamma = 1.4           
        self.rho_normal = 1.204    
        self.cp_air = 1005.0       
        
        self.Cth_motor = 1513.4  # J/K (Steel + Copper Stator)
        self.Cth_head = 1413.0   # J/K (Aluminum Heads)
        
        # State Variables
        self.T_ambient_k = 293.15
        self.T_motor_k = 293.15
        self.T_head_k = 293.15
        
        self.motor_limit_k = 273.15 + 75.0          
        self.motor_throttle_start_k = 273.15 + 65.0 
        self.head_target_k = 273.15 + 60.0          

        # --- 2. Manufacturer Empirical Pneumatic Curves ---
        p_bar = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        f_nlpm = np.array([133, 121, 108, 90, 79, 67, 60, 52])
        c_amps = np.array([9.9, 16.1, 20.8, 22.9, 24.0, 24.5, 24.9, 24.5])
        
        self.flow_curve = np.polyfit(p_bar, f_nlpm, 2)
        self.current_curve = np.polyfit(p_bar, c_amps, 3)

        # --- 3. Run Auto-Calibration ---
        self.motor_Rth_curve, self.head_Rth_curve = self._auto_calibrate_thermal_resistance()

    def _auto_calibrate_thermal_resistance(self):
        """
        Ingests raw empirical data, calculates thermodynamic waste heat, 
        extracts Rth, and returns the polynomial curve fits.
        """
        # ==========================================
        # PASTE RAW MOTOR TEST DATA HERE
        # ==========================================
        raw_motor_pwm = np.array([0.25, 0.50, 0.75, 1.00])
        raw_motor_amps = np.array([6.0, 12.0, 18.0, 24.0])        # Measured Current
        raw_motor_temp_c = np.array([45.0, 55.0, 62.0, 68.0])     # Steady State T_motor
        raw_motor_amb_c = np.array([25.0, 25.0, 25.0, 25.0])      # Room Temp
        
        # Calculate Rth for Motor
        # Q_waste = P_elec - P_shaft = (V * I) - (V * I * eta_controller * eta_motor)
        p_elec_motor = 24.0 * raw_motor_amps
        q_waste_motor = p_elec_motor - (p_elec_motor * 0.90 * 0.85)
        
        calc_Rth_motor = (raw_motor_temp_c - raw_motor_amb_c) / q_waste_motor
        motor_fit = np.polyfit(raw_motor_pwm, calc_Rth_motor, 2)

        # ==========================================
        # PASTE RAW HEAD TEST DATA HERE (Assuming Pump is at 100% PWM)
        # ==========================================
        raw_head_blower_cfm = np.array([0.0, 15.0, 30.0, 50.0])
        raw_head_amps = np.array([24.0, 24.0, 24.0, 24.0])        # Measured Current
        raw_head_press_bar_abs = np.array([4.0, 4.0, 4.0, 4.0])   # Absolute System Pressure (e.g. 3 Bar Gauge + 1 Bar Atm)
        raw_head_temp_c = np.array([85.0, 72.0, 63.0, 55.0])      # Steady State T_head
        raw_head_amb_c = np.array([25.0, 25.0, 25.0, 25.0])       # Room Temp
        
        # Calculate Rth for Head
        calc_Rth_head = np.zeros(len(raw_head_blower_cfm))
        for i in range(len(raw_head_blower_cfm)):
            p_elec = 24.0 * raw_head_amps[i]
            p_shaft = p_elec * 0.90 * 0.85
            
            # Calculate Pneumatic Work at this test point
            dp_bar = max(0.0, raw_head_press_bar_abs[i] - 1.0) # Assumes testing at 1 Bar absolute intake
            flow_nlpm = max(0.0, np.polyval(self.flow_curve, dp_bar))
            m_dot = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            
            pr = raw_head_press_bar_abs[i] / 1.0
            t_in_k = raw_head_amb_c[i] + 273.15
            p_pneumatic = m_dot * self.cp_air * t_in_k * ((pr ** ((self.gamma - 1.0) / self.gamma)) - 1.0)
            
            q_waste_head = max(0.0, p_shaft - p_pneumatic)
            calc_Rth_head[i] = (raw_head_temp_c[i] - raw_head_amb_c[i]) / q_waste_head
            
        head_fit = np.polyfit(raw_head_blower_cfm, calc_Rth_head, 2)
        
        return motor_fit, head_fit

    def _calculate_thermal_derate(self, requested_pwm):
        if self.T_motor_k <= self.motor_throttle_start_k:
            return requested_pwm
        if self.T_motor_k >= self.motor_limit_k:
            return 0.0 
            
        temp_margin = self.motor_limit_k - self.motor_throttle_start_k
        temp_excess = self.T_motor_k - self.motor_throttle_start_k
        return requested_pwm * (1.0 - (temp_excess / temp_margin))

    def update_system_state(self, P_up_pa, T_up_k, P_down_pa, req_pump_pwm, req_blower_cfm, dt_s):
        """Advances state by dt_s. Returns flow, current, gas temp, actual PWM, and metal temps."""
        req_pump_pwm = max(0.0, min(1.0, req_pump_pwm))
        req_blower_cfm = max(0.0, req_blower_cfm)
        
        actual_pwm = self._calculate_thermal_derate(req_pump_pwm)
        
        dp_bar = max(0.0, P_down_pa - P_up_pa) / 100000.0
        
        if dp_bar > 8.0 or actual_pwm == 0.0:
            m_dot_kg_s, current_a, P_pneumatic = 0.0, 0.0, 0.0
        else:
            flow_nlpm = max(0.0, np.polyval(self.flow_curve, dp_bar)) * actual_pwm
            m_dot_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            current_a = max(0.0, np.polyval(self.current_curve, dp_bar)) * actual_pwm
            
            pr = P_down_pa / max(1.0, P_up_pa)
            if pr > 1.0:
                P_pneumatic = m_dot_kg_s * self.cp_air * T_up_k * ((pr ** ((self.gamma - 1.0) / self.gamma)) - 1.0)
            else:
                P_pneumatic = 0.0
        
        P_elec = 24.0 * current_a
        P_shaft = P_elec * 0.90 * 0.85 
        
        heat_motor_w = max(0.0, P_elec - P_shaft)         
        heat_head_w = max(0.0, P_shaft - P_pneumatic)     

        # Use the auto-calibrated curves
        Rth_motor = max(0.05, np.polyval(self.motor_Rth_curve, actual_pwm))
        Rth_head = max(0.05, np.polyval(self.head_Rth_curve, req_blower_cfm))
        
        cooling_motor_w = (self.T_motor_k - self.T_ambient_k) / Rth_motor
        cooling_head_w = (self.T_head_k - self.T_ambient_k) / Rth_head
        
        self.T_motor_k += ((heat_motor_w - cooling_motor_w) / self.Cth_motor) * dt_s
        self.T_head_k += ((heat_head_w - cooling_head_w) / self.Cth_head) * dt_s
        
        if m_dot_kg_s > 0 and P_down_pa > P_up_pa:
            pr = P_down_pa / P_up_pa
            T_out_gas_k = T_up_k * (pr ** ((self.gamma - 1.0) / self.gamma))
        else:
            T_out_gas_k = T_up_k
            
        return m_dot_kg_s, current_a, T_out_gas_k, actual_pwm, self.T_motor_k, self.T_head_k, P_pneumatic


# ==========================================
# Test Execution
# ==========================================
if __name__ == "__main__":
    comp = SmartCompressor_120RND_Empirical()
    
    p_in = 100000.0   # 1 Bar Abs (Intake)
    p_out = 350000.0  # 4 Bar Abs (Discharge)
    t_in = 293.15     # 20 C Room
    
    print("Empirical Compressor Simulation: 100% Load, 2.5 Bar Differential")
    print(f"{'Time(s)':>7} | {'Flow(kg/s)':>12} | {'Motor Temp(C)':>15} | {'Head Temp(C)':>14} | {'Gas Out(C)':>12} | {'Pneumatic Power(W)':>18}")
    print("-" * 75)
    
    # Run a 5-minute (300 second) transient warmup simulation
    dt = 1.0
    for step in range(3001):
        # Requesting 100% pump PWM, 25 CFM from Sunon blower
        m_dot, amps, t_gas, act_pwm, t_mot, t_head, P_pneumatic = comp.update_system_state(
            P_up_pa=p_in, T_up_k=t_in, P_down_pa=p_out, req_pump_pwm=1.0, req_blower_cfm=25.0, dt_s=dt
        )
        
        if step % 40 == 0:
            print(f"{step:7.1f} | {m_dot:12.5f} | {t_mot-273.15:15.1f} | {t_head-273.15:14.1f} | {t_gas-273.15:12.1f} | {P_pneumatic:18.1f}")
            
            # Print a warning if the safety system is pulling power
            if act_pwm < 1.0:
                print(f"  -> THERMAL THROTTLE ACTIVE: Pump power reduced to {act_pwm*100:.1f}%")