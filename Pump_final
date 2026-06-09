import numpy as np

class SmartCompressor_120RND:
    """
    Component Model: G&M Tech 120RND-ED Double Head Pump.
    
    THERMODYNAMIC APPROACH:
    This model uses the First Law of Thermodynamics (Enthalpy Balance) and the 
    I^2R (Copper Loss) electrical method. It completely bypasses the need to guess 
    mechanical motor efficiency, ensuring highly accurate heat generation predictions.
    """
    
    def __init__(self):
        self.component_name = "120RND-ED Empirical Compressor"
        
        # =====================================================================
        # PHYSICAL SYSTEM CONSTANTS
        # =====================================================================
        self.gamma = 1.4              # Specific heat ratio of air
        self.rho_normal = 1.204       # Density of air at 20°C and 1 ATM (kg/m^3)
        self.cp_air = 1005.0          # Specific heat of air at constant pressure (J/kg·K)
        
        # Thermal Capacitance (Mass * Specific Heat) in Joules/Kelvin
        self.Cth_motor = 1513.4       # Blended mass of Steel casing + Copper Stator
        self.Cth_head = 1413.0        # Mass of AL6061 and ADC12 Aluminum Heads
        
        # Safety Throttling Limits (Kelvin)
        self.motor_limit_k = 273.15 + 75.0          # Absolute max safe temp (75°C)
        self.motor_throttle_start_k = 273.15 + 65.0 # Start reducing power here (65°C)

        # Dynamic State Variables (Initialize at Room Temperature)
        self.temp_ambient_k = 293.15
        self.temp_motor_k = 293.15
        self.temp_head_k = 293.15

        # =====================================================================
        # PNEUMATIC MANUFACTURER CURVES (Flow & Current vs. Pressure)
        # =====================================================================
        pressure_points_bar = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        flow_points_nlpm = np.array([133, 121, 108, 90, 79, 67, 60, 52])
        current_points_amps = np.array([9.9, 16.1, 20.8, 22.9, 24.0, 24.5, 24.9, 24.5])
        
        self.flow_curve = np.polyfit(pressure_points_bar, flow_points_nlpm, 2)
        self.current_curve = np.polyfit(pressure_points_bar, current_points_amps, 3)

        # Run the auto-calibration based on the physical test data below
        self._run_empirical_calibration()


    def _run_empirical_calibration(self):
        """
        Reads the raw test data provided by the user, calculates the dynamic 
        Thermal Resistances (Rth), and calibrates the internal cooling Effectiveness.
        """
        
        # =====================================================================
        # TEST 1: THE MULTIMETER TEST (Copper Resistance)
        # Action: Turn pump OFF. Measure resistance across power wires with a multimeter.
        # Note: Run pump until hot, turn off, and immediately measure to get "Hot" resistance.
        # =====================================================================
        self.R_coil_hot_ohms = 0.45 
        

        # =====================================================================
        # TEST 2: THE MOTOR COOLING TEST (Free-Spin / 0 Bar)
        # Action: Remove hoses (0 Bar). Run pump at different PWM speeds. 
        # Wait 15 mins for temps to steady. Record Amps, Room Temp, and Motor Temp.
        # =====================================================================
        test2_motor_pwm = np.array([0.25, 0.50, 0.75, 1.00])
        test2_amps = np.array([6.0, 12.0, 18.0, 24.0])        
        test2_temp_motor_c = np.array([45.0, 55.0, 62.0, 68.0])     
        test2_temp_room_c = np.array([25.0, 25.0, 25.0, 25.0])      
        
        # Math: I^2R exactly calculates motor heat without guessing efficiency
        motor_heat_watts = (test2_amps ** 2) * self.R_coil_hot_ohms
        motor_Rth_values = (test2_temp_motor_c - test2_temp_room_c) / motor_heat_watts
        
        # Create a curve so the software knows Rth for ANY requested PWM
        self.motor_Rth_curve = np.polyfit(test2_motor_pwm, motor_Rth_values, 2)


        # =====================================================================
        # TEST 3: THE HEAD COOLING TEST (Full Load / 4 Bar)
        # Action: Attach hoses. Set back-pressure to 4 Bar. Run external blower at 
        # different speeds (CFM). Wait 15 mins. Record Head Temp and the ACTUAL 
        # temperature of the air shooting out of the discharge hose.
        # =====================================================================
        test3_blower_cfm = np.array([0.0, 15.0, 30.0, 50.0])
        test3_amps = np.array([24.0, 24.0, 24.0, 24.0])        
        test3_pressure_bar = np.array([4.0, 4.0, 4.0, 4.0])   
        test3_temp_head_c = np.array([85.0, 72.0, 63.0, 55.0])      
        test3_temp_room_c = np.array([25.0, 25.0, 25.0, 25.0])       
        test3_temp_gas_out_c = np.array([125.0, 110.0, 95.0, 85.0]) 

        head_Rth_values = np.zeros(len(test3_blower_cfm))
        epsilon_values = np.zeros(len(test3_blower_cfm))
        
        for i in range(len(test3_blower_cfm)):
            # 1. Flow Kinematics
            delta_p_bar = max(0.0, test3_pressure_bar[i] - 1.0) 
            flow_nlpm = max(0.0, np.polyval(self.flow_curve, delta_p_bar))
            mass_flow_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            
            # 2. Convert temperatures to Kelvin for thermodynamic math
            t_in_k = test3_temp_room_c[i] + 273.15
            t_head_k = test3_temp_head_c[i] + 273.15
            t_gas_out_k = test3_temp_gas_out_c[i] + 273.15
            
            # 3. Calculate Isentropic Peak Temperature (The theoretical maximum)
            pressure_ratio = test3_pressure_bar[i] / 1.0
            t_peak_k = t_in_k * (pressure_ratio ** ((self.gamma - 1.0) / self.gamma))
            
            # 4. Calibrate Effectiveness (Epsilon)
            # Epsilon = How closely the gas cooled down to match the metal head temp
            if (t_peak_k - t_head_k) > 0:
                epsilon_values[i] = (t_peak_k - t_gas_out_k) / (t_peak_k - t_head_k)
            else:
                epsilon_values[i] = 0.0
                
            # 5. FIRST LAW ENTHALPY BALANCE (Isolating exactly how much heat hit the metal)
            total_elec_power_w = 24.0 * test3_amps[i]
            motor_heat_w = (test3_amps[i] ** 2) * self.R_coil_hot_ohms
            gas_enthalpy_w = mass_flow_kg_s * self.cp_air * (t_gas_out_k - t_in_k)
            
            # Head Friction = Everything left over after motor heat and gas heat escape
            head_friction_heat_w = max(0.0, total_elec_power_w - motor_heat_w - gas_enthalpy_w)
            
            # Calculate Head Thermal Resistance
            head_Rth_values[i] = (test3_temp_head_c[i] - test3_temp_room_c[i]) / max(0.1, head_friction_heat_w)
            
        # Store the calculated curves and average epsilon
        self.head_Rth_curve = np.polyfit(test3_blower_cfm, head_Rth_values, 2)
        self.epsilon_head = np.clip(np.mean(epsilon_values), 0.0, 1.0)


    def _apply_thermal_safety_throttle(self, requested_pwm):
        """Reduces requested pump speed if the motor approaches critical failure limits."""
        if self.temp_motor_k <= self.motor_throttle_start_k:
            return requested_pwm
        if self.temp_motor_k >= self.motor_limit_k:
            return 0.0 
        
        temp_margin = self.motor_limit_k - self.motor_throttle_start_k
        temp_excess = self.temp_motor_k - self.motor_throttle_start_k
        throttle_multiplier = 1.0 - (temp_excess / temp_margin)
        
        return requested_pwm * throttle_multiplier


    def update_system_state(self, p_up_pa, temp_up_k, p_down_pa, req_pump_pwm, req_blower_cfm, dt_s):
        """
        Advances the simulation by 1 timestep (dt_s). 
        Calculates fluid dynamics, tracks heat generation, and updates metal temperatures.
        """
        # Step 1: Input Validation & Safety Throttling
        req_pump_pwm = max(0.0, min(1.0, req_pump_pwm))
        req_blower_cfm = max(0.0, req_blower_cfm)
        actual_pwm = self._apply_thermal_safety_throttle(req_pump_pwm)
        
        delta_p_bar = max(0.0, p_down_pa - p_up_pa) / 100000.0
        
        # Step 2: Pneumatic & Electrical Flow
        if delta_p_bar > 8.0 or actual_pwm == 0.0:
            mass_flow_kg_s = 0.0
            current_amps = 0.0
            temp_out_gas_k = temp_up_k
            heat_head_w = 0.0
            heat_motor_w = 0.0
        else:
            flow_nlpm = max(0.0, np.polyval(self.flow_curve, delta_p_bar)) * actual_pwm
            mass_flow_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            current_amps = max(0.0, np.polyval(self.current_curve, delta_p_bar)) * actual_pwm
            
            # Step 3: Thermodynamics & Energy Splitting
            
            # A) Calculate pure electrical Motor Heat (I^2R)
            heat_motor_w = (current_amps ** 2) * self.R_coil_hot_ohms
            
            # B) Calculate Gas Temperature Dynamics
            pressure_ratio = p_down_pa / max(1.0, p_up_pa)
            if pressure_ratio > 1.0:
                # The theoretical maximum temperature inside the cylinder
                temp_peak_k = temp_up_k * (pressure_ratio ** ((self.gamma - 1.0) / self.gamma))
            else:
                temp_peak_k = temp_up_k
                
            # The actual gas temp drops as it touches the cooler aluminum head (Quench)
            temp_out_gas_k = temp_peak_k - self.epsilon_head * (temp_peak_k - self.temp_head_k)
            
            # C) Calculate Enthalpy and pure Head Friction Heat
            total_electrical_power_w = 24.0 * current_amps
            gas_enthalpy_w = mass_flow_kg_s * self.cp_air * (temp_out_gas_k - temp_up_k)
            heat_head_w = max(0.0, total_electrical_power_w - heat_motor_w - gas_enthalpy_w)

        # Step 4: Transient Temperature Update (Heating up the mass of the metals)
        
        # Fetch the dynamic cooling resistance based on current fan/blower speeds
        Rth_motor = max(0.05, np.polyval(self.motor_Rth_curve, actual_pwm))
        Rth_head = max(0.05, np.polyval(self.head_Rth_curve, req_blower_cfm))
        
        # Calculate how much heat the fans are successfully blowing away into the room
        cooling_motor_w = (self.temp_motor_k - self.temp_ambient_k) / Rth_motor
        cooling_head_w = (self.temp_head_k - self.temp_ambient_k) / Rth_head
        
        # Update temperatures: dT = (Heat_in - Heat_out) / Thermal_Mass * time_step
        self.temp_motor_k += ((heat_motor_w - cooling_motor_w) / self.Cth_motor) * dt_s
        self.temp_head_k += ((heat_head_w - cooling_head_w) / self.Cth_head) * dt_s
            
        return mass_flow_kg_s, current_amps, temp_out_gas_k, actual_pwm, self.temp_motor_k, self.temp_head_k, total_electrical_power_w, flow_nlpm


# =====================================================================
# SIMULATION EXECUTION (MAIN LOOP)
# =====================================================================
if __name__ == "__main__":
    # Initialize the compressor (This automatically runs the calibration)
    comp = SmartCompressor_120RND()
    
    print(f"--- Calibration Complete ---")
    print(f"Calculated Internal Heat-Exchanger Epsilon: {comp.epsilon_head:.3f}")
    print("-" * 75)
    
    # Define system pressures (1 Bar IN, 4 Bar OUT)
    pressure_intake_pa = 100000.0   
    pressure_discharge_pa = 400000.0  
    temp_room_k = 293.15     
    
    print("Running Thermodynamic Simulation: 100% Load, 3 Bar Differential")
    print(f"{'Time(s)':>7} | {'Flow(kg/s)':>12} | {'Motor Temp(C)':>15} | {'Head Temp(C)':>14} | {'Gas Out(C)':>12} | {'Electrical Power(W)':>18} | {'Flow(NLPM)':>12}")
    print("-" * 75)
    
    time_step_s = 1.0
    PWM_demd = 1
    # Run the simulation for 300 seconds (5 minutes)
    for step in range(3001):
        
        # Advance the physics engine by 1 second
        flow_kg_s, current, t_gas_k, act_pwm, t_mot_k, t_head_k, total_electrical_power_w, flow_nlpm = comp.update_system_state(
            p_up_pa = pressure_intake_pa, 
            temp_up_k = temp_room_k, 
            p_down_pa = pressure_discharge_pa, 
            req_pump_pwm = PWM_demd,        # Ask for 100% speed
            req_blower_cfm = 25.0,     # External Sunon blower running at 25 CFM
            dt_s = time_step_s
        )
        
        # Print the data to the console every 30 seconds
        if step % 30 == 0:
            print(f"{step:7.1f} | {flow_kg_s:12.5f} | {t_mot_k-273.15:15.1f} | {t_head_k-273.15:14.1f} | {t_gas_k-273.15:12.1f} | {total_electrical_power_w:18.1f} | {flow_nlpm:12.1f}")
            
            # Warn the user if the software had to step in and save the motor from melting
            if act_pwm < PWM_demd:
                print(f"  -> WARNING: Thermal Limit Reached! Motor power reduced to {act_pwm*100:.1f}%")