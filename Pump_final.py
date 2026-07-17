import numpy as np
import Intake_hose as ih
import Hepa as hf
import Cabinet_filter as cf
import Silencer as sl


class SmartCompressor:
    """
    Component Model: G&M Tech 120RND-ED Double Head Pump.
    
    THERMODYNAMIC APPROACH:
    This model uses the First Law of Thermodynamics (Enthalpy Balance) and the 
    I^2R (Copper Loss) electrical method. It completely bypasses the need to guess 
    mechanical motor efficiency, ensuring highly accurate heat generation predictions.
    """

    def __init__(self, pump_model="140RND"):

        self.pump_model = pump_model
        
        if self.pump_model == "120RND":
            self.component_name = "120RND-ED Empirical Compressor"
            
            # --- RAW EMPIRICAL DATA (From 120RND.csv at 100% PWM) ---
            self.raw_pressure_bar = np.array([0.14, 0.5, 1.0, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.25, 6.25])
            self.raw_flow_nlpm = np.array([100.35, 95.8, 90.0, 82.5, 79.0, 76.0, 72.5, 69.5, 67.0, 64.0, 59.5, 56.0, 54.0, 49.5, 46.0])
            self.raw_current_amps = np.array([10.5, 13.2, 16.2, 18.3, 19.0, 19.8, 20.3, 20.7, 21.2, 21.6, 22.2, 22.5, 22.8, 23.0, 22.8])
            
            # Thermal Mass Estimations
            self.Cth_motor = 1513.4  
            self.Cth_head = 1413.0   
            
        elif self.pump_model == "140RND":
            self.component_name = "140RND-ED Empirical Compressor"
            
            # --- RAW EMPIRICAL DATA (From 140 RND .csv at 100% PWM) ---
            self.raw_pressure_bar = np.array([0.18, 0.5, 1.0, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.25, 6.25])
            self.raw_flow_nlpm = np.array([112.0, 104.0, 94.0, 85.5, 82.3, 79.3, 76.5, 73.5, 71.0, 68.4, 64.5, 60.5, 57.0, 52.0, 48.0])
            self.raw_current_amps = np.array([15.0, 16.8, 18.9, 20.6, 21.4, 22.0, 22.6, 23.1, 23.6, 24.1, 24.7, 25.1, 25.4, 25.6, 25.4])
            
            # Thermal Mass Estimations
            self.Cth_motor = 1650.0  
            self.Cth_head = 1550.0
            
        else:
            raise ValueError("Invalid pump model selected. Choose '120RND' or '140RND'.")
        
        # =====================================================================
        # PHYSICAL SYSTEM CONSTANTS
        # =====================================================================
        self.gamma = 1.4              # Specific heat ratio of air
        self.rho_normal = 1.204       # Density of air at 20°C and 1 ATM (kg/m^3)
        self.cp_air = 1005.0          # Specific heat of air at constant pressure (J/kg·K)
        
        # Safety Throttling Limits (Kelvin)
        self.motor_limit_k = 273.15 + 85.0          # Absolute max safe temp (85°C)
        self.motor_throttle_start_k = 273.15 + 75.0 # Start reducing power here (75°C)

        # Dynamic State Variables (Initialize at Room Temperature)
        self.temp_ambient_k = 293.15
        self.temp_motor_k = 293.15
        self.temp_head_k = 293.15
        self.voltage_v = 24.0

        self.flow_curve = np.polyfit(self.raw_pressure_bar, self.raw_flow_nlpm, 2)
        self.current_curve = np.polyfit(self.raw_pressure_bar, self.raw_current_amps, 3)

        # Run the auto-calibration based on the physical test data below
        self._run_empirical_calibration()

    def _run_empirical_calibration(self):
        """
        Reads the raw test data provided by the user, calculates the dynamic 
        Thermal Resistances (Rth), and calibrates the internal cooling Effectiveness.
        """
        
        # =====================================================================
        # TEST 1: THE MULTIMETER TEST (Copper Resistance)
        # =====================================================================
        self.R_coil_hot_ohms = 0.45 
        
        # =====================================================================
        # TEST 2: THE MOTOR COOLING TEST (Free-Spin / 0 Bar)
        # Note: We calculate this, but it is ONLY accurate for 0 Bar without external fans.
        # =====================================================================
        test2_motor_pwm = np.array([0.25, 0.50, 0.75, 1.00])
        test2_amps = np.array([1.0 , 2.20, 4.0, 6.8])        
        test2_temp_motor_c = np.array([40.0, 42.0, 44.2, 48.5])     
        test2_temp_room_c = np.array([33.0, 33.0, 33.0, 33.0])
        
        # Math: I^2R exactly calculates motor heat without guessing efficiency
        free_spin_heat_watts = (test2_amps ** 2) * self.R_coil_hot_ohms
        free_spin_Rth_values = (test2_temp_motor_c - test2_temp_room_c) / free_spin_heat_watts

        # =====================================================================
        # TEST 3: THE EXTERNAL BLOWER COOLING TEST (Full Load / TWO BLOWERS)
        # Anchor Point 3 uses actual test data: 2.71 Bar, 17.39 A, 100% Blower (x2)
        # =====================================================================
        # Scaled for 2 blowers
        test3_blower_cfm    = np.array([76.0, 98.0, 107.2]) 
        
        # Updated with 17.39 Amps at max load
        test3_amps          = np.array([15.33, 15.33, 17.39])     
        
        # Updated with 2.71 Bar back-pressure
        test3_pressure_bar  = np.array([2.5,   2.5,   2.71])
        
        # Updated with average of Rear (66.6) and Front (61.4) Head Temps
        test3_temp_head_c   = np.array([63.5,  65.0,  64.0])      
        
        # Updated with actual stable Motor Temp
        test3_temp_motor_c  = np.array([57.0,  58.5,  62.2]) 
        
        # Updated with actual Room Temp
        test3_temp_room_c   = np.array([27.5,  27.5,  26.8])
        
        # Gas Out temp based on "T_coil inlet" data
        test3_temp_gas_out_c= np.array([54.0,  54.0,  39.9]) 

        head_Rth_values = np.zeros(len(test3_blower_cfm))
        motor_Rth_load_values = np.zeros(len(test3_blower_cfm))
        epsilon_values = np.zeros(len(test3_blower_cfm))
        
        for i in range(len(test3_blower_cfm)):
            # 1. Flow Kinematics
            delta_p_bar = max(0.0, test3_pressure_bar[i] - 1.0) 
            flow_nlpm = max(0.0, np.polyval(self.flow_curve, delta_p_bar))
            mass_flow_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            
            # 2. Convert temperatures to Kelvin
            t_in_k = test3_temp_room_c[i] + 273.15
            t_head_k = test3_temp_head_c[i] + 273.15
            t_gas_out_k = test3_temp_gas_out_c[i] + 273.15
            
            # 3. Calculate Isentropic Peak Temperature
            p_down_abs = (test3_pressure_bar[i] + 1.01325) * 1e5  
            p_up_abs   = 1.01325e5                                  
            pressure_ratio = p_down_abs / p_up_abs                  
            t_peak_k = t_in_k * (pressure_ratio ** ((self.gamma - 1.0) / self.gamma))
            
            # 4. Calibrate Effectiveness (Epsilon)
            if (t_peak_k - t_head_k) > 0:
                epsilon_values[i] = (t_peak_k - t_gas_out_k) / (t_peak_k - t_head_k)
            else:
                epsilon_values[i] = 0.0
                
            # 5. FIRST LAW ENTHALPY BALANCE
            total_elec_power_w = self.voltage_v * test3_amps[i]
            motor_heat_w = (test3_amps[i] ** 2) * self.R_coil_hot_ohms
            gas_enthalpy_w = mass_flow_kg_s * self.cp_air * (t_gas_out_k - t_in_k)
            
            # Head Friction Heat
            head_friction_heat_w = max(0.0, total_elec_power_w - motor_heat_w - gas_enthalpy_w)
            
            # 6. Calculate Dynamic Thermal Resistances (Rth) based on external blower
            head_Rth_values[i] = (test3_temp_head_c[i] - test3_temp_room_c[i]) / max(0.1, head_friction_heat_w)
            
            # Calculate the motor's actual resistance when the blower is blowing over it
            motor_Rth_load_values[i] = (test3_temp_motor_c[i] - test3_temp_room_c[i]) / max(0.1, motor_heat_w)
            
        # Store the calculated curves using the external blower CFM
        self.head_Rth_curve = np.polyfit(test3_blower_cfm, head_Rth_values, 2)
        
        # Override the free-spin motor curve with the real loaded blower curve
        self.motor_Rth_curve = np.polyfit(test3_blower_cfm, motor_Rth_load_values, 2)
        
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


    def update_system_state(self, voltage_v, temp_up_k, p_down_pa, req_pump_pwm, req_blower_cfm, dt_s):
        """
        Advances the simulation by 1 timestep (dt_s). 
        Calculates fluid dynamics, tracks heat generation, and updates metal temperatures
        """

        Hepa_filter = hf.ZF111_HEPA_Filter()
        Intake_hose = ih.IntakeHose_HiPoFlex()
        Silencer = sl.AcousticSilencerChamber(chamber_vol_liters=0.564)

        self.voltage_v = voltage_v
        # Step 1: Input Validation & Safety Throttling
        req_pump_pwm = max(0.0, min(1.0, req_pump_pwm))
        req_blower_cfm = max(0.0, req_blower_cfm)
        actual_pwm = self._apply_thermal_safety_throttle(req_pump_pwm)
        
        delta_p_bar = (p_down_pa-1e5) / 100000.0
        
        # Step 2: Pneumatic & Electrical Flow
        if delta_p_bar > 8.0 or actual_pwm == 0.0:
            mass_flow_kg_s = 0.0
            flow_nlpm = 0.0
            current_amps = 0.0
            total_electrical_power_w = 0.0
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

            

            p_up_pa = Intake_hose.calculate_hose_state(
                m_dot_kg_s=mass_flow_kg_s,
                P_in_pa=101325.0,
                T_in_k=temp_up_k,
                T_amb_k=self.temp_ambient_k
            )

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
            total_electrical_power_w = self.voltage_v * current_amps
            gas_enthalpy_w = mass_flow_kg_s * self.cp_air * (temp_out_gas_k - temp_up_k)
            heat_head_w = max(0.0, total_electrical_power_w - heat_motor_w - gas_enthalpy_w)

        # Step 4: Transient Temperature Update (Heating up the mass of the metals)
        
        # Clamp the requested CFM to the bounds of our test data to prevent mathematical extrapolation errors
        safe_cfm = np.clip(req_blower_cfm, 76.0, 107.2)
        
        # Fetch the dynamic cooling resistance based on clamped blower speeds
        Rth_motor = max(0.05, np.polyval(self.motor_Rth_curve, safe_cfm))
        Rth_head = max(0.05, np.polyval(self.head_Rth_curve, safe_cfm))
        
        # Calculate how much heat the fans are successfully blowing away into the room
        cooling_motor_w = (self.temp_motor_k - self.temp_ambient_k) / Rth_motor
        cooling_head_w = (self.temp_head_k - self.temp_ambient_k) / Rth_head
        
        # Update temperatures: dT = (Heat_in - Heat_out) / Thermal_Mass * time_step
        self.temp_motor_k += ((heat_motor_w - cooling_motor_w) / self.Cth_motor) * dt_s
        self.temp_head_k += ((heat_head_w - cooling_head_w) / self.Cth_head) * dt_s
            
        return flow_nlpm, current_amps, temp_out_gas_k, actual_pwm, self.temp_motor_k, self.temp_head_k, total_electrical_power_w

if __name__ == "__main__":
    comp = SmartCompressor()
    
    silencer = sl.Silencer()
    intake_hose = ih.IntakeHose()
    

    print(f"--- Calibration Complete ---")
    print(f"Calculated Internal Heat-Exchanger Epsilon: {comp.epsilon_head:.3f}")
    print("-" * 75)
    pressure_intake_pa = 100000.0   
    pressure_discharge_pa = 400000.0  
    temp_room_k = 293.15    
    flow_nlpm = np.polyval(comp.flow_curve, 4)
    print(f"Test Flow at 4 Bar: {flow_nlpm:.1f} NLPM")