import math
import numpy as np
import matplotlib.pyplot as plt
import Pump_final as Pmp
import Digital_tank as dt
import cooling_coil2 as cc
import Discharge_hose as dh
import NRV as nrv
import air_filter as af
import water_separater as ws
import mist_separator as ms
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import Silencer as sl
import Hepa as hp
import Cabinet_filter as cf
import Intake_hose as ih




# ==========================================
# 1. THE PID CONTROLLER
# ==========================================
class ThermalPIDController:
    def __init__(self, kp, ki, kd, target_temp_c):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.target_temp = target_temp_c
        self.integral_sum = 0.0
        self.previous_error = 0.0
        self.pwm_min = 0.0
        self.pwm_max = 100.0

    def update(self, current_temp_c, dt_seconds):
        error = current_temp_c - self.target_temp
        P_out = self.Kp * error
        
        self.integral_sum += error * dt_seconds
        I_out = self.Ki * self.integral_sum
        
        derivative = (error - self.previous_error) / dt_seconds
        D_out = self.Kd * derivative
        self.previous_error = error
        
        raw_pwm = P_out + I_out + D_out
        final_pwm = max(self.pwm_min, min(self.pwm_max, raw_pwm))
        
        # Anti-windup
        if raw_pwm > self.pwm_max or raw_pwm < self.pwm_min:
            self.integral_sum -= error * dt_seconds 
            
        return final_pwm

# ==========================================
# 2. THE SUNON FAN TRANSLATOR
# ==========================================
blower_pwm_data = np.array([0, 25, 40, 50, 60, 67, 77, 85, 100])
blower_rpm_data = np.array([1250, 2275, 2885, 3355, 3960, 4850, 5750, 6250, 6820])
blower_rpm_fit  = np.polyfit(blower_pwm_data, blower_rpm_data, 2)

def get_cfm_from_pwm(pwm_percentage):
    rpm = np.polyval(blower_rpm_fit, pwm_percentage)
    return (rpm / 6820.0) * 53.6*2


# class SmartCompressor_120RND:
#     """
#     Component Model: G&M Tech 120RND-ED Double Head Pump.
    
#     THERMODYNAMIC APPROACH:
#     This model uses the First Law of Thermodynamics (Enthalpy Balance) and the 
#     I^2R (Copper Loss) electrical method. It completely bypasses the need to guess 
#     mechanical motor efficiency, ensuring highly accurate heat generation predictions.
#     """
    
#     def __init__(self):
#         self.component_name = "120RND-ED Empirical Compressor"
        
#         # =====================================================================
#         # PHYSICAL SYSTEM CONSTANTS
#         # =====================================================================
#         self.gamma = 1.4              # Specific heat ratio of air
#         self.rho_normal = 1.204       # Density of air at 20°C and 1 ATM (kg/m^3)
#         self.cp_air = 1005.0          # Specific heat of air at constant pressure (J/kg·K)
        
#         # Thermal Capacitance (Mass * Specific Heat) in Joules/Kelvin
#         self.Cth_motor = 1513.4       # Blended mass of Steel casing + Copper Stator
#         self.Cth_head = 1413.0        # Mass of AL6061 and ADC12 Aluminum Heads
        
#         # Safety Throttling Limits (Kelvin)
#         self.motor_limit_k = 273.15 + 75.0          # Absolute max safe temp (75°C)
#         self.motor_throttle_start_k = 273.15 + 65.0 # Start reducing power here (65°C)

#         # Dynamic State Variables (Initialize at Room Temperature)
#         self.temp_air = 40 + 273.15
#         self.temp_motor_k = 293.15
#         self.temp_head_k = 40 + 273.15  # Heads start cooler due to direct contact with incoming air

#         # =====================================================================
#         # PNEUMATIC MANUFACTURER CURVES (Flow & Current vs. Pressure)
#         # =====================================================================
#         pressure_points_bar = np.array([0, 1, 2, 3, 4, 5, 6, 7])
#         flow_points_nlpm = np.array([133, 121, 108, 90, 79, 67, 60, 52])
#         current_points_amps = np.array([9.9, 16.1, 20.8, 22.9, 24.0, 24.5, 24.9, 24.5])
        
#         self.flow_curve = np.polyfit(pressure_points_bar, flow_points_nlpm, 2)
#         self.current_curve = np.polyfit(pressure_points_bar, current_points_amps, 3)

#         # Run the auto-calibration based on the physical test data below
#         self._run_empirical_calibration()


#     def _run_empirical_calibration(self):
#         """
#         Reads the raw test data provided by the user, calculates the dynamic 
#         Thermal Resistances (Rth), and calibrates the internal cooling Effectiveness.
#         """
        
#         # =====================================================================
#         # TEST 1: THE MULTIMETER TEST (Copper Resistance)
#         # Action: Turn pump OFF. Measure resistance across power wires with a multimeter.
#         # Note: Run pump until hot, turn off, and immediately measure to get "Hot" resistance.
#         # =====================================================================
#         self.R_coil_hot_ohms = 0.45 
        

#         # =====================================================================
#         # TEST 2: THE MOTOR COOLING TEST (Free-Spin / 0 Bar)
#         # Action: Remove hoses (0 Bar). Run pump at different PWM speeds. 
#         # Wait 15 mins for temps to steady. Record Amps, Room Temp, and Motor Temp.
#         # =====================================================================
#         test2_motor_pwm = np.array([0.25, 0.50, 0.75, 1.00])
#         test2_amps = np.array([1, 2.25, 4.05 , 6.8])        
#         test2_temp_motor_c = np.array([40.0, 42.0, 44.2, 48.5])     
#         test2_temp_room_c = np.array([33.0, 33.0, 33.0, 33.0]) 
#         test2_Voltage = np.array([26.1, 26.1, 26.1, 26.05]) 
        
#         # Math: I^2R exactly calculates motor heat without guessing efficiency
#         motor_heat_watts = (test2_amps ** 2) * self.R_coil_hot_ohms
#         motor_Rth_values = (test2_temp_motor_c - test2_temp_room_c) / motor_heat_watts
        
#         # Create a curve so the software knows Rth for ANY requested PWM
#         self.motor_Rth_curve = np.polyfit(test2_motor_pwm, motor_Rth_values, 2)


#         # =====================================================================
#         # TEST 3: THE HEAD COOLING TEST (Full Load / 4 Bar)
#         # Action: Attach hoses. Set back-pressure to 4 Bar. Run external blower at 
#         # different speeds (CFM). Wait 15 mins. Record Head Temp and the ACTUAL 
#         # temperature of the air shooting out of the discharge hose.
#         # =====================================================================
#         test3_blower_cfm = np.array([0.0, 18.9, 53.6, 80.0, 107.2])
#         test3_amps = np.array([24.0, 24.0, 24.0, 24.0, 24.0])        
#         test3_pressure_bar = np.array([4.0, 4.0, 4.0, 4.0, 4.0])   
#         test3_temp_head_c = np.array([88.0, 72.0, 54.0, 49.0, 45.0])      
#         test3_temp_room_c = np.array([40.0, 40.0, 40.0, 40.0, 40.0])       
#         test3_temp_gas_out_c = np.array([130.0, 112.0, 88.0, 80.0, 75.0])  

#         head_Rth_values = np.zeros(len(test3_blower_cfm))
#         epsilon_values = np.zeros(len(test3_blower_cfm))
        
#         for i in range(len(test3_blower_cfm)):
#             # 1. Flow Kinematics
#             delta_p_bar = max(0.0, test3_pressure_bar[i] - 1.0) 
#             flow_nlpm = max(0.0, np.polyval(self.flow_curve, delta_p_bar))
#             mass_flow_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            
#             # 2. Convert temperatures to Kelvin for thermodynamic math
#             t_in_k = test3_temp_room_c[i] + 273.15
#             t_head_k = test3_temp_head_c[i] + 273.15
#             t_gas_out_k = test3_temp_gas_out_c[i] + 273.15
            
#             # 3. Calculate Isentropic Peak Temperature (The theoretical maximum)
#             pressure_ratio = test3_pressure_bar[i] / 1.0
#             t_peak_k = t_in_k * (pressure_ratio ** ((self.gamma - 1.0) / self.gamma))
            
#             # 4. Calibrate Effectiveness (Epsilon)
#             # Epsilon = How closely the gas cooled down to match the metal head temp
#             if (t_peak_k - t_head_k) > 0:
#                 epsilon_values[i] = (t_peak_k - t_gas_out_k) / (t_peak_k - t_head_k)
#             else:
#                 epsilon_values[i] = 0.0
                
#             # 5. FIRST LAW ENTHALPY BALANCE (Isolating exactly how much heat hit the metal)
#             total_elec_power_w = 24.0 * test3_amps[i]
#             motor_heat_w = (test3_amps[i] ** 2) * self.R_coil_hot_ohms
#             gas_enthalpy_w = mass_flow_kg_s * self.cp_air * (t_gas_out_k - t_in_k)
            
#             # Head Friction = Everything left over after motor heat and gas heat escape
#             head_friction_heat_w = max(0.0, total_elec_power_w - motor_heat_w - gas_enthalpy_w)
            
#             # Calculate Head Thermal Resistance
#             head_Rth_values[i] = (test3_temp_head_c[i] - test3_temp_room_c[i]) / max(0.1, head_friction_heat_w)
            
#         # Store the calculated curves and average epsilon
#         self.head_Rth_curve = np.polyfit(test3_blower_cfm, head_Rth_values, 2)
#         self.epsilon_head = np.clip(np.mean(epsilon_values), 0.0, 1.0)


#     def _apply_thermal_safety_throttle(self, requested_pwm):
#         """Reduces requested pump speed if the motor approaches critical failure limits."""
#         if self.temp_motor_k <= self.motor_throttle_start_k:
#             return requested_pwm
#         if self.temp_motor_k >= self.motor_limit_k:
#             return 0.0 
        
#         temp_margin = self.motor_limit_k - self.motor_throttle_start_k
#         temp_excess = self.temp_motor_k - self.motor_throttle_start_k
#         throttle_multiplier = 1.0 - (temp_excess / temp_margin)
        
#         return requested_pwm * throttle_multiplier


#     def update_system_state(self, p_up_pa, temp_up_k, p_down_pa, req_pump_pwm, req_blower_cfm, dt_s):
#         """
#         Advances the simulation by 1 timestep (dt_s). 
#         Calculates fluid dynamics, tracks heat generation, and updates metal temperatures.
#         """
#         # Step 1: Input Validation & Safety Throttling
#         req_pump_pwm = max(0.0, min(1.0, req_pump_pwm))
#         req_blower_cfm = max(0.0, req_blower_cfm)
#         actual_pwm = self._apply_thermal_safety_throttle(req_pump_pwm)
        
#         delta_p_bar = max(0.0, p_down_pa - p_up_pa) / 100000.0
        
#         # Step 2: Pneumatic & Electrical Flow
#         if delta_p_bar > 8.0 or actual_pwm == 0.0:
#             mass_flow_kg_s = 0.0
#             flow_nlpm = 0.0
#             current_amps = 0.0
#             total_electrical_power_w = 0.0
#             temp_out_gas_k = temp_up_k
#             heat_head_w = 0.0
#             heat_motor_w = 0.0
#         else:
#             flow_nlpm = max(0.0, np.polyval(self.flow_curve, delta_p_bar)) * actual_pwm
#             mass_flow_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
#             current_amps = max(0.0, np.polyval(self.current_curve, delta_p_bar)) * actual_pwm
            
#             # Step 3: Thermodynamics & Energy Splitting
            
#             # A) Calculate pure electrical Motor Heat (I^2R)
#             heat_motor_w = (current_amps ** 2) * self.R_coil_hot_ohms
            
#             # B) Calculate Gas Temperature Dynamics
#             pressure_ratio = p_down_pa / max(1.0, p_up_pa)
#             if pressure_ratio > 1.0:
#                 # The theoretical maximum temperature inside the cylinder
#                 temp_peak_k = temp_up_k * (pressure_ratio ** ((self.gamma - 1.0) / self.gamma))
#             else:
#                 temp_peak_k = temp_up_k
                
#             # The actual gas temp drops as it touches the cooler aluminum head (Quench)
#             temp_out_gas_k = temp_peak_k - self.epsilon_head * (temp_peak_k - self.temp_head_k)
            
#             # C) Calculate Enthalpy and pure Head Friction Heat
#             total_electrical_power_w = 24.0 * current_amps
#             gas_enthalpy_w = mass_flow_kg_s * self.cp_air * (temp_out_gas_k - temp_up_k)
#             heat_head_w = max(0.0, total_electrical_power_w - heat_motor_w - gas_enthalpy_w)

#         # Step 4: Transient Temperature Update (Heating up the mass of the metals)
        
#         # Fetch the dynamic cooling resistance based on current fan/blower speeds
#         Rth_motor = max(0.05, np.polyval(self.motor_Rth_curve, actual_pwm))
#         Rth_head = max(0.05, np.polyval(self.head_Rth_curve, req_blower_cfm))
        
#         # Calculate how much heat the fans are successfully blowing away into the room
#         cooling_motor_w = (self.temp_motor_k - self.temp_air) / Rth_motor
#         cooling_head_w = (self.temp_head_k - self.temp_air) / Rth_head
        
#         # Update temperatures: dT = (Heat_in - Heat_out) / Thermal_Mass * time_step
#         self.temp_motor_k += ((heat_motor_w - cooling_motor_w) / self.Cth_motor) * dt_s
#         self.temp_head_k += ((heat_head_w - cooling_head_w) / self.Cth_head) * dt_s
            
#         return flow_nlpm, current_amps, temp_out_gas_k, actual_pwm, self.temp_motor_k, self.temp_head_k, total_electrical_power_w


# =====================================================================
# SIMULATION EXECUTION (MAIN LOOP)
# =====================================================================
if __name__ == "__main__":
    # Initialize the components
    comp = Pmp.SmartCompressor(pump_model="140RND")
    pid = ThermalPIDController(kp=4.0, ki=0.5, kd=0.1, target_temp_c=50.0)
    hose = dh.DischargeHose_HiPoFlex()
    Cooling_coil = cc.CopperCoolingCoil_TwinFan()
    Nrv = nrv.SMC_AKH10_NRV()
    Air_filter = af.SMC_AF20_Filter()
    Water_sep = ws.SMC_AFG20_WaterSeparator()
    Mist_sep = ms.SMC_AFM20_MistSeparator()
    Intake_hose = ih.IntakeHose_HiPoFlex()

    # Air_filter = af.SMC_AF20_Filter(valve_closed= True  )  # Simulating the unvalved 1.8mm leak
    # Water_sep = ws.SMC_AFG20_WaterSeparator(valve_closed= True  )  # Simulating the unvalved 1.8mm leak
    # Mist_sep = ms.SMC_AFM20_MistSeparator(valve_closed= True  )  # Simulating the unvalved 1.8mm leak
    
    # NEW: Initialize Intake Components
    Silencer = sl.AcousticSilencerChamber(chamber_vol_liters=0.564)  # ID - 114.3 , Length - 55.0 mm, Volume - 564 mL
    Hepa = hp.ZF111_HEPA_Filter()
    # Note: Cabinet filter is just a function, so we don't need to initialize a class for it!
    
    # System Constants
    temp_room_k = 273.15 + 35.0    # 35°C Room
    
    # Sync the compressor's ambient cooling temp to the room temp
    comp.temp_motor_k = temp_room_k
    comp.temp_head_k = temp_room_k

    dt_s = 0.01

    tank = dt.SmartCompressorTankTwin(
        volume_liters=2.0,
        motor_voltage_v=24.0
    )
    total_time_s = 600
    time = np.arange(0, total_time_s, dt_s)



    # =================================================================
    hist_time = []
    hist_p_tank = []
    hist_pwm = []
    hist_power = []
    hist_temp_motor_c = []
    hist_temp_head_c = []
    hist_q_mist = []
    hist_q_comp = []
    hist_q_vent = []
    hist_blower_cfm = []

    # NEW: Cascading Pressure History
    hist_p_hepa = []
    hist_p_silencer= []
    hist_p_intake = []
    hist_p_comp = []
    hist_flow_compressor = []
    hist_p_hose = []
    hist_p_coil = []
    hist_p_af = []
    hist_p_ws = []
    hist_p_ms = []
    hist_delta_P_af = []
    hist_delta_P_wf = []
    hist_delta_P_ms = []
    hist_delta_APU = []

    # NEW: Leakage Flow History
    hist_leak_af = []
    hist_leak_ws = []
    hist_leak_ms = []

    # Initialize variables for t=0
    current_blower_cfm = 0.0 
    current_compressor_flow_nlpm = 0.0 
    q_nlpm_mist = 0.0          
    requested_pwm = 100.0      

    # FIX: seed friction with a realistic non-zero value so the chain
    # sees meaningful inlet pressure from step 1 instead of atmospheric.
    # ~0.15 bar is a reasonable first-principles estimate of line losses.
    actual_system_friction_pa = 0.0
    p_tank_abs_pa = 101325.0
    actual_intake_m_dot_kg_s = 0.0 

    for t in time:
        
        # =====================================================================
        # 1. INTAKE DYNAMICS
        # =====================================================================
        rho_room = 101325.0 / (287.05 * temp_room_k) 
        intake_flow_lpm = (actual_intake_m_dot_kg_s / rho_room) * 1000.0 * 60.0
        intake_flow_cfm = intake_flow_lpm * 0.0353147

        dp_cabinet_pa = cf.calculate_filter_pressure_drop(flow_rate_cfm=intake_flow_cfm)
        p_after_cabinet_pa = 101325.0 - dp_cabinet_pa

        Hepa.update_loading(intake_flow_lpm, dt_s)
        dp_hepa_pa = Hepa.pressure_drop(intake_flow_lpm)
        p_after_hepa_pa = p_after_cabinet_pa - dp_hepa_pa

        p_silencer_pa, _ = Silencer.update_state(
            P_upstream_pa=p_after_hepa_pa, 
            m_dot_out_kg_s=actual_intake_m_dot_kg_s, 
            dt_s=dt_s
        )

        # D. Intake Hose (Real Physics Module)
        # We use an underscore '_' for the output temperature because 
        # the air is already at room temp, so we don't need to track it.
        p_comp_intake_pa, _ = Intake_hose.calculate_hose_state(
            m_dot_kg_s = actual_intake_m_dot_kg_s,
            P_in_pa = p_silencer_pa,       
            T_in_k = temp_room_k,
            T_amb_k = temp_room_k 
        )

        # =====================================================================
        # 2. THE COMPRESSOR
        # =====================================================================
        p_comp_discharge_pa = p_tank_abs_pa + actual_system_friction_pa
        # print("Comp_discgarge_bar:", p_comp_discharge_pa/100000, "Comp_intake_bar:",  p_comp_intake_pa/1e5, "afterSilencer_bar:",  p_silencer_pa/1e5, "After_hepa_bar:", p_after_hepa_pa/1e5  )
        # p_comp_discharge_pa = p_tank_abs_pa

        (current_compressor_flow_nlpm, current_amps, temp_out_gas_k, 
         act_pwm, temp_motor_k, temp_head_k, power_w) = comp.update_system_state(
            p_up_pa = p_comp_intake_pa,          
            temp_up_k = temp_room_k, 
            p_down_pa = p_comp_discharge_pa,     
            req_pump_pwm = requested_pwm,        
            req_blower_cfm = current_blower_cfm, 
            dt_s = dt_s
        )
        # if t % 75.0 < dt_s:
        #     print(current_compressor_flow_nlpm)
        m_dot_kg_s = (current_compressor_flow_nlpm / 1000.0 / 60.0) * comp.rho_normal
        # actual_intake_m_dot_kg_s = (0.8 * actual_intake_m_dot_kg_s) + (0.2 * m_dot_kg_s) 

        # =====================================================================
        # 3. DISCHARGE HOSE & COOLING COIL
        # =====================================================================
        P_out_pa_hose, T_out_k_hose = hose.calculate_hose_state(
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = p_comp_discharge_pa,       
            T_in_k = temp_out_gas_k,
            T_amb_k = temp_room_k 
        )

        T_out_k_coil, T_ambient_out_celsius, P_out_pa_coil = Cooling_coil.analyze_coil(
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = P_out_pa_hose,  
            T_in_k = T_out_k_hose,
            T_amb_k = temp_room_k,
            selected_length = 3.36 
        )
        comp.temp_air = T_ambient_out_celsius + 273.15

        # =====================================================================
        # 4. NRV & SEPARATORS
        # =====================================================================
        P_out_pa_nrv, delta_p_mbar_nrv, T_out_k_nrv = Nrv.calculate_valve_state(
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = P_out_pa_coil,
            T_in_k = T_out_k_coil
        )

        P_out_pa_filter, delta_p_mbar_filter, dp_viscous_mbar, dp_inertial_mbar, m_dot_out_kg_s_air_filter = Air_filter.calculate_filter_state(
            t=t,
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = P_out_pa_nrv,
            T_in_k = T_out_k_nrv
        )

        P_out_pa_water, delta_p_mbar_water, m_dot_effective_water, water_escaped_mg_s, vapor_passed_mg_s = Water_sep.update_state(
            t=t,
            m_dot_in_kg_s = m_dot_out_kg_s_air_filter,
            P_in_pa = P_out_pa_filter,
            T_in_k = T_out_k_nrv,
            T_amb_C = temp_room_k - 273.15,
            RH_amb = 0.5
        )


        P_out_pa_mist, delta_p_mbar_mist, q_nlpm_mist, mist_escaped_mg_s = Mist_sep.update_state(
            t=t,
            m_dot_in_kg_s = m_dot_effective_water,
            P_in_pa = P_out_pa_water,
            T_in_k = T_out_k_nrv,
            aerosol_water_in_mg_s= vapor_passed_mg_s,
            dt_seconds = dt_s,
        )

        # =====================================================================
        # 5. THE TANK 
        # =====================================================================
        p_tank_gauge, requested_pwm, q_pump_in, q_vent_out = tank.simulate(
            t = t,
            T_in_K= T_out_k_nrv,
            dt_s=dt_s,
            t_insp=0.5,
            flow_insp=48,
            t_exp=0.5,
            flow_exp=48,
            external_inflow_nlpm= q_nlpm_mist,  
        )
        p_tank_abs_pa = (p_tank_gauge * 100000.0) + 101325.0
        # print(q_pump_in)


        # =====================================================================
        # 6. SYSTEM CONTROLLERS
        # =====================================================================
        fan_pwm = pid.update(temp_head_k - 273.15, dt_s)
        fan_cfm = get_cfm_from_pwm(fan_pwm)
        current_blower_cfm = max(2 * fan_cfm, 0)  

        new_friction_pa = max(0.0, p_comp_discharge_pa - P_out_pa_mist)
        actual_system_friction_pa = (0* actual_system_friction_pa) + (1 * new_friction_pa)

        # =====================================================================
        # 7. METRICS & APPEND (Downsampled to 2 seconds)
        # =====================================================================
        if t % 2.0 < dt_s:
            
            # --- CALCULATE CASCADING PRESSURES (Convert Pa Abs to Bar Gauge) ---
            hist_p_comp.append((p_comp_discharge_pa - 101325.0) / 100000.0)
            hist_flow_compressor.append(current_compressor_flow_nlpm)
            hist_p_hose.append((P_out_pa_hose - 101325.0) / 100000.0)
            hist_p_coil.append((P_out_pa_coil - 101325.0) / 100000.0)
            hist_p_af.append((P_out_pa_filter - 101325.0) / 100000.0)
            hist_p_ws.append((P_out_pa_water - 101325.0) / 100000.0)
            hist_p_ms.append((P_out_pa_mist - 101325.0) / 100000.0)
            hist_delta_P_af.append(delta_p_mbar_filter/1000)
            hist_delta_P_wf.append(delta_p_mbar_water/100000)
            hist_delta_P_ms.append(delta_p_mbar_mist/100000)
            hist_p_tank.append(p_tank_gauge)
            hist_delta_APU.append((P_out_pa_coil - P_out_pa_mist) / 100000.0)

            
            # --- CALCULATE LEAKAGE FLOWS (NLPM) ---
            # Conversion factor: kg/s -> NLPM based on standard air density
            to_nlpm = (1.0 / comp.rho_normal) * 60000.0
            
            # Leakage = Mass entering the component MINUS Mass leaving the component
            af_leak = max(0.0, (m_dot_kg_s - m_dot_out_kg_s_air_filter) * to_nlpm)
            ws_leak = max(0.0, (m_dot_out_kg_s_air_filter - m_dot_effective_water) * to_nlpm)
            ms_leak = max(0.0, ((m_dot_effective_water)*to_nlpm - q_nlpm_mist))
            
            hist_leak_af.append(af_leak)
            hist_leak_ws.append(ws_leak)
            hist_leak_ms.append(ms_leak)
            

            # --- STANDARD METRICS ---
            hist_time.append(t)
            hist_pwm.append(act_pwm * 100.0) 
            hist_power.append(power_w) 
            hist_temp_motor_c.append(temp_motor_k - 273.15) 
            hist_temp_head_c.append(temp_head_k - 273.15)   
            hist_q_comp.append(current_compressor_flow_nlpm)
            hist_q_mist.append(q_nlpm_mist)
            hist_q_vent.append(q_vent_out)
            hist_blower_cfm.append(current_blower_cfm)

        # if t % 75.0 < dt_s:
        #     print(q_pump_in)

# =====================================================================
    # DASHBOARD GENERATION (Now with Units!)
    # =====================================================================
    print("Generating Interactive Dashboard...")

    fig = make_subplots(
        rows=6, cols=1, 
        shared_xaxes=True, 
        vertical_spacing=0.04,
        subplot_titles=(
            "1. System Pressure Cascade (Bar Gauge)", 
            "2. Net Pneumatic Flows (NLPM)", 
            "3. Drain Restrictor Leakages (NLPM)", 
            "4. Thermal States (°C)", 
            "5. Electrical Power (Watts)", 
            "6. Compressor Motor Control (%)"
        )
    )

    # --- ROW 1: Pressure Cascade ---
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_hepa, name="0. After HEPA", line=dict(color='darkorange', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_silencer, name="0.5 After Silencer", line=dict(color='gold', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_comp, name="1. After Compressor", line=dict(color='darkred', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_hose, name="2. After Hose", line=dict(color='orangered', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_coil, name="3. After Coil", line=dict(color='orange', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_af, name="4. After Air Filter", line=dict(color='green', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_P_af, name="5. Delta_P Air Filter", line=dict(color='skyblue', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_P_wf, name="6. Delta_P water Filter ", line=dict(color='green', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_P_ms, name="7. Delta_P Mist Filter", line=dict(color='green', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_ws, name="8. After water separator", line=dict(color='purple', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_ms, name="9. After Mist Sep", line=dict(color='black', width=3), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_tank, name="10. Final Tank Pressure", line=dict(color='blue', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_APU, name="Delta P Across APU", line=dict(color='teal', dash='dash', width=2), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)

    # --- ROW 2: Net Flows ---
    fig.add_trace(go.Scatter(x=hist_time, y=hist_q_vent, name="Ventilator Demand", line=dict(color='orange', dash='dash', width=2), hovertemplate="%{y:.1f} NLPM | %{customdata}"), row=2, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_q_mist, name="Total Supply to Tank", line=dict(color='green', width=2), hovertemplate="%{y:.1f} NLPM | %{customdata}"), row=2, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_flow_compressor, name="Total Supply from compressor", line=dict(color='violet', width=2), hovertemplate="%{y:.1f} NLPM | %{customdata}"), row=2, col=1)

    # --- ROW 3: Drain Leakages ---
    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_af, name="Air Filter Leak", line=dict(color='green', dash='dot', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_ws, name="Water Sep Leak", line=dict(color='blue', dash='dot', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_ms, name="Mist Sep Leak", line=dict(color='purple', dash='dot', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)

    # --- ROW 4: Temperatures ---
    fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_head_c, name="Compressor Head Temp", line=dict(color='red', width=2), hovertemplate="%{y:.1f} °C | %{customdata}"), row=4, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_motor_c, name="Motor Temp", line=dict(color='darkred', dash='dot', width=2), hovertemplate="%{y:.1f} °C | %{customdata}"), row=4, col=1)

    # --- ROW 5: Power ---
    fig.add_trace(go.Scatter(x=hist_time, y=hist_power, name="Power Consumption", line=dict(color='purple', width=2), hovertemplate="%{y:.0f} W | %{customdata}"), row=5, col=1)

    # --- ROW 6: Motor Control ---
    fig.add_trace(go.Scatter(x=hist_time, y=hist_pwm, name="Motor PWM", line=dict(color='black', width=2), hovertemplate="%{y:.0f} % | %{customdata}"), row=6, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_blower_cfm, name="Blower CFM", line=dict(color='teal', dash='dash', width=2), hovertemplate="%{y:.1f} CFM | %{customdata}"), row=6, col=1)

    # =====================================================================
    # DYNAMIC HOVER TIME FORMATTING
    # =====================================================================
    formatted_hover_times = []
    for s in hist_time:
        if s < 60:
            formatted_hover_times.append(f"{s:.1f}s")
        elif s < 3600:
            m = int(s // 60)
            sec = s % 60
            formatted_hover_times.append(f"{m}m {sec:.1f}s")
        else:
            h = int(s // 3600)
            m = int((s % 3600) // 60)
            sec = s % 60
            formatted_hover_times.append(f"{h}h {m}m {sec:.0f}s")

    # Only inject the custom string now. Do NOT globally overwrite the hovertemplate.
    fig.update_traces(customdata=formatted_hover_times)

    # =====================================================================
    # Standard UI & Layout Upgrades
    # =====================================================================
    fig.update_layout(
        title="Digital Twin Dashboard",
        height=1400,          
        template="plotly_white", 
        hovermode="x unified",   
        showlegend=True,
        legend=dict(orientation="v", yanchor="top", y=1.0, xanchor="left", x=1.02)
    )

    fig.update_xaxes(title_text="Time (Seconds)", row=6, col=1)
    fig.update_yaxes(range=[0, 105], row=6, col=1)

    fig.show()




























    # # =================================================================
    # hist_time = []
    # hist_p_tank = []
    # hist_pwm = []
    # hist_power = []
    # hist_temp_motor_c = []
    # hist_temp_head_c = []
    # hist_q_mist = []
    # hist_q_vent = []
    # hist_blower_cfm = []

    # # Initialize variables for t=0
    # current_blower_cfm = 0.0 
    # current_compressor_flow_nlpm = 0.0 
    # q_nlpm_mist = 0.0          # Changed from None to 0.0 to prevent crash on loop 1
    # requested_pwm = 100.0      # Assume 100% demand on startup

    # actual_system_friction_pa = 0.0
    # p_tank_abs_pa = 101325.0
    # actual_intake_m_dot_kg_s = 0.0 # Tracks suction mass flow

    # for t in time:
        
    #     # =====================================================================
    #     # 1. INTAKE DYNAMICS (Room -> Cabinet -> HEPA -> Silencer -> Hose)
    #     # =====================================================================
    #     # Convert previous timestep's mass flow into volume flow for the filters
    #     rho_room = 101325.0 / (287.05 * temp_room_k) 
    #     intake_flow_lpm = (actual_intake_m_dot_kg_s / rho_room) * 1000.0 * 60.0
    #     intake_flow_cfm = intake_flow_lpm * 0.0353147

    #     # A. Cabinet Filter (Louvered IP54)
    #     dp_cabinet_pa = cf.calculate_filter_pressure_drop(flow_rate_cfm=intake_flow_cfm)
    #     p_after_cabinet_pa = 101325.0 - dp_cabinet_pa

    #     # B. HEPA Filter (ZF-111 Non-linear Curve)
    #     Hepa.update_loading(intake_flow_lpm, dt_s)
    #     dp_hepa_pa = Hepa.pressure_drop(intake_flow_lpm)
    #     p_after_hepa_pa = p_after_cabinet_pa - dp_hepa_pa

    #     # C. Acoustic Silencer Chamber (Dynamic Inertia Buffer)
    #     p_silencer_pa, _ = Silencer.update_state(
    #         P_upstream_pa=p_after_hepa_pa, 
    #         m_dot_out_kg_s=actual_intake_m_dot_kg_s, 
    #         dt_s=dt_s
    #     )

    #     # D. Intake Hose (Placeholder logic)
    #     dp_intake_hose_pa = 200.0 * (intake_flow_lpm / 100.0)**2 
    #     p_comp_intake_pa = p_silencer_pa - dp_intake_hose_pa

    #     # Safety Clamp: Prevent total vacuum from crashing the compressor math
    #     p_comp_intake_pa = max(10000.0, p_comp_intake_pa)


    #     # =====================================================================
    #     # 2. THE COMPRESSOR (The Muscle)
    #     # =====================================================================
    #     p_comp_discharge_pa = p_tank_abs_pa + actual_system_friction_pa

    #     (current_compressor_flow_nlpm, current_amps, temp_out_gas_k, 
    #      act_pwm, temp_motor_k, temp_head_k, power_w) = comp.update_system_state(
    #         p_up_pa = p_comp_intake_pa,          # <--- Intake Suction Penalty applied!
    #         temp_up_k = temp_room_k, 
    #         p_down_pa = p_comp_discharge_pa,     
    #         req_pump_pwm = requested_pwm,        
    #         req_blower_cfm = current_blower_cfm, 
    #         dt_s = dt_s
    #     )
        
    #     # m_dot_kg_s = (current_compressor_flow_nlpm / 1000.0 / 60.0) * comp.rho_normal
    #     # # Update the intake tracker so the filters feel the suction on the NEXT loop!
    #     # actual_intake_m_dot_kg_s = m_dot_kg_s 


    #     m_dot_kg_s = (current_compressor_flow_nlpm / 1000.0 / 60.0) * comp.rho_normal
        
    #     # FLUID MOMENTUM FIX: Air mass cannot accelerate instantly. 
    #     # We blend 80% of the old flow with 20% of the new flow to simulate physical inertia.
    #     actual_intake_m_dot_kg_s = (0.8 * actual_intake_m_dot_kg_s) + (0.2 * m_dot_kg_s)


    #     # =====================================================================
    #     # 3. DISCHARGE HOSE & COOLING COIL
    #     # =====================================================================
    #     P_out_pa_hose, T_out_k_hose = hose.calculate_hose_state(
    #         m_dot_kg_s = m_dot_kg_s,
    #         P_in_pa = p_comp_discharge_pa,       
    #         T_in_k = temp_out_gas_k,
    #         T_amb_k = temp_room_k 
    #     )

    #     T_out_k_coil, T_ambient_out_celsius, P_out_pa_coil = Cooling_coil.analyze_coil(
    #         m_dot_kg_s = m_dot_kg_s,
    #         P_in_pa = P_out_pa_hose,  
    #         T_in_k = T_out_k_hose,
    #         T_amb_k = temp_room_k,
    #         selected_length = 3.36 
    #     )
    #     comp.temp_air = T_ambient_out_celsius + 273.15


    #     # =====================================================================
    #     # 4. NRV & SEPARATORS
    #     # =====================================================================
    #     P_out_pa_nrv, delta_p_mbar_nrv, T_out_k_nrv = Nrv.calculate_valve_state(
    #         m_dot_kg_s = m_dot_kg_s,
    #         P_in_pa = P_out_pa_coil,
    #         T_in_k = T_out_k_coil
    #     )

    #     P_out_pa_filter, delta_p_mbar_filter, dp_viscous_mbar, dp_inertial_mbar, m_dot_out_kg_s_air_filter = Air_filter.calculate_filter_state(
    #         m_dot_kg_s = m_dot_kg_s,
    #         P_in_pa = P_out_pa_nrv,
    #         T_in_k = T_out_k_nrv
    #     )

    #     P_out_pa_water, delta_p_mbar_water, m_dot_effective_water, water_escaped_mg_s, vapor_passed_mg_s = Water_sep.update_state(
    #         m_dot_in_kg_s = m_dot_out_kg_s_air_filter,
    #         P_in_pa = P_out_pa_filter,
    #         T_in_k = T_out_k_nrv,
    #         T_amb_C = temp_room_k - 273.15,
    #         RH_amb = 0.5
    #     )

    #     P_out_pa_mist, delta_p_mbar_mist, q_nlpm_mist, mist_escaped_mg_s = Mist_sep.update_state(
    #         m_dot_in_kg_s = m_dot_effective_water,
    #         P_in_pa = P_out_pa_water,
    #         T_in_k = T_out_k_nrv,
    #         aerosol_water_in_mg_s= vapor_passed_mg_s,
    #         dt_seconds = dt_s,
    #     )


    #     # =====================================================================
    #     # 5. THE TANK (The final destination for the air)
    #     # =====================================================================
    #     p_tank_gauge, requested_pwm, q_pump_in, q_vent_out = tank.simulate(
    #         t=t,
    #         dt_s=dt_s,
    #         t_insp=0.5,
    #         flow_insp=50,
    #         t_exp=0.5,
    #         flow_exp=50,
    #         external_inflow_nlpm= q_nlpm_mist  
    #     )
    #     p_tank_abs_pa = (p_tank_gauge * 100000.0) + 101325.0


    #     # =====================================================================
    #     # 6. SYSTEM CONTROLLERS & MATH TRACKERS
    #     # =====================================================================
    #     # RUN PID CONTROLLER based on the new head temperature
    #     fan_pwm = pid.update(temp_head_k - 273.15, dt_s)
    #     fan_cfm = get_cfm_from_pwm(fan_pwm)
    #     current_blower_cfm = max(2 * fan_cfm, 0)  

    #     # # MEASURE EXACT FRICTION FOR THE NEXT TIMESTEP
    #     # actual_system_friction_pa = max(0.0, p_comp_discharge_pa - P_out_pa_mist)


    #     # MEASURE EXACT FRICTION FOR THE NEXT TIMESTEP
    #     # FLUID MOMENTUM FIX: Smooth the friction response to prevent discharge ping-ponging
    #     new_friction_pa = max(0.0, p_comp_discharge_pa - P_out_pa_mist)
    #     actual_system_friction_pa = (0.8 * actual_system_friction_pa) + (0.2 * new_friction_pa)


    #     # =====================================================================
    #     # 7. APPEND TO PLOTTING LISTS (Downsampled for speed)
    #     # =====================================================================
    #     if t % 3.0 < dt_s:
    #         hist_time.append(t)
    #         hist_p_tank.append(p_tank_gauge)
    #         hist_pwm.append(act_pwm * 100.0) 
    #         hist_power.append(power_w) 
    #         hist_temp_motor_c.append(temp_motor_k - 273.15) 
    #         hist_temp_head_c.append(temp_head_k - 273.15)   
    #         hist_q_mist.append(q_nlpm_mist)
    #         hist_q_vent.append(q_vent_out)
    #         hist_blower_cfm.append(current_blower_cfm)


    #     # Print logic
    #     # if t % 75.0 < dt_s:  
    #     #     print(f"Time: {t:.2f} s | Tank Pres: {p_tank_gauge:.2f} Bar | Flow In: {q_nlpm_mist:.1f} NLPM | Blower: {current_blower_cfm:.1f} CFM | Head Temp: {temp_head_k - 273.15:.1f} °C")

    # # =====================================================================
    # # # DASHBOARD GENERATION
    # # =====================================================================
    # print("Generating Interactive Dashboard...")

    # fig = make_subplots(
    #     rows=5, cols=1, 
    #     shared_xaxes=True, 
    #     vertical_spacing=0.05,
    #     subplot_titles=(
    #         "1. Tank Pressure (Bar Gauge)", 
    #         "2. Pneumatic Flows (NLPM)", 
    #         "3. Thermal States (°C)", 
    #         "4. Electrical Power (Watts)", 
    #         "5. Compressor Motor Control (%)"
    #     )
    # )

    # # --- ROW 1: Pressure ---
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_p_tank, name="Tank Pressure", line=dict(color='blue', width=2)), row=1, col=1)

    # # --- ROW 2: Flows (Supply vs Demand) ---
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_q_vent, name="Ventilator Demand", line=dict(color='orange', dash='dash', width=2)), row=2, col=1)
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_q_mist, name="Supplied by Compressor", line=dict(color='green', width=2)), row=2, col=1)

    # # --- ROW 3: Temperatures ---
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_head_c, name="Compressor Head Temp", line=dict(color='red', width=2)), row=3, col=1)
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_motor_c, name="Motor Temp", line=dict(color='darkred', dash='dot', width=2)), row=3, col=1)

    # # --- ROW 4: Power ---
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_power, name="Power Consumption", line=dict(color='purple', width=2)), row=4, col=1)

    # # --- ROW 5: PWM Duty Cycle ---
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_pwm, name="Motor PWM", line=dict(color='black', width=2)), row=5, col=1)
    # fig.add_trace(go.Scatter(x=hist_time, y=hist_blower_cfm, name="Blower CFM", line=dict(color='teal', dash='dash', width=2)), row=5, col=1)

    # # =====================================================================
    # # DYNAMIC HOVER TIME FORMATTING
    # # =====================================================================
    # formatted_hover_times = []
    # for s in hist_time:
    #     if s < 60:
    #         formatted_hover_times.append(f"{s:.1f}s")
    #     elif s < 3600:
    #         m = int(s // 60)
    #         sec = s % 60
    #         formatted_hover_times.append(f"{m}m {sec:.1f}s")
    #     else:
    #         h = int(s // 3600)
    #         m = int((s % 3600) // 60)
    #         sec = s % 60
    #         formatted_hover_times.append(f"{h}h {m}m {sec:.0f}s")

    # fig.update_traces(
    #     customdata=formatted_hover_times,
    #     hovertemplate="%{y:.2f}  |  %{customdata}"
    # )

    # # =====================================================================
    # # Standard UI & Layout Upgrades
    # # =====================================================================
    # fig.update_layout(
    #     title="Digital Twin Telemetry Dashboard",
    #     height=1200,          
    #     template="plotly_white", 
    #     hovermode="x unified",   
    #     showlegend=True,
    #     legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
    # )

    # # Put the main x-axis back to standard numbers
    # fig.update_xaxes(title_text="Time (Seconds)", row=5, col=1)
    # fig.update_yaxes(range=[0, 105], row=5, col=1)

    # fig.show()

    # # # Uncomment these if Plotly hangs in your VS Code terminal!
    # # print("Saving dashboard to 'Digital_Twin_Dashboard.html'...")
    # # fig.write_html("Digital_Twin_Dashboard.html")
    # # print("Done! Open the HTML file in your folder to view it.")