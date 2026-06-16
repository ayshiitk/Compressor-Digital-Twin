import math
import numpy as np
import matplotlib.pyplot as plt
import Digital_tank as dt
import cooling_coil as cc
import Discharge_hose as dh
import NRV as nrv
import air_filter as af
import water_separater as ws
import mist_separator as ms
import plotly.graph_objects as go
from plotly.subplots import make_subplots
# import Silencer as sl
# import Hepa as hp
# import Cabinet_filter as cf




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
def get_cfm_from_pwm(pwm_percentage):
    """
    Translates PWM% to CFM based on the SUNON PF97332BX datasheet.
    0% PWM = 1200 RPM. 100% PWM = 6800 RPM. Max CFM = 53.6.
    """
    rpm = 1200.0 + (pwm_percentage / 100.0) * (6800.0 - 1200.0)
    # Assuming linear scaling of flow with RPM for a fixed static system
    actual_cfm = (rpm / 6800.0) * 53.6
    return actual_cfm

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
        self.temp_air = 40 + 273.15
        self.temp_motor_k = 293.15
        self.temp_head_k = 40 + 273.15  # Heads start cooler due to direct contact with incoming air

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
        test3_blower_cfm = np.array([0.0, 18.9, 53.6, 80.0, 107.2])
        test3_amps = np.array([24.0, 24.0, 24.0, 24.0, 24.0])        
        test3_pressure_bar = np.array([4.0, 4.0, 4.0, 4.0, 4.0])   
        test3_temp_head_c = np.array([88.0, 72.0, 54.0, 49.0, 45.0])      
        test3_temp_room_c = np.array([40.0, 40.0, 40.0, 40.0, 40.0])       
        test3_temp_gas_out_c = np.array([130.0, 112.0, 88.0, 80.0, 75.0])  

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
        cooling_motor_w = (self.temp_motor_k - self.temp_air) / Rth_motor
        cooling_head_w = (self.temp_head_k - self.temp_air) / Rth_head
        
        # Update temperatures: dT = (Heat_in - Heat_out) / Thermal_Mass * time_step
        self.temp_motor_k += ((heat_motor_w - cooling_motor_w) / self.Cth_motor) * dt_s
        self.temp_head_k += ((heat_head_w - cooling_head_w) / self.Cth_head) * dt_s
            
        return flow_nlpm, current_amps, temp_out_gas_k, actual_pwm, self.temp_motor_k, self.temp_head_k, total_electrical_power_w


# =====================================================================
# SIMULATION EXECUTION (MAIN LOOP)
# =====================================================================
if __name__ == "__main__":
    # Initialize the compressor
    comp = SmartCompressor_120RND()
    pid = ThermalPIDController(kp=4.0, ki=0.5, kd=0.1, target_temp_c=50.0)
    hose = dh.DischargeHose_HiPoFlex()
    Cooling_coil = cc.CopperCoolingCoil_TwinFan()
    Nrv = nrv.SMC_AKH10_NRV()
    Air_filter = af.SMC_AF20_Filter()
    Water_sep = ws.SMC_AFG20_WaterSeparator()
    Mist_sep = ms.SMC_AFM20_MistSeparator()
    # Silencer = sl.AcousticSilencerChamber()
    # Hepa = hp.ZF111_HEPA_Filter()
    # Cabinet_filter = cf.calculate_filter_pressure_drop()    

    # Air_filter = af.SMC_AF20_Filter(valve_closed= True  )  # Simulating the unvalved 1.8mm leak
    # Water_sep = ws.SMC_AFG20_WaterSeparator(valve_closed= True  )  # Simulating the unvalved 1.8mm leak
    # Mist_sep = ms.SMC_AFM20_MistSeparator(valve_closed= True  )  # Simulating the unvalved 1.8mm leak
    
    # System Constants
    pressure_intake_pa = 100000.0   
    temp_room_k = 273.15 + 35.0    # 35°C Room
    
    # Sync the compressor's ambient cooling temp to the room temp
    comp.temp_motor_k = temp_room_k
    comp.temp_head_k = temp_room_k

    # We need to track the compressor's suction mass flow to feed the filters
    # actual_intake_m_dot_kg_s = 0.0

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
    hist_q_vent = []
    hist_blower_cfm = []


    
    # Initialize variables for t=0
    current_blower_cfm = 0.0 
    current_compressor_flow_nlpm = 0.0 
    q_nlpm_mist = None # Starts at 0

    actual_system_friction_pa = 0.0
    p_tank_abs_pa = 101325.0

    for t in time:
        # 1. TANK

        p_tank_gauge, requested_pwm, q_pump_in, q_vent_out = tank.simulate(
            t=t,
            dt_s=dt_s,
            t_insp=0.5,
            flow_insp=80,
            t_exp=0.5,
            flow_exp=80,
            external_inflow_nlpm= q_nlpm_mist  # <--- BRIDGE CONNECTED HERE
        )

        # print(p_tank_gauge)
        p_tank_abs_pa = (p_tank_gauge * 100000.0) + 101325.0




        # =====================================================================
        # 2. THE COMPRESSOR
        # =====================================================================
        # EXACT CALCULATION: Tank Pressure + The exact friction measured on the last loop
        p_comp_discharge_pa = p_tank_abs_pa + actual_system_friction_pa

        (current_compressor_flow_nlpm, current_amps, temp_out_gas_k, 
         act_pwm, temp_motor_k, temp_head_k, power_w) = comp.update_system_state(
            p_up_pa = pressure_intake_pa, 
            temp_up_k = temp_room_k, 
            p_down_pa = p_comp_discharge_pa,     # <--- Uses the exact calculated pressure
            req_pump_pwm = requested_pwm,        
            req_blower_cfm = current_blower_cfm, 
            dt_s = dt_s
        )
        m_dot_kg_s = (current_compressor_flow_nlpm / 1000.0 / 60.0) * comp.rho_normal

        # =====================================================================
        # 3. DISCHARGE HOSE
        # =====================================================================
        P_out_pa_hose, T_out_k_hose = hose.calculate_hose_state(
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = p_comp_discharge_pa,       # <--- Hose starts at the exact discharge pressure
            T_in_k = temp_out_gas_k,
            T_amb_k = temp_room_k 
        )
#        COMPRESSOR

        # (current_compressor_flow_nlpm, current_amps, temp_out_gas_k, 
        #  act_pwm, temp_motor_k, temp_head_k, power_w) = comp.update_system_state(
        #     p_up_pa = pressure_intake_pa, 
        #     temp_up_k = temp_room_k, 
        #     p_down_pa = p_tank_abs_pa,           # Pushing against tank pressure
        #     req_pump_pwm = requested_pwm,        # Speed commanded by tank logic
        #     req_blower_cfm = current_blower_cfm, # Cooling from PID
        #     dt_s = dt_s
        # )
        # m_dot_kg_s = (current_compressor_flow_nlpm / 1000.0 / 60.0) * comp.rho_normal

        # #  HOSE

        # P_out_pa_hose, T_out_k_hose = hose.calculate_hose_state(
        #     m_dot_kg_s = m_dot_kg_s,
        #     P_in_pa = p_tank_abs_pa,  
        #     T_in_k = temp_out_gas_k,
        #     T_amb_k = temp_room_k 
        # )

#       COOLING_COIL
        T_out_k_coil, T_ambient_out_celsius, P_out_pa_coil = Cooling_coil.analyze_coil(
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = P_out_pa_hose,  
            T_in_k = T_out_k_hose,
            T_amb_k = temp_room_k,
            selected_length = 3.36 
        )


        comp.temp_air = T_ambient_out_celsius + 273.15

        # NRV
        P_out_pa_nrv, delta_p_mbar_nrv, T_out_k_nrv = Nrv.calculate_valve_state(
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = P_out_pa_coil,
            T_in_k = T_out_k_coil
        )
#       AIR_FILTER

        P_out_pa_filter, delta_p_mbar_filter, dp_viscous_mbar, dp_inertial_mbar, m_dot_out_kg_s_air_filter = Air_filter.calculate_filter_state(
            m_dot_kg_s = m_dot_kg_s,
            P_in_pa = P_out_pa_nrv,
            T_in_k = T_out_k_nrv
        )
        # WATER_SEPARATOR
        P_out_pa_water, delta_p_mbar_water, m_dot_effective_water, water_escaped_mg_s, vapor_passed_mg_s = Water_sep.update_state(
            m_dot_in_kg_s = m_dot_out_kg_s_air_filter,
            P_in_pa = P_out_pa_filter,
            T_in_k = T_out_k_nrv,
            T_amb_C = temp_room_k - 273.15,
            RH_amb = 0.5
        )
        # MIST SEPARATOR 
        P_out_pa_mist, delta_p_mbar_mist, q_nlpm_mist, mist_escaped_mg_s = Mist_sep.update_state(
            m_dot_in_kg_s = m_dot_effective_water,
            P_in_pa = P_out_pa_water,
            T_in_k = T_out_k_nrv,
            aerosol_water_in_mg_s= vapor_passed_mg_s,
            dt_seconds = dt_s,
        )

    
        # Convert tank gauge pressure to Absolute Pascals for the thermodynamic math
        # p_tank_abs_pa = (p_tank_gauge * 100000.0) + 101325.0
        
        # 3. RUN PID CONTROLLER based on the new head temperature
        fan_pwm = pid.update(temp_head_k - 273.15, dt_s)
        
        # 4. Translate PID PWM to actual CFM for the next time step
        fan_cfm = get_cfm_from_pwm(fan_pwm)
        current_blower_cfm = max(2 * fan_cfm, 0)  

        # =====================================================================
        # MEASURE EXACT FRICTION FOR THE NEXT TIMESTEP
        # =====================================================================
        # Total friction = The pressure leaving the compressor MINUS the pressure that survived to the Mist Separator
        actual_system_friction_pa = max(0.0, p_comp_discharge_pa - P_out_pa_mist)

        if t % 2 < dt_s:
            hist_time.append(t)
            hist_p_tank.append(p_tank_gauge)
            hist_pwm.append(act_pwm * 100.0) 
            hist_power.append(power_w) 
            hist_temp_motor_c.append(temp_motor_k - 273.15) 
            hist_temp_head_c.append(temp_head_k - 273.15)   
            hist_q_mist.append(q_nlpm_mist)
            hist_q_vent.append(q_vent_out)
            hist_blower_cfm.append(current_blower_cfm)

        # Print logic
        # if t % 2.0 < dt_s:  
        #     print(f"Time: {t:.2f} s | Tank Pres: {p_tank_gauge:.2f} Bar | Flow In: {q_nlpm_mist:.1f} NLPM | Blower: {current_blower_cfm:.1f} CFM | Head Temp: {temp_head_k - 273.15:.1f} °C")

print("Generating Interactive Dashboard...")

fig = make_subplots(
    rows=5, cols=1, 
    shared_xaxes=True, 
    vertical_spacing=0.05,
    subplot_titles=(
        "1. Tank Pressure (Bar Gauge)", 
        "2. Pneumatic Flows (NLPM)", 
        "3. Thermal States (°C)", 
        "4. Electrical Power (Watts)", 
        "5. Compressor Motor Control (%)"
    )
)

# --- ROW 1: Pressure ---
fig.add_trace(go.Scatter(x=hist_time, y=hist_p_tank, name="Tank Pressure", line=dict(color='blue', width=2)), row=1, col=1)

# --- ROW 2: Flows (Supply vs Demand) ---
fig.add_trace(go.Scatter(x=hist_time, y=hist_q_vent, name="Ventilator Demand", line=dict(color='orange', dash='dash', width=2)), row=2, col=1)
fig.add_trace(go.Scatter(x=hist_time, y=hist_q_mist, name="Supplied by Compressor", line=dict(color='green', width=2)), row=2, col=1)

# --- ROW 3: Temperatures ---
fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_head_c, name="Compressor Head Temp", line=dict(color='red', width=2)), row=3, col=1)
fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_motor_c, name="Motor Temp", line=dict(color='darkred', dash='dot', width=2)), row=3, col=1)

# --- ROW 4: Power ---
fig.add_trace(go.Scatter(x=hist_time, y=hist_power, name="Power Consumption", line=dict(color='purple', width=2)), row=4, col=1)

# --- ROW 5: PWM Duty Cycle ---
fig.add_trace(go.Scatter(x=hist_time, y=hist_pwm, name="Motor PWM", line=dict(color='black', width=2)), row=5, col=1)
fig.add_trace(go.Scatter(x=hist_time, y=hist_blower_cfm, name="Blower CFM", line=dict(color='teal', dash='dash', width=2)), row=5, col=1)

# UI & Layout Upgrades
fig.update_layout(
    title="Digital Twin Telemetry Dashboard",
    height=1200,          
    template="plotly_white", 
    hovermode="x unified",   # The magic hover feature!
    showlegend=True,
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
)

fig.update_xaxes(title_text="Time (Seconds)", row=5, col=1)
fig.update_yaxes(range=[0, 105], row=5, col=1)

# =====================================================================
# DYNAMIC HOVER TIME FORMATTING
# =====================================================================
# 1. Generate the formatted time string for every single data point
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

# 2. Inject this custom text into EVERY line on the graph
# Using 'update_traces' saves you from having to type it into all 8 lines manually!
fig.update_traces(
    customdata=formatted_hover_times,
    hovertemplate="%{y:.2f}  |  %{customdata}"
)

# =====================================================================
# 3. Standard UI & Layout Upgrades
# =====================================================================
fig.update_layout(
    title="Digital Twin Telemetry Dashboard",
    height=1200,          
    template="plotly_white", 
    hovermode="x unified",   
    showlegend=True,
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
)

# Put the main x-axis back to standard numbers
fig.update_xaxes(title_text="Time (Seconds)", row=5, col=1)
fig.update_yaxes(range=[0, 105], row=5, col=1)

fig.show()


# print("Saving dashboard to 'Digital_Twin_Dashboard.html'...")
# fig.write_html("Digital_Twin_Dashboard.html")
# print("Done! Open the HTML file in your folder to view it.")