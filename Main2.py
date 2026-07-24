import math
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Ensure your filenames match these exactly
import Pump as Pmp 
import Digital_tank2 as dt
import cooling_coil2 as cc
import Discharge_hose as dh
import NRV as nrv
import air_filter as af
import water_separater as ws
import mist_separator as ms

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
        # Intentional constraint to pin blower to 100% for thermal testing
        self.pwm_min = 20.0
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
        
        if raw_pwm > self.pwm_max or raw_pwm < self.pwm_min:
            self.integral_sum -= error * dt_seconds 
            
        return final_pwm

# ==========================================
# 2. THE SUNON FAN TRANSLATOR & COMPRESSOR RPM
# ==========================================
blower_pwm_data = np.array([0, 25, 40, 50, 60, 67, 77, 85, 100])
blower_rpm_data = np.array([1250, 2275, 2885, 3355, 3960, 4850, 5750, 6250, 6820])
blower_rpm_fit  = np.polyfit(blower_pwm_data, blower_rpm_data, 2)

def get_cfm_from_pwm(pwm_percentage):
    rpm = np.polyval(blower_rpm_fit, pwm_percentage)
    return (rpm / 6820.0) * 53.6 * 2

def predict_speed_pct_for_rpm(target_rpm, pressure_bar):
    # Enforce a strict linear mapping (0 to 2300 RPM = 0 to 100 PWM)
    # The pressure_bar argument is ignored here.
    
    clamped_rpm = max(0.0, min(2300.0, target_rpm))
    speed_pct = (clamped_rpm / 2300.0) * 100.0
    
    return speed_pct


# def predict_speed_pct_for_rpm(target_rpm, pressure_bar):
#     r = max(0.0, target_rpm) / 1000.0
#     p = max(0.0, pressure_bar)
#     speed_pct = (
#         0.8105
#         + 32.2281 * r
#         + 1.6880 * p
#         + 3.3907 * r**2
#         - 1.8009 * r * p
#         - 0.0380 * p**2
#         - 0.8592 * r**3
#         + 0.4832 * r**2 * p
#         + 0.0035 * r * p**2
#         + 0.0057 * p**3
#     )
#     return max(0.0, min(100.0, speed_pct))

# =====================================================================
# SIMULATION EXECUTION (MAIN LOOP)
# =====================================================================
if __name__ == "__main__":
    
    # =======================================================
    # CENTRALIZED SYSTEM ENVIRONMENTAL CONSTANTS
    # Change these here and they apply to the entire model
    # =======================================================
    T_AMB_C  = 35.0                # Ambient Room Temperature in Celsius
    T_AMB_K  = 273.15 + T_AMB_C    # Converted to Kelvin
    RH_AMB   = 0.50                # 50% Relative Humidity
    P_ATM_PA = 101325.0            # Standard Atmospheric Pressure
    # =======================================================

    # Initialize all subsystems
    comp = Pmp.SmartCompressor(T_amb_k=T_AMB_K, pump_model="140RND", filter_hours=0)
    pid = ThermalPIDController(kp=4.0, ki=0.5, kd=0.1, target_temp_c=50.0)
    hose = dh.DischargeHose_HiPoFlex()
    Cooling_coil = cc.CopperCoolingCoil_TwinFan()
    Nrv = nrv.SMC_AKH10_NRV()
    Air_filter = af.SMC_AF20_Filter(D2=0.60)
    Water_sep = ws.SMC_AFG20_WaterSeparator(D2=0.6)
    Mist_sep = ms.SMC_AFM20_MistSeparator(D2=0.6)
    tank = dt.SmartCompressorTankTwin(volume_liters=2.0, motor_voltage_v=24.0, D2=0.6)
    
    # Overwrite all module defaults with Centralized Variables
    comp.temp_ambient_k = T_AMB_K 
    comp.temp_motor_k = T_AMB_K
    comp.temp_head_k = T_AMB_K
    tank.T_k = T_AMB_K

    dt_s = 0.05
    total_time_s = 60
    time_arr = np.arange(0, total_time_s, dt_s)

    # Data Tracking Arrays
    hist_time, hist_p_tank, hist_pwm, hist_power = [], [], [], []
    hist_temp_motor_c, hist_temp_head_c, hist_temp_out_gas_c, hist_temp_in_gas_c = [], [], [], []
    hist_q_mist, hist_q_comp, hist_q_vent, hist_blower_cfm = [], [], [], []
    hist_p_hepa, hist_p_silencer, hist_p_intake, hist_p_comp = [], [], [], []
    hist_flow_compressor, hist_p_hose, hist_p_coil = [], [], []
    hist_p_af, hist_p_ws, hist_p_ms = [], [], []
    hist_delta_P_af, hist_delta_P_wf, hist_delta_P_ms, hist_delta_APU = [], [], [], []
    hist_leak_af, hist_leak_ws, hist_leak_ms, hist_leak_tank, hist_leak_total = [], [], [], [], []

    current_blower_pwm = 0.0 
    current_compressor_flow_nlpm = 0.0 
    q_nlpm_mist = 0.0          
    
    # --- CONTROL OVERRIDES ---
    pump_control_mode = "MANUAL"  # "MANUAL" or "PID"
    pump_command_mode = "RPM"     # "PWM" or "RPM"
    manual_pump_pwm = 100.0       
    manual_target_rpm = 2300.0    
    
    p_tank_gauge = 0.0
    tank_requested_pwm = 100.0
    startup_ramp_duration_s = 0.3
    actual_system_friction_pa = 0.0
    p_tank_abs_pa = P_ATM_PA

    for t in time_arr:

        # 1. DETERMINE PUMP COMMAND (0-100 Scale)
        if pump_control_mode == "MANUAL":
            if pump_command_mode == "RPM":
                manual_pump_pwm = predict_speed_pct_for_rpm(manual_target_rpm, p_tank_gauge)
            requested_pwm = manual_pump_pwm
        else:
            requested_pwm = tank_requested_pwm

        # 2. THE COMPRESSOR
        p_comp_discharge_pa = p_tank_abs_pa + actual_system_friction_pa
        startup_ramp_factor = min(1.0, t / startup_ramp_duration_s) if startup_ramp_duration_s > 0 else 1.0
        effective_pump_pwm = requested_pwm * startup_ramp_factor

        # FIX: PWM scaled 0-1, 10 variables unpacked, Ambient mapped centrally
        (current_compressor_flow_nlpm, current_amps, temp_out_gas_k, 
         act_pwm, temp_motor_k, temp_head_k, power_w, p_intake_pa, 
         p_after_hepa, p_silencer_pa) = comp.update_system_state(
            voltage_v=24.0,        
            temp_up_k=T_AMB_K, 
            p_down_abs_pa=p_comp_discharge_pa,     
            req_pump_pwm=(effective_pump_pwm / 100.0),        
            req_blower_pwm=(current_blower_pwm / 100.0), 
            dt_s=dt_s
        )
        
        m_dot_kg_s = (current_compressor_flow_nlpm / 1000.0 / 60.0) * comp.rho_normal

        # 3. DISCHARGE HOSE & COOLING COIL
        # FIX: Unpacking 3 variables
        P_out_pa_hose, T_out_k_hose, heat_loss_hose = hose.calculate_hose_state(
            m_dot_kg_s=m_dot_kg_s,
            P_in_pa=p_comp_discharge_pa,       
            T_in_k=temp_out_gas_k,
            T_amb_k=T_AMB_K 
        )

        # FIX: Unpacking 4 variables
        T_out_k_coil, P_out_pa_coil, dp_coil_pa = Cooling_coil.analyze_coil(
            m_dot_kg_s=m_dot_kg_s,
            P_in_pa=P_out_pa_hose,  
            T_in_k=T_out_k_hose,
            T_amb_k=T_AMB_K,
            selected_length=3.36 
        )

        # 4. NRV & SEPARATORS
        P_out_pa_nrv, delta_p_mbar_nrv, T_out_k_nrv = Nrv.calculate_valve_state(
            m_dot_kg_s=m_dot_kg_s,
            P_in_pa=P_out_pa_coil,
            T_in_k=T_out_k_coil
        )

        P_out_pa_filter, delta_p_mbar_filter, m_dot_out_kg_s_air_filter, air_filter_leak_nlpm = Air_filter.calculate_filter_state(
            t=t,
            m_dot_kg_s=m_dot_kg_s,
            P_in_pa=P_out_pa_nrv,
            T_in_k=T_out_k_nrv,
            P_atm_pa=P_ATM_PA
        )

        P_out_pa_water, delta_p_mbar_water, m_dot_effective_water, water_escaped_mg_s, vapor_passed_mg_s, water_leak_nlpm = Water_sep.update_state(
            t=t,
            m_dot_in_kg_s=m_dot_out_kg_s_air_filter,
            P_in_pa=P_out_pa_filter,
            T_in_k=T_out_k_nrv,
            T_amb_C=T_AMB_C,
            RH_amb=RH_AMB,
            P_amb_pa=P_ATM_PA
        )

        P_out_pa_mist, delta_p_mbar_mist, q_nlpm_mist, mist_escaped_mg_s, mist_leak_nlpm, m_dot_effective_mist = Mist_sep.update_state(
            t=t,
            m_dot_in_kg_s=m_dot_effective_water,
            P_in_pa=P_out_pa_water,
            T_in_k=T_out_k_nrv,
            aerosol_water_in_mg_s=vapor_passed_mg_s,
            dt_seconds=dt_s
        )

        # 5. THE TANK 
        p_tank_gauge, tank_requested_pwm, q_pump_in, q_vent_out, tank_leak = tank.simulate(
            t=t,
            T_in_K=T_out_k_nrv,
            dt_s=dt_s,
            t_insp=0.5,
            flow_insp=80,
            t_exp=0.5,
            flow_exp=80,
            external_inflow_nlpm=q_nlpm_mist
        )

        p_tank_abs_pa = (p_tank_gauge * 100000.0) + P_ATM_PA

        # 6. SYSTEM CONTROLLERS
        fan_pwm = pid.update(temp_head_k - 273.15, dt_s)
        current_blower_pwm = max(fan_pwm, 0.0)
        current_blower_cfm = get_cfm_from_pwm(current_blower_pwm)  

        new_friction_pa = max(0.0, p_comp_discharge_pa - P_out_pa_mist)
        actual_system_friction_pa = (0.9 * actual_system_friction_pa) + (0.1 * new_friction_pa)

        # 7. METRICS & APPEND (Downsampled to 2 seconds)
        if t % 2.0 < dt_s:
            hist_p_comp.append((p_comp_discharge_pa - P_ATM_PA) / 100000.0)
            hist_flow_compressor.append(current_compressor_flow_nlpm)
            hist_p_hose.append((P_out_pa_hose - P_ATM_PA) / 100000.0)
            hist_p_coil.append((P_out_pa_coil - P_ATM_PA) / 100000.0)
            hist_p_af.append((P_out_pa_filter - P_ATM_PA) / 100000.0)
            hist_p_ws.append((P_out_pa_water - P_ATM_PA) / 100000.0)
            hist_p_ms.append((P_out_pa_mist - P_ATM_PA) / 100000.0)
            hist_delta_P_af.append(delta_p_mbar_filter/1000)
            hist_delta_P_wf.append(delta_p_mbar_water/1000)
            hist_delta_P_ms.append(delta_p_mbar_mist/1000)
            hist_p_tank.append(p_tank_gauge)
            hist_delta_APU.append((P_out_pa_coil - P_out_pa_mist) / 100000.0)
            hist_p_intake.append((p_intake_pa - P_ATM_PA) / 100000.0)
            hist_p_hepa.append((p_after_hepa - P_ATM_PA) / 100000.0)
            hist_p_silencer.append((p_silencer_pa - P_ATM_PA) / 100000.0)

            to_nlpm = (1.0 / comp.rho_normal) * 60000.0
            
            af_leak = max(0.0, (m_dot_kg_s - m_dot_out_kg_s_air_filter) * to_nlpm)
            ws_leak = max(0.0, (m_dot_out_kg_s_air_filter - m_dot_effective_water) * to_nlpm)
            ms_leak = max(0.0, ((m_dot_effective_water)*to_nlpm - q_nlpm_mist))
            
            hist_leak_af.append(af_leak)
            hist_leak_ws.append(ws_leak)
            hist_leak_ms.append(ms_leak)
            hist_leak_tank.append(tank_leak)
            hist_leak_total.append(af_leak + ws_leak + ms_leak + tank_leak)
            
            hist_time.append(t)
            hist_pwm.append(act_pwm * 100.0) 
            hist_power.append(power_w) 
            hist_temp_motor_c.append(temp_motor_k - 273.15) 
            hist_temp_head_c.append(temp_head_k - 273.15)  
            hist_temp_out_gas_c.append(temp_out_gas_k - 273.15)
            hist_temp_in_gas_c.append(T_out_k_nrv - 273.15) 
            hist_q_comp.append(current_compressor_flow_nlpm)
            hist_q_mist.append(q_nlpm_mist)
            hist_q_vent.append(q_vent_out)
            hist_blower_cfm.append(current_blower_cfm)

    # DASHBOARD GENERATION
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

    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_silencer, name="0.5 After Silencer", line=dict(color='gold', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_intake, name="0.75 After Intake", line=dict(color='yellow', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_comp, name="1. After Compressor", line=dict(color='darkred', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_hose, name="2. After Hose", line=dict(color='orangered', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_coil, name="3. After Coil", line=dict(color='orange', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_af, name="4. After Air Filter", line=dict(color='green', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_ws, name="8. After water separator", line=dict(color='purple', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_ms, name="9. After Mist Sep", line=dict(color='black', width=3), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_p_tank, name="10. Final Tank Pressure", line=dict(color='blue', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_P_af, name="5. Delta_P Air Filter", line=dict(color='skyblue', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_P_wf, name="6. Delta_P water Filter ", line=dict(color='green', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_P_ms, name="7. Delta_P Mist Filter", line=dict(color='green', width=1), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_delta_APU, name="Delta P Across APU", line=dict(color='teal', dash='dash', width=2), hovertemplate="%{y:.4f} Bar | %{customdata}"), row=1, col=1)
    
    fig.add_trace(go.Scatter(x=hist_time, y=hist_q_vent, name="Ventilator Demand", line=dict(color='orange', dash='dash', width=2), hovertemplate="%{y:.1f} NLPM | %{customdata}"), row=2, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_q_mist, name="Total Supply to Tank", line=dict(color='green', width=2), hovertemplate="%{y:.1f} NLPM | %{customdata}"), row=2, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_flow_compressor, name="Total Supply from compressor", line=dict(color='violet', width=2), hovertemplate="%{y:.1f} NLPM | %{customdata}"), row=2, col=1)

    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_af, name="Air Filter Leak", line=dict(color='green', dash='dot', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_ws, name="Water Sep Leak", line=dict(color='blue', dash='dot', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_ms, name="Mist Sep Leak", line=dict(color='purple', dash='dot', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_tank, name="Tank Leak", line=dict(color='red', dash='dot', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_leak_total, name="Total Leak", line=dict(color='black', width=2), hovertemplate="%{y:.4f} NLPM | %{customdata}"), row=3, col=1)

    fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_head_c, name="Compressor Head Temp", line=dict(color='red', width=2), hovertemplate="%{y:.1f} °C | %{customdata}"), row=4, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_motor_c, name="Motor Temp", line=dict(color='darkred', dash='dot', width=2), hovertemplate="%{y:.1f} °C | %{customdata}"), row=4, col=1)

    fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_in_gas_c, name="Gas Temp Inlet", line=dict(color='orange', width=2), hovertemplate="%{y:.1f} °C | %{customdata}"), row=4, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_temp_out_gas_c, name="Gas Temp Outlet", line=dict(color='gold', dash='dot', width=2), hovertemplate="%{y:.1f} °C | %{customdata}"), row=4, col=1)

    fig.add_trace(go.Scatter(x=hist_time, y=hist_power, name="Power Consumption", line=dict(color='purple', width=2), hovertemplate="%{y:.0f} W | %{customdata}"), row=5, col=1)

    fig.add_trace(go.Scatter(x=hist_time, y=hist_pwm, name="Motor PWM", line=dict(color='black', width=2), hovertemplate="%{y:.0f} % | %{customdata}"), row=6, col=1)
    fig.add_trace(go.Scatter(x=hist_time, y=hist_blower_cfm, name="Blower CFM", line=dict(color='teal', dash='dash', width=2), hovertemplate="%{y:.1f} CFM | %{customdata}"), row=6, col=1)

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

    fig.update_traces(customdata=formatted_hover_times)

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