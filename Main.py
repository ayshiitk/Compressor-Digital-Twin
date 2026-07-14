import math
import numpy as np
import Pump_final as Pmp
import Digital_tank as dt
import cooling_coil2 as cc
import Discharge_hose as dh
import NRV as nrv
import air_filter as af
import water_separater as ws
import mist_separator as ms
import Silencer as sl
import Hepa as hp
import Cabinet_filter as cf
import Intake_hose as ih
import battery as bt

import time as sys_time
from bridge import TelemetryBridge

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
    return (rpm / 6820.0) * 53.6 * 2

def predict_pump_rpm(speed_pct, pressure_bar):
    s = max(0.0, min(100.0, speed_pct))
    p = max(0.0, pressure_bar)
    rpm = (
        -22.7999
        + 30.7415 * s
        - 51.5842 * p
        - 0.0746 * s**2
        + 1.5701 * s * p
        + 0.7879 * p**2
        + 0.000530 * s**3
        - 0.012021 * s**2 * p
        + 0.004537 * s * p**2
        - 0.1867 * p**3
    )
    return max(0.0, rpm)

def predict_speed_pct_for_rpm(target_rpm, pressure_bar):
    r = max(0.0, target_rpm) / 1000.0
    p = max(0.0, pressure_bar)
    speed_pct = (
        0.8105
        + 32.2281 * r
        + 1.6880 * p
        + 3.3907 * r**2
        - 1.8009 * r * p
        - 0.0380 * p**2
        - 0.8592 * r**3
        + 0.4832 * r**2 * p
        + 0.0035 * r * p**2
        + 0.0057 * p**3
    )
    return max(0.0, min(100.0, speed_pct))

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
    Silencer = sl.AcousticSilencerChamber(chamber_vol_liters=0.564)  
    Hepa = hp.ZF111_HEPA_Filter()
    
    # Initialize the Battery Management System
    battery = bt.AdvancedVentilatorBMS()

    # System Constants
    temp_room_k = 273.15 + 35.0    
    comp.temp_motor_k = temp_room_k
    comp.temp_head_k = temp_room_k
    dt_s = 0.01

    tank = dt.SmartCompressorTankTwin(volume_liters=2.0, motor_voltage_v=24.0)
    t = 0.0
    last_telemetry_state = None

    # Initialize dynamic variables for t=0
    current_blower_cfm = 0.0 
    current_compressor_flow_nlpm = 0.0 
    q_nlpm_mist = 0.0          
    requested_pwm = 100.0      
    pump_control_mode = "AUTO"
    pump_command_mode = "PWM"
    manual_pump_pwm = 100.0
    manual_target_rpm = 2500.0
    blower_control_mode = "AUTO"
    manual_blower_pwm = 0.0
    sim_speed_multiplier = 1.0
    actual_system_friction_pa = 0.0
    p_tank_gauge = 0.0
    p_tank_abs_pa = 101325.0
    actual_intake_m_dot_kg_s = 0.0 
    
    # Run the BMS once at t=0 to get starting voltage and baseline data
    ac_mains_connected = True
    batter_data = battery.step(ac_connected=ac_mains_connected, compressor_draw_amps=0.0, dt=dt_s)
    current_voltage_v = batter_data["v_out"]

    # =====================================================================
    # INITIALIZE TELEMETRY BRIDGE & CONTROL STATES
    # =====================================================================
    bridge = TelemetryBridge(host="localhost", port=8765)
    bridge.start()
    
    active_mode_name = "Startup Default"
    target_flow_lpm = 48.0
    target_exp_flow_lpm = 0.0  # <--- FIXED: Added Expiration Flow variable
    target_ti_s = 0.5
    target_te_s = 0.5
    sim_is_running = True

    print("\n[SYSTEM] Digital Twin Live. Open dashboard.html to view telemetry.")
    print("[SYSTEM] Press Ctrl+C to terminate.\n")

    try:
        while True:
            loop_start_time = sys_time.perf_counter()
            
            # =====================================================================
            # 0. HANDLE INCOMING TELEMETRY COMMANDS
            # =====================================================================
            for cmd in bridge.get_commands():
                if cmd.get("type") == "set_mode":
                    active_mode_name = cmd["name"]
                    target_flow_lpm = cmd["flowrate_target_lpm"]
                    target_exp_flow_lpm = cmd.get("flowrate_exp_lpm", 0.0) # <--- FIXED: Catch from UI
                    target_ti_s = cmd["ti_s"]
                    target_te_s = cmd["te_s"]
                    print(f"UI Command -> Mode: {active_mode_name}, Insp Flow: {target_flow_lpm} LPM, Exp Flow: {target_exp_flow_lpm} LPM")
                    
                elif cmd.get("type") == "sim_control":
                    if cmd["action"] == "stop":
                        sim_is_running = False
                        requested_pwm = 0.0  
                    elif cmd["action"] == "start":
                        sim_is_running = True
                        requested_pwm = 100.0
                    # --- AC Power Toggles for Simulation ---
                    elif cmd["action"] == "ac_disconnect":
                        ac_mains_connected = False
                        print("[POWER] AC Mains Disconnected. Running on Battery!")
                    elif cmd["action"] == "ac_connect":
                        ac_mains_connected = True
                        print("[POWER] AC Mains Restored. Entering Charge logic.")

                elif cmd.get("type") == "set_pump_control":
                    pump_control_mode = cmd.get("mode", "AUTO")
                    pump_command_mode = cmd.get("command_mode", pump_command_mode)
                    manual_pump_pwm = max(0.0, min(100.0, float(cmd.get("pwm_pct", manual_pump_pwm))))
                    manual_target_rpm = max(0.0, float(cmd.get("target_rpm", manual_target_rpm)))
                    if pump_control_mode == "MANUAL":
                        if pump_command_mode == "RPM":
                            manual_pump_pwm = predict_speed_pct_for_rpm(manual_target_rpm, p_tank_gauge)
                        requested_pwm = manual_pump_pwm
                    print(f"UI Command -> Pump Control: {pump_control_mode}, {pump_command_mode}, Demand: {manual_pump_pwm:.1f}%, Target RPM: {manual_target_rpm:.0f}")

                elif cmd.get("type") == "set_blower_control":
                    blower_control_mode = cmd.get("mode", "AUTO")
                    manual_blower_pwm = max(0.0, min(100.0, float(cmd.get("pwm_pct", manual_blower_pwm))))
                    print(f"UI Command -> Blower Control: {blower_control_mode}, PWM: {manual_blower_pwm:.0f}%")

                elif cmd.get("type") == "set_sim_speed":
                    sim_speed_multiplier = max(0.1, float(cmd.get("multiplier", sim_speed_multiplier)))
                    print(f"UI Command -> Simulation Speed: {sim_speed_multiplier:.1f}x")

            if not sim_is_running:
                if last_telemetry_state is not None:
                    last_telemetry_state["sim_running"] = False
                    last_telemetry_state["control"]["sim_speed_multiplier"] = float(sim_speed_multiplier)
                    bridge.broadcast(last_telemetry_state)
                sys_time.sleep(0.05)
                continue

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

            p_comp_intake_pa, _ = Intake_hose.calculate_hose_state(
                m_dot_kg_s = actual_intake_m_dot_kg_s,
                P_in_pa = p_silencer_pa,       
                T_in_k = temp_room_k,
                T_amb_k = temp_room_k 
            )

            # =====================================================================
            # 2. THE COMPRESSOR & BATTERY INTEGRATION
            # =====================================================================
            p_comp_discharge_pa = p_tank_abs_pa + actual_system_friction_pa

            # Apply BMS Safety Derating: Converts 0-100% PWM to 0.0-1.0 multiplier 
            # and scales it back if the battery is dangerously hot or almost empty.
            if pump_control_mode == "MANUAL" and pump_command_mode == "RPM":
                manual_pump_pwm = predict_speed_pct_for_rpm(manual_target_rpm, p_tank_gauge)
            pump_command_pwm = manual_pump_pwm if pump_control_mode == "MANUAL" else requested_pwm
            safe_pwm_multiplier = (pump_command_pwm / 100.0) * batter_data["derating_multiplier"]

            (current_compressor_flow_nlpm, current_amps, temp_out_gas_k, 
             act_pwm, temp_motor_k, temp_head_k, power_w) = comp.update_system_state(
                voltage_v = current_voltage_v,
                p_up_pa = p_comp_intake_pa,          
                temp_up_k = temp_room_k, 
                p_down_pa = p_comp_discharge_pa,     
                req_pump_pwm = safe_pwm_multiplier,        
                req_blower_cfm = current_blower_cfm, 
                dt_s = dt_s
            )
            
            m_dot_kg_s = (current_compressor_flow_nlpm / 1000.0 / 60.0) * comp.rho_normal
            actual_intake_m_dot_kg_s = (0.8 * actual_intake_m_dot_kg_s) + (0.2 * m_dot_kg_s)

            # Advance the BMS simulation
            batter_data = battery.step(
                ac_connected = ac_mains_connected, 
                compressor_draw_amps = current_amps, 
                dt = dt_s
            )

            # Update system voltage for the next loop (simulating voltage sag)
            current_voltage_v = batter_data["v_out"]

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
            p_tank_gauge, tank_requested_pwm, q_pump_in, q_vent_out = tank.simulate(
                t = t,
                T_in_K= T_out_k_nrv,
                dt_s=dt_s,
                t_insp=target_ti_s,
                flow_insp=target_flow_lpm,
                t_exp=target_te_s,
                flow_exp=target_exp_flow_lpm, # <--- FIXED: Use Expiration Flow variable here
                external_inflow_nlpm= q_nlpm_mist,  
            )
            requested_pwm = manual_pump_pwm if pump_control_mode == "MANUAL" else tank_requested_pwm
            p_tank_abs_pa = (p_tank_gauge * 100000.0) + 101325.0

            # =====================================================================
            # 6. SYSTEM CONTROLLERS
            # =====================================================================
            pid_fan_pwm = pid.update(temp_head_k - 273.15, dt_s)
            fan_pwm = manual_blower_pwm if blower_control_mode == "MANUAL" else pid_fan_pwm
            fan_cfm = get_cfm_from_pwm(fan_pwm)
            current_blower_cfm = max(2 * fan_cfm, 0)  

            new_friction_pa = max(0.0, p_comp_discharge_pa - P_out_pa_mist)
            actual_system_friction_pa = (0 * actual_system_friction_pa) + (1 * new_friction_pa)

            # =====================================================================
            # 7. TELEMETRY MAPPING & BROADCAST
            # =====================================================================
            to_nlpm = (1.0 / comp.rho_normal) * 60000.0
            af_leak = max(0.0, (m_dot_kg_s - m_dot_out_kg_s_air_filter) * to_nlpm)
            ws_leak = max(0.0, (m_dot_out_kg_s_air_filter - m_dot_effective_water) * to_nlpm)
            ms_leak = max(0.0, ((m_dot_effective_water)*to_nlpm - q_nlpm_mist))

            telemetry_state = {
                "t": float(t),
                "mode": {
                    "name": active_mode_name,
                    "flowrate_target_lpm": float(target_flow_lpm),
                    "flowrate_exp_lpm": float(target_exp_flow_lpm), # <--- FIXED: Map to JSON
                    "ti_s": float(target_ti_s),
                    "te_s": float(target_te_s)
                },
                "flowrate_delivered_lpm": float(q_nlpm_mist), 
                "tank_pressure_kpa": float(p_tank_gauge), 
                "tank_pressure_bar": float(p_tank_gauge),
                "drain": {
                    "total_lpm": float(af_leak + ws_leak + ms_leak),
                    "air_filter_lpm": float(af_leak),
                    "water_separator_lpm": float(ws_leak),
                    "mist_separator_lpm": float(ms_leak)
                },
                "motor": {
                    "rpm": float(predict_pump_rpm(act_pwm * 100.0, p_tank_gauge)), 
                    "power_w": float(power_w),
                    "temp_c": float(temp_motor_k - 273.15)
                },
                "pump_head_temp_c": float(temp_head_k - 273.15),
                
                # --- Full Battery Integration for Dashboard ---
                "battery": { 
                    "voltage_v": float(batter_data["v_out"]),
                    "soc_pct": float(batter_data["soc_percent"]),
                    "temp_c": float(batter_data["temperature_c"]),
                    "status": str(batter_data["status"]),
                    "time_remaining_min": float(batter_data["time_remaining_min"]),
                    "derating_multiplier": float(batter_data["derating_multiplier"]),
                    "fan_pwm": float(batter_data["fan_pwm"])
                },
                
                "blower_pwm_pct": float(fan_pwm),
                "control": {
                    "pump_mode": pump_control_mode,
                    "pump_command_mode": pump_command_mode,
                    "pump_command_pwm_pct": float(pump_command_pwm),
                    "pump_manual_pwm_pct": float(manual_pump_pwm),
                    "pump_target_rpm": float(manual_target_rpm),
                    "blower_mode": blower_control_mode,
                    "blower_manual_pwm_pct": float(manual_blower_pwm),
                    "sim_speed_multiplier": float(sim_speed_multiplier)
                },
                "pressure_drop": {
                    "total_kpa": float((p_comp_discharge_pa - P_out_pa_mist)),
                    "hepa_kpa": float(dp_hepa_pa / 1000.0),
                    "post_pump_kpa": float((p_comp_discharge_pa - P_out_pa_hose) / 1000.0),
                    "hose_kpa": float((P_out_pa_hose - P_out_pa_coil) / 1000.0),
                    "cooling_coil_kpa": float((P_out_pa_coil - P_out_pa_filter) / 1000.0),
                    "air_filter_kpa": float(delta_p_mbar_filter / 10.0),
                    "water_separator_kpa": float(delta_p_mbar_water / 10.0),
                    "mist_separator_kpa": float(delta_p_mbar_mist / 10.0),
                    "tank_kpa": 0.0
                },
                "sim_running": sim_is_running
            }

            bridge.broadcast(telemetry_state)
            last_telemetry_state = telemetry_state
            t += dt_s
            
            # Pacing: 1x means dt_s simulation seconds per dt_s wall-clock seconds.
            target_wall_dt_s = dt_s / sim_speed_multiplier
            elapsed_wall_s = sys_time.perf_counter() - loop_start_time
            sleep_s = target_wall_dt_s - elapsed_wall_s
            if sleep_s > 0:
                sys_time.sleep(sleep_s) 

    except KeyboardInterrupt:
        print("\n[SYSTEM] Live run terminated by user.")
    
    finally:
        bridge.stop()
        print("\n[SYSTEM] Telemetry Bridge offline. Exiting.")
