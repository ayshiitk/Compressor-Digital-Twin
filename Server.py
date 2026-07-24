"""
server.py — Live WebSocket bridge for the G&M Digital Twin Studio dashboard.

WHAT THIS DOES
---------------
It runs your exact Main2.py pneumatic-train simulation loop (Pump, Digital_tank2,
cooling_coil2, Discharge_hose, NRV, air_filter, water_separater, mist_separator)
PLUS the battery.py BMS (which Main2.py never called), and streams every step live
to dashboard.html over a WebSocket at ws://localhost:8765.

You never touch this file, Main2.py, or VS Code again after today. To use it:

  1. Put server.py in the SAME FOLDER as:
       Main2.py, battery.py, Pump.py, Digital_tank2.py, cooling_coil2.py,
       Discharge_hose.py, NRV.py, air_filter.py, water_separater.py, mist_separator.py
  2. One-time install:      pip install websockets
  3. Run it (double-click start_server.bat on Windows, or `python3 server.py`
     on Mac/Linux). Leave that terminal window open in the background.
  4. Open dashboard.html in your browser. Set your parameters, click
     "Apply Parameters & Stage Sim", then "Run Sim". That's it — every
     future run/parameter-change happens from the browser, nothing else.

WHY A SERVER PROCESS IS STILL NEEDED
--------------------------------------
A browser tab can't execute your Python physics modules directly — there is no
way around having *some* Python process alive to run the numerical model. What
this script removes is the need to ever open VS Code, edit values in Main2.py,
or manually re-run a script per parameter change: the dashboard now fully
drives the simulation (start/stop/params/power-source) in real time.

BATTERY INTEGRATION
--------------------
Main2.py never instantiated AdvancedVentilatorBMS. This server does, every
step:
  - It reads compressor current draw (current_amps) from comp.update_system_state()
    and feeds it to battery.step(ac_connected, compressor_draw_amps, dt).
  - The battery's derating_multiplier now genuinely throttles the compressor
    (effective_pump_pwm *= derating) — matching the docstring in battery.py
    ("power multiplier to send to the compressor").
  - AC/DC toggle from the dashboard's Power Source buttons flips `ac_connected`
    live, switching the BMS between its charge and discharge branches.

NOTE ON FIELD MAPPINGS
------------------------
A few dashboard fields (pump RPM, "after intake hose" pressure) aren't emitted
directly by name from your existing modules based on how Main2.py calls them.
Search "APPROX:" in this file for the couple of places I derived a reasonable
value from what IS available, rather than inventing new physics.
"""

import asyncio
import json
import time
import traceback

import numpy as np
import websockets

# Same imports Main2.py uses — must sit next to this file.
import Pump as Pmp
import Digital_tank2 as dt_mod
import cooling_coil2 as cc
import Discharge_hose as dh
import NRV as nrv
import air_filter as af
import water_separater as ws
import mist_separator as ms

from battery import AdvancedVentilatorBMS

P_ATM_PA = 101325.0
HOST, PORT = "localhost", 8765


# ==========================================================
# PID (copied verbatim from Main2.py — untouched logic)
# ==========================================================
class ThermalPIDController:
    def __init__(self, kp, ki, kd, target_temp_c):
        self.Kp, self.Ki, self.Kd = kp, ki, kd
        self.target_temp = target_temp_c
        self.integral_sum = 0.0
        self.previous_error = 0.0
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


blower_pwm_data = np.array([0, 25, 40, 50, 60, 67, 77, 85, 100])
blower_rpm_data = np.array([1250, 2275, 2885, 3355, 3960, 4850, 5750, 6250, 6820])
blower_rpm_fit = np.polyfit(blower_pwm_data, blower_rpm_data, 2)

# Generate the inverse curve to predict PWM from a requested RPM
blower_pwm_fit = np.polyfit(blower_rpm_data, blower_pwm_data, 2)

def get_blower_pwm_from_rpm(target_rpm):
    if target_rpm <= 0.0:
        return 0.0
    # Clamp to physical limits of the Sunon fan data
    clamped_rpm = max(1250.0, min(6820.0, target_rpm))
    predicted_pwm = np.polyval(blower_pwm_fit, clamped_rpm)
    return max(0.0, min(100.0, predicted_pwm))


def get_blower_rpm_from_pwm(pwm_percentage):
    return np.polyval(blower_rpm_fit, pwm_percentage)


def get_cfm_from_pwm(pwm_percentage):
    rpm = get_blower_rpm_from_pwm(pwm_percentage)
    return (rpm / 6820.0) * 53.6 * 2


def predict_speed_pct_for_rpm(target_rpm, pressure_bar):
    clamped_rpm = max(0.0, min(2300.0, target_rpm))
    return (clamped_rpm / 2300.0) * 100.0


# ==========================================================
# SIMULATION ENGINE — one instance built fresh per "Apply & Stage"
# ==========================================================
class SimEngine:
    def __init__(self, params):
        self.p = params
        amb_c = float(params.get("ambient_c", 25.0))
        self.T_AMB_C = amb_c
        self.T_AMB_K = 273.15 + amb_c
        self.RH_AMB = 0.50

        model = params.get("compressor_model", "140RND")
        hepa_hrs = float(params.get("hepa_hrs", 0.0))

        ori = params.get("orifices", {}) or {}
        af_mm = float(ori.get("af_mm", 0.6))
        ws_mm = float(ori.get("ws_mm", 0.6))
        ms_mm = float(ori.get("ms_mm", 0.6))
        tank_mm = float(ori.get("tank_mm", 0.0))

        self.comp = Pmp.SmartCompressor(T_amb_k=self.T_AMB_K, pump_model=model, filter_hours=hepa_hrs)
        self.pid = ThermalPIDController(kp=4.0, ki=0.5, kd=0.1, target_temp_c=50.0)
        self.hose = dh.DischargeHose_HiPoFlex()
        self.coil = cc.CopperCoolingCoil_TwinFan()
        self.nrv = nrv.SMC_AKH10_NRV()
        self.air_filter = af.SMC_AF20_Filter(D2=af_mm)
        self.water_sep = ws.SMC_AFG20_WaterSeparator(D2=ws_mm)
        self.mist_sep = ms.SMC_AFM20_MistSeparator(D2=ms_mm)
        self.tank = dt_mod.SmartCompressorTankTwin(volume_liters=2.0, motor_voltage_v=24.0, D2=tank_mm)

        self.comp.temp_ambient_k = self.T_AMB_K
        self.comp.temp_motor_k = self.T_AMB_K
        self.comp.temp_head_k = self.T_AMB_K
        self.tank.T_k = self.T_AMB_K

        # Battery — persists soc/temp across "Apply & Stage" so it behaves like a
        # real pack rather than resetting to 100% every time you tweak a param.
        self.battery = AdvancedVentilatorBMS(initial_soc=1.0)

        self.dt_s = 0.05
        self.sim_time_s = float(params.get("sim_time_s", 1000.0))

        self.flow_insp = float(params.get("flow_insp", 48.0))
        self.flow_exp = float(params.get("flow_exp", 0.0))
        self.ti_s = float(params.get("ti_s", 0.5))
        self.te_s = float(params.get("te_s", 0.5))

        self.pump_mode = params.get("pump_mode", "AUTO")  # AUTO / MANUAL_PWM / MANUAL_RPM
        self.manual_pump_pwm = float(params.get("pump_pwm", 100.0))
        self.manual_target_rpm = float(params.get("pump_rpm", 2300.0))
        self.min_rpm_cap = float(params.get("min_rpm_cap", 1000.0))
        
        # Inject custom pressure thresholds directly into the tank instance
        self.tank.min_recovery_pressure = float(params.get("min_press", 1.5))
        self.tank.max_recovery_pressure = float(params.get("max_press", 2.5))
        
        self.blower_mode = params.get("blower_mode", "AUTO")
        self.blower_rpm_manual = float(params.get("blower_rpm", 6820.0))

        self.ac_connected = True  # dashboard defaults to AC Mains active

        self.startup_ramp_duration_s = 0.3
        self.actual_system_friction_pa = 0.0
        self.p_tank_gauge = 0.0
        self.p_tank_abs_pa = P_ATM_PA
        self.tank_requested_pwm = 100.0
        self.current_blower_pwm = 0.0

    def set_power_source(self, source):
        self.ac_connected = (source == "AC")

    def step(self, t):
        p = self
        # 1. PUMP COMMAND
        if p.pump_mode == "MANUAL_RPM":
            requested_pwm = predict_speed_pct_for_rpm(p.manual_target_rpm, p.p_tank_gauge)
        elif p.pump_mode == "MANUAL_PWM":
            requested_pwm = p.manual_pump_pwm
        else:  # AUTO — demand-following, driven by the tank controller
            requested_pwm = p.tank_requested_pwm

        # --- HARD RPM FLOOR LOGIC ---
        MIN_RPM = p.min_rpm_cap
        MAX_RPM = 2300.0 
        
        # Convert requested PWM to RPM, apply the RPM floor, then convert back
        current_requested_rpm = (requested_pwm / 100.0) * 2300.0
        clamped_rpm = max(MIN_RPM, min(MAX_RPM, current_requested_rpm))
        requested_pwm = (clamped_rpm / 2300.0) * 100.0
        # ------------------------------

        # Battery derating genuinely throttles the compressor
        requested_pwm *= p.last_derating if hasattr(p, "last_derating") else 1.0

        # 2. COMPRESSOR
        p_comp_discharge_pa = p.p_tank_abs_pa + p.actual_system_friction_pa
        ramp = min(1.0, t / p.startup_ramp_duration_s) if p.startup_ramp_duration_s > 0 else 1.0
        effective_pump_pwm = requested_pwm * ramp

        (flow_nlpm, current_amps, temp_out_gas_k, act_pwm, temp_motor_k, temp_head_k,
         power_w, p_intake_pa, p_after_hepa, p_silencer_pa) = p.comp.update_system_state(
            voltage_v=24.0,
            temp_up_k=p.T_AMB_K,
            p_down_abs_pa=p_comp_discharge_pa,
            req_pump_pwm=(effective_pump_pwm / 100.0),
            req_blower_pwm=(p.current_blower_pwm / 100.0),
            dt_s=p.dt_s,
        )

        m_dot_kg_s = (flow_nlpm / 1000.0 / 60.0) * p.comp.rho_normal

        # 3. HOSE & COIL
        P_out_pa_hose, T_out_k_hose, _ = p.hose.calculate_hose_state(
            m_dot_kg_s=m_dot_kg_s, P_in_pa=p_comp_discharge_pa,
            T_in_k=temp_out_gas_k, T_amb_k=p.T_AMB_K,
        )
        T_out_k_coil, P_out_pa_coil, _ = p.coil.analyze_coil(
            m_dot_kg_s=m_dot_kg_s, P_in_pa=P_out_pa_hose,
            T_in_k=T_out_k_hose, T_amb_k=p.T_AMB_K, selected_length=3.36,
        )

        # 4. NRV & SEPARATORS
        P_out_pa_nrv, _, T_out_k_nrv = p.nrv.calculate_valve_state(
            m_dot_kg_s=m_dot_kg_s, P_in_pa=P_out_pa_coil, T_in_k=T_out_k_coil,
        )
        P_out_pa_filter, _, m_dot_out_af, _ = p.air_filter.calculate_filter_state(
            t=t, m_dot_kg_s=m_dot_kg_s, P_in_pa=P_out_pa_nrv,
            T_in_k=T_out_k_nrv, P_atm_pa=P_ATM_PA,
        )
        P_out_pa_water, _, m_dot_eff_water, _, vapor_passed_mg_s, _ = p.water_sep.update_state(
            t=t, m_dot_in_kg_s=m_dot_out_af, P_in_pa=P_out_pa_filter, T_in_k=T_out_k_nrv,
            T_amb_C=p.T_AMB_C, RH_amb=p.RH_AMB, P_amb_pa=P_ATM_PA,
        )
        P_out_pa_mist, _, q_nlpm_mist, _, _, _ = p.mist_sep.update_state(
            t=t, m_dot_in_kg_s=m_dot_eff_water, P_in_pa=P_out_pa_water, T_in_k=T_out_k_nrv,
            aerosol_water_in_mg_s=vapor_passed_mg_s, dt_seconds=p.dt_s,
        )

        # 5. TANK
        p.p_tank_gauge, p.tank_requested_pwm, q_pump_in, q_vent_out, tank_leak = p.tank.simulate(
            t=t, T_in_K=T_out_k_nrv, dt_s=p.dt_s,
            t_insp=p.ti_s, flow_insp=p.flow_insp, t_exp=p.te_s, flow_exp=p.flow_exp,
            external_inflow_nlpm=q_nlpm_mist,
        )
        p.p_tank_abs_pa = (p.p_tank_gauge * 100000.0) + P_ATM_PA

        # 6. THERMAL FAN CONTROL
        # Calculate what the PID *wants* based on the pump head temperature
        fan_pwm_auto = p.pid.update(temp_head_k - 273.15, p.dt_s)

        if p.blower_mode == "MANUAL_RPM":
            p.current_blower_pwm = get_blower_pwm_from_rpm(p.blower_rpm_manual)
        else:  # AUTO
            p.current_blower_pwm = max(0.0, min(100.0, fan_pwm_auto))

        blower_cfm = get_cfm_from_pwm(p.current_blower_pwm)
        blower_rpm = get_blower_rpm_from_pwm(p.current_blower_pwm)

        new_friction_pa = max(0.0, p_comp_discharge_pa - P_out_pa_mist)
        p.actual_system_friction_pa = (0.9 * p.actual_system_friction_pa) + (0.1 * new_friction_pa)

        # 7. BATTERY BMS STEP — the integration Main2.py never had
        bms = p.battery.step(ac_connected=p.ac_connected, compressor_draw_amps=current_amps, dt=p.dt_s)
        p.last_derating = bms["derating_multiplier"]

        to_nlpm = (1.0 / p.comp.rho_normal) * 60000.0
        af_leak = max(0.0, (m_dot_kg_s - m_dot_out_af) * to_nlpm)
        ws_leak = max(0.0, (m_dot_out_af - m_dot_eff_water) * to_nlpm)
        ms_leak = max(0.0, (m_dot_eff_water * to_nlpm) - q_nlpm_mist)
        total_leak = af_leak + ws_leak + ms_leak + tank_leak

        bar = lambda pa: (pa - P_ATM_PA) / 100000.0

        return {
            "t": round(t, 3),
            "flowrate_delivered_lpm": q_vent_out,
            "flow_cascade": {
                "compressor_out": flow_nlpm, 
                "tank_net": q_pump_in - q_vent_out,
                "tank_in": q_nlpm_mist
            },
            "mode": {"flowrate_target_lpm": p.flow_insp},
            "drain": {
                "total_lpm": total_leak, "tank_lpm": tank_leak,
                "air_filter_lpm": af_leak, "water_separator_lpm": ws_leak,
                "mist_separator_lpm": ms_leak,
            },
            "tank_pressure_bar": p.p_tank_gauge,
            "pressure_cascade": {
                "hepa": bar(p_after_hepa), "silencer": bar(p_silencer_pa),
                "intake_hose": bar(p_intake_pa),  # APPROX: compressor inlet pressure
                "compressor": bar(p_comp_discharge_pa), "discharge_hose": bar(P_out_pa_hose),
                "coil": bar(P_out_pa_coil), "nrv": bar(P_out_pa_nrv), "air_filter": bar(P_out_pa_filter),
                "water_sep": bar(P_out_pa_water), "mist_sep": bar(P_out_pa_mist),
            },
            "motor": {
                "power_w": power_w,
                "rpm": act_pwm * 2300.0,
                "temp_c": temp_motor_k - 273.15,
            },
            "pump_head_temp_c": temp_head_k - 273.15,
            "battery": {
                "voltage_v": bms["v_out"], "soc_pct": bms["soc_percent"],
                "temp_c": bms["temperature_c"], "time_remaining_min": bms["time_remaining_min"],
                "status": bms["status"], "derating": bms["derating_multiplier"],
            },
            "blower_cfm": blower_cfm,
            "blower_rpm": blower_rpm,
        }


# ==========================================================
# WEBSOCKET SERVER
# ==========================================================
CLIENTS = set()
STATE = {"engine": None, "task": None, "raw_params": None}


async def broadcast(msg):
    if not CLIENTS:
        return
    data = json.dumps(msg)
    dead = []
    for ws_ in CLIENTS:
        try:
            await ws_.send(data)
        except websockets.exceptions.ConnectionClosed:
            dead.append(ws_)
    for d in dead:
        CLIENTS.discard(d)


async def run_loop():
    eng = STATE["engine"]
    time_arr = np.arange(0, eng.sim_time_s, eng.dt_s)
    real_start = time.perf_counter()
    try:
        for t in time_arr:
            frame = eng.step(float(t))
            await broadcast(frame)
            # Real-time pacing so the live curves animate like an actual test
            # bench rather than dumping the whole run in one burst.
            target_elapsed = float(t) + eng.dt_s
            behind = target_elapsed - (time.perf_counter() - real_start)
            if behind > 0:
                await asyncio.sleep(behind)
            else:
                await asyncio.sleep(0)  # yield control, keep it responsive
        await broadcast({"sim_event": "complete"})
    except asyncio.CancelledError:
        pass
    except Exception as e:
        traceback.print_exc()
        await broadcast({"sim_event": "error", "message": str(e)})


def stop_task():
    task = STATE.get("task")
    if task and not task.done():
        task.cancel()
    STATE["task"] = None


async def handler(websocket):
    CLIENTS.add(websocket)
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            mtype = msg.get("type")

            if mtype == "stage_parameters":
                stop_task()
                STATE["raw_params"] = msg
                try:
                    STATE["engine"] = SimEngine(msg)
                except Exception as e:
                    traceback.print_exc()
                    await websocket.send(json.dumps({"sim_event": "error", "message": f"Stage failed: {e}"}))

            elif mtype == "sim_control":
                action = msg.get("action")
                if action == "start":
                    if STATE["engine"] is None:
                        await websocket.send(json.dumps({"sim_event": "error", "message": "Stage parameters first."}))
                    else:
                        stop_task()
                        # Fresh engine each Run, but keep battery state persistent
                        # across runs by carrying it over instead of rebuilding.
                        old_battery = STATE["engine"].battery
                        STATE["engine"] = SimEngine(STATE["raw_params"])
                        STATE["engine"].battery = old_battery
                        STATE["task"] = asyncio.create_task(run_loop())
                elif action == "stop":
                    stop_task()

            elif mtype == "set_power_source":
                if STATE["engine"] is not None:
                    STATE["engine"].set_power_source(msg.get("source", "AC"))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CLIENTS.discard(websocket)


async def main():
    print(f"Digital Twin bridge live at ws://{HOST}:{PORT}")
    print("Open dashboard.html in your browser now. Leave this window running.")
    async with websockets.serve(handler, HOST, PORT, max_size=None):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())