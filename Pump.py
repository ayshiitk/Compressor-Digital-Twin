import numpy as np
import matplotlib.pyplot as plt
import time

# --- PNEUMATIC DEPENDENCIES ---
import Intake_hose as ih
import Hepa2 as hf
import Silencer as sl

class SmartCompressor:
    """
    Component Model: G&M Tech 120RND-ED / 140RND-ED Double Head Pump.
    Fully integrated with upstream tract, thermal ML, and external blower cooling.
    """
    def __init__(self, T_amb_k, pump_model="140RND", filter_hours=0):
        self.pump_model = pump_model
        self.filter_hours = filter_hours
        self.T_amb_k = T_amb_k
        
        # --- Upstream Tract Initialization ---
        self.hepa = hf.ZF111_HEPA_Filter(usage_hours=filter_hours)
        self.silencer = sl.AcousticSilencerChamber(chamber_vol_liters=0.564) 
        self.intake_hose = ih.IntakeHose_HiPoFlex() 

        # Initial inlet pressure (Standard Atmospheric, Absolute Pa)
        self.p_up_pa = 101325.0 

        # --- Pump Specs Initialization ---
        if self.pump_model == "120RND":
            self.component_name = "120RND-ED Empirical Compressor"
            self.raw_pressure_bar = np.array([0.14, 0.5, 1.0, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.25, 6.25])
            self.raw_flow_nlpm = np.array([100.35, 95.8, 90.0, 82.5, 79.0, 76.0, 72.5, 69.5, 67.0, 64.0, 59.5, 56.0, 54.0, 49.5, 46.0])
            self.raw_current_amps = np.array([10.5, 13.2, 16.2, 18.3, 19.0, 19.8, 20.3, 20.7, 21.2, 21.6, 22.2, 22.5, 22.8, 23.0, 22.8])
            self.Cth_motor = 1513.4  
            self.Cth_head = 1413.0 
            self.mechanical_friction_amps = 5.30  
            
        elif self.pump_model == "140RND":
            self.component_name = "140RND-ED Empirical Compressor"
            self.raw_pressure_bar = np.array([0.18, 0.5, 1.0, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.25, 6.25])
            self.raw_flow_nlpm = np.array([112.0, 104.0, 94.0, 85.5, 82.3, 79.3, 76.5, 73.5, 71.0, 68.4, 64.5, 60.5, 57.0, 52.0, 48.0])
            self.raw_current_amps = np.array([15.0, 16.8, 18.9, 20.6, 21.4, 22.0, 22.6, 23.1, 23.6, 24.1, 24.7, 25.1, 25.4, 25.6, 25.4])
            self.Cth_motor = 1650.0  
            self.Cth_head = 1550.0
            self.mechanical_friction_amps = 5.75  
        else:
            raise ValueError("Invalid pump model. Choose '120RND' or '140RND'.")
        
        # --- Constants & Limits ---
        self.gamma = 1.4              
        self.rho_normal = 1.204       
        self.cp_air = 1005.0          
        self.motor_limit_k = 273.15 + 75.0          
        self.motor_throttle_start_k = 273.15 + 65.0 
        
        # Set this to the spec sheet rating of your 24V cooling fan
        self.blower_max_amps = 1.2    

        # --- State Variables ---
        self.temp_ambient_k = T_amb_k
        self.temp_motor_k = 293.15
        self.temp_head_k = 293.15
        self.voltage_v = 24.0

        # Polynomial fits mapped to Gauge Pressure (Bar)
        self.flow_curve = np.polyfit(self.raw_pressure_bar, self.raw_flow_nlpm, 2)
        self.current_curve = np.polyfit(self.raw_pressure_bar, self.raw_current_amps, 3)

        self._run_empirical_calibration()

 # =========================================================================
    # THERMAL TEST DATA ENTRY
    # =========================================================================
    # "speed"  : Commanded PWM speed of the pump (0 to 100%).
    # "blower" : Commanded PWM speed of the cooling blower (0 to 100%).
    # "flow"   : Raw air flow measured by sensors in LPM.
    # "press"  : Air pressure output of the pump in Bar (Gauge).
    # "amps"   : Steady electrical current drawn by the pump ONLY.
    # "motor"  : Surface temperature of the motor in °C.
    # "rear"   : Temperature of the rear pump head in °C.
    # "front"  : Temperature of the front pump head in °C.
    # "room"   : Ambient room temperature during the test in °C.
    # =========================================================================
    THERMAL_TEST_DATA = [
        # --- REAL BENCH DATA (Extracted from 100% Blower Tests) ---
        # Speed 20
        {"speed": 20, "blower": 100, "flow": 0,  "press": 2.27, "amps": 7.21,  "motor": 50.8, "rear": 49.8, "front": 45.8, "room": 27.0},
        {"speed": 20, "blower": 100, "flow": 15, "press": 1.39, "amps": 5.65,  "motor": 44.3, "rear": 45.0, "front": 42.2, "room": 26.8},
        {"speed": 20, "blower": 100, "flow": 30, "press": 0.41, "amps": 3.31,  "motor": 39.5, "rear": 39.6, "front": 38.2, "room": 26.7},
        # Speed 25
        {"speed": 25, "blower": 100, "flow": 0,  "press": 3.21, "amps": 10.75, "motor": 53.0, "rear": 54.0, "front": 49.2, "room": 26.8},
        {"speed": 25, "blower": 100, "flow": 15, "press": 2.05, "amps": 8.69,  "motor": 50.8, "rear": 51.3, "front": 47.4, "room": 26.5},
        {"speed": 25, "blower": 100, "flow": 40, "press": 0.32, "amps": 4.15,  "motor": 40.3, "rear": 40.9, "front": 39.2, "room": 26.3},
        # Speed 30
        {"speed": 30, "blower": 100, "flow": 0,  "press": 2.98, "amps": 12.47, "motor": 57.0, "rear": 58.3, "front": 53.0, "room": 26.2},
        {"speed": 30, "blower": 100, "flow": 10, "press": 2.34, "amps": 11.01, "motor": 55.2, "rear": 56.4, "front": 51.6, "room": 26.2},
        {"speed": 30, "blower": 100, "flow": 20, "press": 1.69, "amps": 9.17,  "motor": 51.2, "rear": 53.0, "front": 48.9, "room": 26.1},
        {"speed": 30, "blower": 100, "flow": 40, "press": 0.56, "amps": 5.89,  "motor": 41.5, "rear": 43.5, "front": 41.5, "room": 26.2},
        # Speed 35
        {"speed": 35, "blower": 100, "flow": 0,  "press": 3.36, "amps": 15.88, "motor": 65.0, "rear": 65.0, "front": 58.7, "room": 26.6},
        {"speed": 35, "blower": 100, "flow": 10, "press": 2.73, "amps": 13.90, "motor": 62.9, "rear": 63.1, "front": 57.5, "room": 26.7},
        {"speed": 35, "blower": 100, "flow": 20, "press": 2.10, "amps": 11.85, "motor": 58.5, "rear": 59.7, "front": 55.0, "room": 26.6},
        {"speed": 35, "blower": 100, "flow": 50, "press": 0.38, "amps": 6.28,  "motor": 46.0, "rear": 46.8, "front": 44.4, "room": 26.5},
        # Speed 40
        {"speed": 40, "blower": 100, "flow": 10, "press": 3.03, "amps": 18.58, "motor": 67.5, "rear": 67.7, "front": 61.6, "room": 27.6},
        {"speed": 40, "blower": 100, "flow": 30, "press": 1.85, "amps": 12.59, "motor": 56.4, "rear": 60.5, "front": 56.7, "room": 27.8},
        {"speed": 40, "blower": 100, "flow": 57, "press": 0.35, "amps": 7.39,  "motor": 46.9, "rear": 49.3, "front": 47.0, "room": 27.4},
        # Speed 45
        {"speed": 45, "blower": 100, "flow": 25, "press": 2.71, "amps": 17.39, "motor": 62.2, "rear": 66.6, "front": 61.4, "room": 26.8},
        {"speed": 45, "blower": 100, "flow": 45, "press": 1.44, "amps": 11.60, "motor": 50.2, "rear": 57.4, "front": 54.0, "room": 26.9},
        {"speed": 45, "blower": 100, "flow": 64, "press": 0.35, "amps": 8.04,  "motor": 47.2, "rear": 50.2, "front": 48.4, "room": 26.9},
        
        # Speed 50 
        {"speed": 50, "blower": 100, "flow": 20, "press": 3.80, "amps": 19.50, "motor": 72.0, "rear": 75.0, "front": 69.0, "room": 27.0},
        {"speed": 50, "blower": 100, "flow": 40, "press": 2.50, "amps": 15.20, "motor": 64.0, "rear": 67.0, "front": 62.0, "room": 27.0},
        {"speed": 50, "blower": 100, "flow": 70, "press": 0.60, "amps": 10.50, "motor": 55.0, "rear": 58.0, "front": 53.0, "room": 27.0},

        # Speed 60
        {"speed": 60, "blower": 100, "flow": 25, "press": 4.50, "amps": 21.00, "motor": 80.0, "rear": 83.0, "front": 75.0, "room": 27.0},
        {"speed": 60, "blower": 100, "flow": 50, "press": 3.00, "amps": 16.50, "motor": 70.0, "rear": 73.0, "front": 66.0, "room": 27.0},
        {"speed": 60, "blower": 100, "flow": 85, "press": 0.80, "amps": 12.00, "motor": 60.0, "rear": 63.0, "front": 57.0, "room": 27.0},

        # Speed 80 
        {"speed": 80, "blower": 100, "flow": 30, "press": 5.50, "amps": 24.50, "motor": 95.0, "rear": 98.0, "front": 89.0, "room": 27.0},
        {"speed": 80, "blower": 100, "flow": 60, "press": 3.50, "amps": 19.20, "motor": 83.0, "rear": 87.0, "front": 79.0, "room": 27.0},
        {"speed": 80, "blower": 100, "flow": 100,"press": 1.20, "amps": 14.50, "motor": 72.0, "rear": 76.0, "front": 69.0, "room": 27.0},

        # Speed 100
        {"speed": 100, "blower": 100, "flow": 35, "press": 6.20, "amps": 27.00, "motor": 112.0, "rear": 116.0, "front": 105.0, "room": 27.0},
        {"speed": 100, "blower": 100, "flow": 70, "press": 4.20, "amps": 21.80, "motor": 98.0,  "rear": 103.0, "front": 93.0,  "room": 27.0},
        {"speed": 100, "blower": 100, "flow": 110,"press": 1.50, "amps": 17.00, "motor": 85.0,  "rear": 90.0,  "front": 81.0,  "room": 27.0},
    ]

  
    _MOTOR_FEATURE_CANDIDATES = [["speed", "blower"], ["speed", "press", "blower"], ["speed", "flow", "blower"], ["speed", "press", "flow", "blower"]]
    _HEAD_FEATURE_CANDIDATES = [["speed", "room", "blower"], ["speed", "press", "room", "blower"], ["speed", "flow", "room", "blower"], ["speed", "press", "flow", "room", "blower"]]
    _MOTOR_TEMP_FEATURE_CANDIDATES = _HEAD_FEATURE_CANDIDATES

    @staticmethod
    def _build_design_matrix(rows, feature_names):
        cols = [np.ones(len(rows))]
        for name in feature_names:
            cols.append(np.array([float(r[name]) for r in rows]))
        return np.column_stack(cols)

    @staticmethod
    def _loocv_rmse(X, y):
        n = len(y)
        if n <= X.shape[1]: return np.inf
        sq_errors = []
        for i in range(n):
            mask = np.ones(n, dtype=bool)
            mask[i] = False
            try:
                coeffs, _, _, _ = np.linalg.lstsq(X[mask], y[mask], rcond=None)
            except np.linalg.LinAlgError:
                return np.inf
            pred = X[i] @ coeffs
            sq_errors.append((pred - y[i]) ** 2)
        return float(np.sqrt(np.mean(sq_errors)))

    def _select_best_model(self, rows, y, candidates, label):
        results = []
        for feats in candidates:
            X = self._build_design_matrix(rows, feats)
            rmse = self._loocv_rmse(X, y)
            results.append((rmse, feats, X))
        results.sort(key=lambda r: r[0])
        print(f"[{label}] ML Selection (LOOCV RMSE):")
        for rmse, feats, _ in results:
            rmse_str = f"{rmse:.3f}" if np.isfinite(rmse) else "insufficient data"
            print(f"    {'+'.join(feats):<28s} -> {rmse_str}")
        
        best_rmse, best_feats, best_X = results[0]
        coeffs, _, _, _ = np.linalg.lstsq(best_X, y, rcond=None)
        print(f"    -> Selected Model: {'+'.join(best_feats)}\n")
        return best_feats, coeffs

    def _fit_thermal_model_from_table(self):
        rows = self.THERMAL_TEST_DATA
        if len(rows) < 4:
            print("[Warning] Need at least 4 stable data points to fit models.")
            return

        cur_a = np.array([r["amps"] for r in rows])
        motor_c = np.array([r["motor"] for r in rows])
        room_c = np.array([r["room"] for r in rows])
        
        heat_motor_w = (cur_a ** 2) * self.R_coil_hot_ohms
        dT_motor = np.maximum(motor_c - room_c, 0.5)
        G_motor_measured = heat_motor_w / dT_motor
        
        print("\n--- Running Thermal Calibration ---")
        self.motor_features, self.G_motor_coeffs = self._select_best_model(rows, G_motor_measured, self._MOTOR_FEATURE_CANDIDATES, "Motor Conductance")
        
        rear_c = np.array([r["rear"] for r in rows])
        front_c = np.array([r["front"] for r in rows])
        self.head_rear_features, self.T_head_rear_coeffs = self._select_best_model(rows, rear_c, self._HEAD_FEATURE_CANDIDATES, "Head Rear Temp")
        self.head_front_features, self.T_head_front_coeffs = self._select_best_model(rows, front_c, self._HEAD_FEATURE_CANDIDATES, "Head Front Temp")
        self.motor_temp_features, self.T_motor_coeffs = self._select_best_model(rows, motor_c, self._MOTOR_TEMP_FEATURE_CANDIDATES, "Motor Temp (Direct)")

    def _run_empirical_calibration(self):
        self.R_coil_hot_ohms = 0.45
        self._fit_thermal_model_from_table()
        self.tau_head_s = 900.0  
        self.epsilon_head = 0.65 

    def _eval_features(self, feature_names, coeffs, speed, press=None, flow=None, room=None, blower=None):
        values = {"speed": speed, "press": press, "flow": flow, "room": room, "blower": blower}
        total = coeffs[0]
        for name, coeff in zip(feature_names, coeffs[1:]):
            total += coeff * values[name]
        return total

    def _motor_conductance_w_per_k(self, speed, press, flow, room, blower):
        G = self._eval_features(self.motor_features, self.G_motor_coeffs, speed, press, flow, room, blower)
        return max(0.05, G)

    def _head_ss_temps_c(self, speed, press, flow, room, blower):
        t_rear = self._eval_features(self.head_rear_features, self.T_head_rear_coeffs, speed, press, flow, room, blower)
        t_front = self._eval_features(self.head_front_features, self.T_head_front_coeffs, speed, press, flow, room, blower)
        return t_rear, t_front

    def _apply_thermal_safety_throttle(self, requested_pwm):
        if self.temp_motor_k <= self.motor_throttle_start_k: return requested_pwm
        if self.temp_motor_k >= self.motor_limit_k: return 0.0 
        
        temp_margin = self.motor_limit_k - self.motor_throttle_start_k
        temp_excess = self.temp_motor_k - self.motor_throttle_start_k
        return requested_pwm * (1.0 - (temp_excess / temp_margin))

    def update_system_state(self, voltage_v, temp_up_k, p_down_abs_pa, req_pump_pwm, req_blower_pwm, dt_s):
        self.voltage_v = voltage_v
        req_pump_pwm = max(0.0, min(1.0, req_pump_pwm))
        req_blower_pwm = max(0.0, min(1.0, req_blower_pwm))
        actual_pwm = self._apply_thermal_safety_throttle(req_pump_pwm)

        tuning_factor = 1.15  # Adjust this factor to tune the controller's aggressiveness
        
        # 1. PNEUMATIC INTEGRATION (PUMP)
        delta_p_bar = (p_down_abs_pa - self.p_up_pa) / 100000.0
        density_ratio = 1.0

        if delta_p_bar > 6.5 or actual_pwm == 0.0:
            mass_flow_kg_s, flow_nlpm, current_amps = 0.0, 0.0, 0.0
        else:
            base_flow_nlpm = max(0.0, np.polyval(self.flow_curve, delta_p_bar))*tuning_factor
            flow_nlpm = base_flow_nlpm * density_ratio * actual_pwm
            mass_flow_kg_s = ((flow_nlpm / 1000.0) / 60.0) * self.rho_normal
            
            base_current = max(0.0, np.polyval(self.current_curve, delta_p_bar))
            total_pneumatic_current = max(0.0, base_current - self.mechanical_friction_amps)
            current_amps = (self.mechanical_friction_amps + (total_pneumatic_current * density_ratio)) * actual_pwm

        # 2. UPSTREAM COUPLING: Silencer & HEPA
        inlet_flow_guess_nlpm = abs((self.silencer.m_dot_in / self.rho_normal) * 60.0 * 1000.0)
        
        for _ in range(6):
            dp_hepa_probe = self.hepa.pressure_drop(inlet_flow_guess_nlpm)
            p_after_hepa_probe = 101325.0 - dp_hepa_probe
            
            m_dot_in_probe, _, _ = self.silencer._compute_step(
                self.silencer.m_dot_in, self.silencer.mass_kg, self.silencer.pressure_pa,
                p_after_hepa_probe, mass_flow_kg_s, dt_s
            )
            inlet_flow_guess_nlpm = abs((m_dot_in_probe / self.rho_normal) * 60.0 * 1000.0)

        inlet_flow_nlpm = inlet_flow_guess_nlpm
        dp_hepa = self.hepa.pressure_drop(inlet_flow_nlpm)
        p_after_hepa = 101325.0 - dp_hepa

        p_silencer_pa, _ = self.silencer.update_state(p_after_hepa, mass_flow_kg_s, dt_s)

        # 3. HOSE INTEGRATION
        raw_p_up_pa, hose_temp_k, heat_loss_w = self.intake_hose.calculate_hose_state(
            m_dot_kg_s=mass_flow_kg_s, 
            P_in_pa=p_silencer_pa, 
            T_in_k=temp_up_k, 
            T_amb_k=self.temp_ambient_k
        )
        
        relax_beta = 0.9  
        self.p_up_pa = (relax_beta * self.p_up_pa) + ((1.0 - relax_beta) * raw_p_up_pa)

        # 4. THERMODYNAMICS & ELECTRICAL
        blower_current = req_blower_pwm * self.blower_max_amps
        
        if actual_pwm == 0.0:
            heat_motor_w = 0.0
            total_electrical_power_w = self.voltage_v * blower_current
            temp_out_gas_k = temp_up_k
        else:
            heat_motor_w = (current_amps ** 2) * self.R_coil_hot_ohms
            pressure_ratio = p_down_abs_pa / max(1.0, self.p_up_pa)
            
            temp_peak_k = hose_temp_k * (pressure_ratio ** ((self.gamma - 1.0) / self.gamma)) if pressure_ratio > 1.0 else hose_temp_k
            temp_out_gas_k = temp_peak_k - self.epsilon_head * (temp_peak_k - self.temp_head_k)
            
            # Combine pump and blower power draw
            total_electrical_power_w = self.voltage_v * (current_amps + blower_current)

        # 5. TRANSIENT TEMPERATURE UPDATE
        speed_pct = actual_pwm * 100.0
        blower_pct = req_blower_pwm * 100.0
        pressure_bar_thermal = max(0.0, delta_p_bar)
        t_room_c = self.temp_ambient_k - 273.15

        # Update Motor Temp
        G_motor = self._motor_conductance_w_per_k(speed_pct, pressure_bar_thermal, flow_nlpm, t_room_c, blower_pct)
        cooling_motor_w = G_motor * (self.temp_motor_k - self.temp_ambient_k)
        self.temp_motor_k += ((heat_motor_w - cooling_motor_w) / self.Cth_motor) * dt_s

        # Update Head Temp
        t_head_rear_ss_c, t_head_front_ss_c = self._head_ss_temps_c(speed_pct, pressure_bar_thermal, flow_nlpm, t_room_c, blower_pct)
        t_head_ss_k = ((t_head_rear_ss_c + t_head_front_ss_c) / 2.0) + 273.15
        self.temp_head_k += ((t_head_ss_k - self.temp_head_k) / self.tau_head_s) * dt_s

        return flow_nlpm, current_amps, temp_out_gas_k, actual_pwm, self.temp_motor_k, self.temp_head_k, total_electrical_power_w, self.p_up_pa, p_after_hepa, p_silencer_pa

# =========================================================================
# STANDALONE EXECUTION / TEST DRIVER
# =========================================================================
if __name__ == "__main__":
    print("\nBooting Simulation Environment...")
    pump = SmartCompressor(pump_model="140RND", filter_hours=500)
    
    dt_s = 0.1             
    duration_s = 600.0     
    steps = int(duration_s / dt_s)
    
    req_pump_pwm = 0.25
    req_blower_pwm = 1.0  # Simulating blower running at 100% capacity          
    p_down_abs_pa = 101325.0 + 150000.0  
    
    time_log, flow_log, motor_temp_log, head_temp_log = [], [], [], []

    print(f"\nSimulating {duration_s}s runtime at {req_pump_pwm*100}% Pump PWM and {req_blower_pwm*100}% Blower PWM...")
    
    for step in range(steps):
        t = step * dt_s
        flow_nlpm, current_amps, t_gas, act_pwm, t_motor, t_head, pwr, p_up, p_hepa, p_silencer = pump.update_system_state(
            voltage_v=24.0, 
            temp_up_k=298.15, 
            p_down_abs_pa=p_down_abs_pa,
            req_pump_pwm=req_pump_pwm,
            req_blower_pwm=req_blower_pwm,
            dt_s=dt_s
        )
        
        if step % 10 == 0:
            time_log.append(t)
            flow_log.append(flow_nlpm)
            motor_temp_log.append(t_motor - 273.15)
            head_temp_log.append(t_head - 273.15)

    print("Simulation Complete. Plotting data...")
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax1.plot(time_log, motor_temp_log, label="Motor Temp (°C)", color='red', linewidth=2)
    ax1.plot(time_log, head_temp_log, label="Head Temp (°C)", color='darkorange', linewidth=2)
    ax1.set_title("Compressor Thermal Rise")
    ax1.set_ylabel("Temperature (°C)")
    ax1.legend()
    ax1.grid(True)
    
    ax2.plot(time_log, flow_log, label="Air Flow (NLPM)", color='blue', linewidth=2)
    ax2.set_title("Pneumatic Output")
    ax2.set_ylabel("Flow Rate (NLPM)")
    ax2.set_xlabel("Time (Seconds)")
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()