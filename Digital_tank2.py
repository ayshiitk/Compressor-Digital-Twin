import numpy as np
import matplotlib.pyplot as plt
import math
import Drain_leak_calc as dlc

class SmartCompressorTankTwin:
    def __init__(self, volume_liters=2.0, motor_voltage_v=24.0, D2=1.0):
        # Thermodynamic Constants
        self.R_air = 287.05          
        self.T_k = 273.15 + 30.0  # Initial tank temperature (K)            
        self.P_atm_pa = 101325.0
        self.rho_normal = 1.204    

        self.Drain_leak = dlc.RestrictorFlowSolver()

        self.drain_dia = D2  # Drain hole diameter (mm)
        
        # Thermodynamic Specific Heats for Air
        self.cv_air = 718.0          # Specific heat at constant volume (J/kg*K)
        self.cp_air = 1005.0         # Specific heat at constant pressure (J/kg*K)
        
        # Tank State
        self.V_m3 = volume_liters / 1000.0
        self.mass_kg = (self.P_atm_pa * self.V_m3) / (self.R_air * self.T_k) 
        
        # Track the total Joules of thermal energy currently trapped inside the tank
        self.U_joules = self.mass_kg * self.cv_air * self.T_k

        self.voltage_v = motor_voltage_v
        
        # Empirical Data (G&M Tech Compressor)
        pressure_points_bar = np.array([0.14, 0.5, 1.0, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.25, 6.25])
        flow_points_nlpm    = np.array([100.35, 95.8, 90.0, 82.5, 79.0, 76.0, 72.5, 69.5, 67.0, 64.0, 59.5, 56.0, 54.0, 49.5, 46.0])

        pressure_points_bar_i = np.array([0.14, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.25, 6.25])
        current_points_amps   = np.array([10.5, 13.2, 16.2, 18.3, 19.8, 20.7, 21.6, 22.5, 23.0, 22.8])
        
        self.pump_curve_flow = np.polyfit(pressure_points_bar, flow_points_nlpm, 2)
        self.pump_curve_current = np.polyfit(pressure_points_bar_i, current_points_amps, 2)
        
        self.total_energy_joules = 0.0

        # Initialize State Machine
        self.control_state = "MAX_RECOVERY"

        # Closed-loop feedback state for POWER_SAVING PWM
        self.last_actual_inflow_nlpm = 0.0   # What the FULL pneumatic chain actually delivered last step
        self.power_saving_pwm = 1.0          # Persistent PWM command while in POWER_SAVING
        self.flow_error_integral = 0.0       # Integral of (demand - actual) while in POWER_SAVING
        self.ki_flow = 0.006                 # Integral gain (NLPM error -> PWM fraction). Tune if needed.
        self._prev_control_state = "MAX_RECOVERY"
        
        # Default thresholds (overridden by server.py / dashboard)
        self.min_recovery_pressure = 1.5
        self.max_recovery_pressure = 2.5

    def get_pump_metrics(self, p_gauge_bar):
        p_lookup = max(0.0, min(6.5, p_gauge_bar))
        flow_nlpm = max(0.0, float(np.polyval(self.pump_curve_flow, p_lookup)))
        current_amps = max(0.0, float(np.polyval(self.pump_curve_current, p_lookup)))
        return flow_nlpm, current_amps
    
    def calculate_controller_pwm(self, p_gauge_bar, avg_demand_nlpm):
        """
        3-State Advanced Control Logic:
        - Drops <= Min Recovery -> Max Recovery
        - Hits >= 4.0 -> Coast Down
        - Otherwise -> Power Saving (CLOSED-LOOP flow tracking)
        """
        # 1. Evaluate State Transitions
        if p_gauge_bar <= self.min_recovery_pressure:
            self.control_state = "MAX_RECOVERY"
        elif p_gauge_bar >= 4.0:
            self.control_state = "COAST_DOWN"

        if self.control_state == "MAX_RECOVERY" and p_gauge_bar >= self.max_recovery_pressure:
            self.control_state = "POWER_SAVING"
        elif self.control_state == "COAST_DOWN" and p_gauge_bar <= self.max_recovery_pressure:
            self.control_state = "POWER_SAVING"

        # 2. Detect a FRESH entry into POWER_SAVING and (re)seed the controller.
        just_entered_power_saving = (
            self.control_state == "POWER_SAVING" and self._prev_control_state != "POWER_SAVING"
        )
        if just_entered_power_saving:
            self.flow_error_integral = 0.0
            ideal_max_flow, _ = self.get_pump_metrics(p_gauge_bar)
            self.power_saving_pwm = (
                max(0.0, min(1.0, avg_demand_nlpm / ideal_max_flow)) if ideal_max_flow > 0 else 1.0
            )

        self._prev_control_state = self.control_state

        # 3. Execute State Logic
        if self.control_state == "MAX_RECOVERY":
            return 1.0  # 100% PWM

        elif self.control_state == "COAST_DOWN":
            return 0.0  # 0% PWM (Pump Off)

        elif self.control_state == "POWER_SAVING":
            flow_error_nlpm = avg_demand_nlpm - self.last_actual_inflow_nlpm
            self.flow_error_integral += flow_error_nlpm

            max_integral = 500.0
            self.flow_error_integral = max(-max_integral, min(max_integral, self.flow_error_integral))

            correction = self.ki_flow * self.flow_error_integral
            self.power_saving_pwm = max(0.0, min(1.0, self.power_saving_pwm + correction))

            return self.power_saving_pwm

    def simulate(self, t, T_in_K, dt_s, t_insp, flow_insp, t_exp, flow_exp, external_inflow_nlpm=None):
        cycle_time = t_insp + t_exp
        avg_demand_nlpm = ((t_insp * flow_insp) + (t_exp * flow_exp)) / cycle_time

        

        cycle_t = t % cycle_time
        if cycle_t < t_insp:
            q_vent_demand_nlpm = flow_insp
        else:
            q_vent_demand_nlpm = flow_exp

        # 1. Evaluate Pressure based on CURRENT mass and temperature
        p_tank_abs = (self.mass_kg * self.R_air * self.T_k) / self.V_m3
        p_tank_gauge = (p_tank_abs - self.P_atm_pa) / 100000.0

        current_pwm = self.calculate_controller_pwm(p_tank_gauge, avg_demand_nlpm)

        if external_inflow_nlpm is not None:
            actual_inflow_nlpm  = external_inflow_nlpm
        else:
            max_flow, _ = self.get_pump_metrics(p_tank_gauge)
            actual_inflow_nlpm = current_pwm * max_flow

        if p_tank_gauge <= 0.0 and actual_inflow_nlpm < q_vent_demand_nlpm:
            q_vent_actual = actual_inflow_nlpm
        else:
            q_vent_actual = q_vent_demand_nlpm

        # --- NEW: Calculate 1mm Drain Leakage ---
        m_dot_leak_kg_s, leak_nlpm = self.Drain_leak.calculate_flow(self.drain_dia, p_tank_abs, T_in_K)

        # 2. Fix the Mass Flow Unit Mismatch (Both must be strictly kg/s)
        m_dot_in_kg_s = (actual_inflow_nlpm / 1000.0 / 60.0) * self.rho_normal
        # m_dot_in_kg_s = (actual_inflow_nlpm)
        m_dot_out_kg_s = (q_vent_actual / 1000.0 / 60.0) * self.rho_normal

        # 3. CONSERVATION OF ENERGY (First Law of Thermodynamics)
        enthalpy_in = m_dot_in_kg_s * self.cp_air * T_in_K
        enthalpy_out = m_dot_out_kg_s * self.cp_air * self.T_k
        enthalpy_leak = m_dot_leak_kg_s * self.cp_air * self.T_k

        # 4. Update the Tank's Internal State
        self.U_joules += (enthalpy_in - enthalpy_out - enthalpy_leak) * dt_s
        self.mass_kg += (m_dot_in_kg_s - m_dot_out_kg_s - m_dot_leak_kg_s) * dt_s
        
        # 5. Calculate the true mixed temperature for the NEXT simulation step
        if self.mass_kg > 0:
            self.T_k = self.U_joules / (self.mass_kg * self.cv_air)

        self.last_actual_inflow_nlpm = actual_inflow_nlpm
        X= m_dot_in_kg_s-m_dot_out_kg_s-m_dot_leak_kg_s
        # print(f"X: {X}, mass_kg: {self.mass_kg}")
        # print(f"Flow_in: {flow:.2f} NLPM, Flow_out: {q_vent_actual:.2f} NLPM, Flow_leak: {leak_nlpm:.2f} NLPM")
        # print(f"m_dot_in_kg_s: {m_dot_in_kg_s:.6f}, m_dot_out_kg_s: {m_dot_out_kg_s:.6f}, m_dot_leak_kg_s: {m_dot_leak_kg_s:.6f}, mass_kg: {self.mass_kg:.4f}")
        return p_tank_gauge, current_pwm * 100.0, actual_inflow_nlpm, q_vent_actual, leak_nlpm

if __name__ == "__main__":
    tank = SmartCompressorTankTwin(volume_liters=2.0, motor_voltage_v=24.0)
    
    # Run a single step to verify the new returns
    p, pwm, q_in, q_out, q_leak = tank.simulate(
        t=0.0, 
        T_in_K=293.15, 
        dt_s=0.01, 
        t_insp=0.5, 
        flow_insp=80.0, 
        t_exp=0.5, 
        flow_exp=80.0
    )
    
    print("Initial Simulation Step Results:")
    print(f"Tank Pressure : {p:.3f} Bar (Gauge)")
    print(f"Motor PWM     : {pwm:.1f}%")
    print(f"Inflow        : {q_in:.1f} NLPM")
    print(f"Vent Outflow  : {q_out:.1f} NLPM")
    print(f"Parasitic Leak: {q_leak:.2f} NLPM")