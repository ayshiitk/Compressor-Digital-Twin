import numpy as np
import matplotlib.pyplot as plt

class SmartCompressorTankTwin:
    def __init__(self, volume_liters=2.0, motor_voltage_v=24.0):
        # Thermodynamic Constants
        self.R_air = 287.05          
        self.T_k = 293.15            
        self.P_atm_pa = 101325.0
        self.rho_normal = 1.204      
        
        # Tank State
        self.V_m3 = volume_liters / 1000.0
        self.mass_kg = (self.P_atm_pa * self.V_m3) / (self.R_air * self.T_k) # Starts at 0 bar gauge
        self.voltage_v = motor_voltage_v
        
        # Empirical Data (G&M Tech Compressor)
        pressure_points_bar = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        flow_points_nlpm = np.array([133, 121, 108, 90, 79, 67, 60, 52])
        current_points_amps = np.array([9.9, 16.1, 20.8, 22.9, 24.0, 24.5, 24.9, 24.5])
        
        self.pump_curve_flow = np.polyfit(pressure_points_bar, flow_points_nlpm, 2)
        self.pump_curve_current = np.polyfit(pressure_points_bar, current_points_amps, 2)
        
        self.total_energy_joules = 0.0
        
        # Initialize State Machine
        self.control_state = "MAX_RECOVERY"

    def get_pump_metrics(self, p_gauge_bar):
        p_lookup = max(0.0, min(8.0, p_gauge_bar))
        flow_nlpm = max(0.0, float(np.polyval(self.pump_curve_flow, p_lookup)))
        current_amps = max(0.0, float(np.polyval(self.pump_curve_current, p_lookup)))
        return flow_nlpm, current_amps

    def calculate_controller_pwm(self, p_gauge_bar, avg_demand_nlpm):
        """
        3-State Advanced Control Logic:
        - Drops <= 1.5 -> Max Recovery to 2.25
        - Hits >= 3.0 -> Coast Down to 2.25
        - Otherwise -> Power Saving (Feed-forward average match)
        """
        # 1. Evaluate State Transitions
        if p_gauge_bar <= 1.5:
            self.control_state = "MAX_RECOVERY"
        elif p_gauge_bar >= 3.0:
            self.control_state = "COAST_DOWN"
            
        if self.control_state == "MAX_RECOVERY" and p_gauge_bar >= 2.25:
            self.control_state = "POWER_SAVING"
        elif self.control_state == "COAST_DOWN" and p_gauge_bar <= 2.25:
            self.control_state = "POWER_SAVING"

        # 2. Execute State Logic
        if self.control_state == "MAX_RECOVERY":
            return 1.0 # 100% PWM
            
        elif self.control_state == "COAST_DOWN":
            return 0.0 # 0% PWM (Pump Off)
            
        elif self.control_state == "POWER_SAVING":
            # Match the average ventilator flow demand dynamically
            max_available_flow, _ = self.get_pump_metrics(p_gauge_bar)
            if max_available_flow > 0:
                required_pwm = avg_demand_nlpm / max_available_flow
            else:
                required_pwm = 1.0
            return max(0.0, min(1.0, required_pwm))

    # def simulate(self, t, dt_s , t_insp=0.5, flow_insp=80.0, t_exp=0.5, flow_exp=80.0):
    #     # time = np.arange(0, total_time_s, dt_s)
        
    #     # Calculate the feed-forward target average flow for the Power Saving Mode
    #     cycle_time = t_insp + t_exp
    #     avg_demand_nlpm = ((t_insp * flow_insp) + (t_exp * flow_exp)) / cycle_time
        
    #     # Data Loggers
    #     # hist_p_tank, hist_pwm, hist_q_pump, hist_q_vent = [], [], [], []
    #     # hist_power, hist_energy, hist_amps = [], [], []
        
    #     # for t in time:
    #     # 1. Resolve Instantaneous Ventilator Demand
    #     cycle_t = t % cycle_time
    #     if cycle_t < t_insp:
    #         q_vent_demand_nlpm = flow_insp
    #     else:
    #         q_vent_demand_nlpm = flow_exp

    #     # 2. Read Pneumatic State
    #     p_tank_abs = (self.mass_kg * self.R_air * self.T_k) / self.V_m3
    #     p_tank_gauge = (p_tank_abs - self.P_atm_pa) / 1e5
        
    #     # 3. Control Logic & Motor State
    #     current_pwm = self.calculate_controller_pwm(p_tank_gauge, avg_demand_nlpm)
    #     max_flow, max_amps = self.get_pump_metrics(p_tank_gauge)
        
    #     q_pump_nlpm = current_pwm * max_flow
    #     actual_amps = current_pwm * max_amps  
    #     power_w = actual_amps * self.voltage_v
        
    #     # Safety: Cannot pull flow from an empty tank
    #     if p_tank_gauge <= 0.0 and q_pump_nlpm < q_vent_demand_nlpm:
    #         q_vent_actual = q_pump_nlpm
    #     else:
    #         q_vent_actual = q_vent_demand_nlpm
            
    #     # 4. Integrate System States (Conservation of Mass and Energy)
    #     m_dot_in = (q_pump_nlpm / 1000.0 / 60.0) * self.rho_normal
    #     m_dot_out = (q_vent_actual / 1000.0 / 60.0) * self.rho_normal
        
    #     self.mass_kg += (m_dot_in - m_dot_out) * dt_s
    #     self.total_energy_joules += power_w * dt_s

    #     return p_tank_gauge, current_pwm, q_pump_nlpm, q_vent_actual, power_w, actual_amps
    

    def simulate(self, t, dt_s , t_insp=0.5, flow_insp=80.0, t_exp=0.5, flow_exp=80.0, external_inflow_nlpm=None):
        # Calculate the feed-forward target average flow for the Power Saving Mode
        cycle_time = t_insp + t_exp
        avg_demand_nlpm = ((t_insp * flow_insp) + (t_exp * flow_exp)) / cycle_time
        
        # 1. Resolve Instantaneous Ventilator Demand
        cycle_t = t % cycle_time
        if cycle_t < t_insp:
            q_vent_demand_nlpm = flow_insp
        else:
            q_vent_demand_nlpm = flow_exp

        # 2. Read Pneumatic State
        p_tank_abs = (self.mass_kg * self.R_air * self.T_k) / self.V_m3
        p_tank_gauge = (p_tank_abs - self.P_atm_pa) / 1e5
        
        # 3. Control Logic & Motor State
        current_pwm = self.calculate_controller_pwm(p_tank_gauge, avg_demand_nlpm)
        
        # ---> THE GHOST FLOW FIX <---
        # If the external advanced compressor gives us real flow, use it. 
        # Otherwise, use the simple internal curves.
        if external_inflow_nlpm is not None:
            actual_inflow_nlpm = external_inflow_nlpm
        else:
            max_flow, _ = self.get_pump_metrics(p_tank_gauge)
            actual_inflow_nlpm = current_pwm * max_flow

        # Safety: Cannot pull flow from an empty tank
        if p_tank_gauge <= 0.0 and actual_inflow_nlpm < q_vent_demand_nlpm:
            q_vent_actual = actual_inflow_nlpm
        else:
            q_vent_actual = q_vent_demand_nlpm
            
        # 4. Integrate System States (Conservation of Mass)
        m_dot_in = (actual_inflow_nlpm / 1000.0 / 60.0) * self.rho_normal
        m_dot_out = (q_vent_actual / 1000.0 / 60.0) * self.rho_normal
        
        self.mass_kg += (m_dot_in - m_dot_out) * dt_s

        return p_tank_gauge, current_pwm, actual_inflow_nlpm, q_vent_actual
    

