import numpy as np
import matplotlib.pyplot as plt

class SmartCompressorTankTwin:
    def __init__(self, volume_liters=2.0, motor_voltage_v=24.0):
        # Thermodynamic Constants
        self.R_air = 287.05          
        self.T_k = 293.15            
        self.P_atm_pa = 101325.0
        self.rho_normal = 1.204    
        
        # Thermodynamic Specific Heats for Air
        self.cv_air = 718.0          # Specific heat at constant volume (J/kg*K)
        self.cp_air = 1005.0         # Specific heat at constant pressure (J/kg*K)
        
        # Tank State
        self.V_m3 = volume_liters / 1000.0
        self.mass_kg = (self.P_atm_pa * self.V_m3) / (self.R_air * self.T_k) 
        
        # Track the total Joules of thermal energy currently trapped inside the tank
        self.U_joules = self.mass_kg * self.cv_air * self.T_k


        
        # # Tank State
        # self.V_m3 = volume_liters / 1000.0
        # self.mass_kg = (self.P_atm_pa * self.V_m3) / (self.R_air * self.T_k) # Starts at 0 bar gauge
        self.voltage_v = motor_voltage_v
        
        # Empirical Data (G&M Tech Compressor)

        pressure_points_bar = np.array([0.14, 0.5, 1.0, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0, 4.5, 5.25, 6.25])
        flow_points_nlpm    = np.array([100.35, 95.8, 90.0, 82.5, 79.0, 76.0, 72.5, 69.5, 67.0, 64.0, 59.5, 56.0, 54.0, 49.5, 46.0])


        pressure_points_bar_i = np.array([0.14, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.25, 6.25])
        current_points_amps   = np.array([10.5, 13.2, 16.2, 18.3, 19.8, 20.7, 21.6, 22.5, 23.0, 22.8])
        # pressure_points_bar = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        # flow_points_nlpm = np.array([133, 121, 108, 90, 79, 67, 60, 52])
        # current_points_amps = np.array([9.9, 16.1, 20.8, 22.9, 24.0, 24.5, 24.9, 24.5])
        
        self.pump_curve_flow = np.polyfit(pressure_points_bar, flow_points_nlpm, 2)
        self.pump_curve_current = np.polyfit(pressure_points_bar_i, current_points_amps, 2)
        
        self.total_energy_joules = 0.0
        
        # Initialize State Machine
        self.control_state = "MAX_RECOVERY"

        # Initialize State Machine
        self.control_state = "MAX_RECOVERY"

        # --- NEW: Closed-loop feedback state for POWER_SAVING PWM ---
        self.last_actual_inflow_nlpm = 0.0   # What the FULL pneumatic chain actually delivered last step
        self.power_saving_pwm = 1.0          # Persistent PWM command while in POWER_SAVING
        self.flow_error_integral = 0.0       # Integral of (demand - actual) while in POWER_SAVING
        self.ki_flow = 0.006                # Integral gain (NLPM error -> PWM fraction). Tune if needed.
        self._prev_control_state = "MAX_RECOVERY"

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
        - Otherwise -> Power Saving (CLOSED-LOOP flow tracking)
        """
        # 1. Evaluate State Transitions
        if p_gauge_bar <= 1.5:
            self.control_state = "MAX_RECOVERY"
        elif p_gauge_bar >= 4.0:
            self.control_state = "COAST_DOWN"

        if self.control_state == "MAX_RECOVERY" and p_gauge_bar >= 2.8:
            self.control_state = "POWER_SAVING"
        elif self.control_state == "COAST_DOWN" and p_gauge_bar <= 2.8:
            self.control_state = "POWER_SAVING"

        # 2. Detect a FRESH entry into POWER_SAVING and (re)seed the controller.
        #    Anti-windup: don't let stale integral error from MAX_RECOVERY/COAST_DOWN
        #    carry into this mode.
        just_entered_power_saving = (
            self.control_state == "POWER_SAVING" and self._prev_control_state != "POWER_SAVING"
        )
        if just_entered_power_saving:
            self.flow_error_integral = 0.0
            # One-time open-loop guess so we don't start cold at 0% PWM.
            # This is ONLY a seed value now — not the ongoing controller.
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
            # --- THE FIX ---
            # OLD (broken): required_pwm = avg_demand_nlpm / ideal_curve_flow
            #   -> assumes zero suction loss, zero filter/separator drop, zero leaks
            #   -> systematically under-commands PWM since real delivered flow
            #      is always less than the bare manufacturer curve says.
            #
            # NEW: track the flow that ACTUALLY reached the tank last step
            # (self.last_actual_inflow_nlpm, updated in simulate()) and nudge
            # PWM with integral feedback until delivered flow matches demand.
            # This self-corrects for whatever the real losses happen to be,
            # instead of guessing off an idealized curve.
            flow_error_nlpm = avg_demand_nlpm - self.last_actual_inflow_nlpm
            self.flow_error_integral += flow_error_nlpm

            # Clamp the integral so a long starvation/oversupply period
            # doesn't wind up into a huge stale correction.
            max_integral = 500.0
            self.flow_error_integral = max(-max_integral, min(max_integral, self.flow_error_integral))

            correction = self.ki_flow * self.flow_error_integral
            self.power_saving_pwm = max(0.0, min(1.0, self.power_saving_pwm + correction))

            return self.power_saving_pwm

    # def calculate_controller_pwm(self, p_gauge_bar, avg_demand_nlpm):
    #     """
    #     3-State Advanced Control Logic:
    #     - Drops <= 1.5 -> Max Recovery to 2.25
    #     - Hits >= 3.0 -> Coast Down to 2.25
    #     - Otherwise -> Power Saving (Feed-forward average match)
    #     """
    #     # 1. Evaluate State Transitions
    #     if p_gauge_bar <= 1.5:
    #         self.control_state = "MAX_RECOVERY"
    #     elif p_gauge_bar >= 3.0:
    #         self.control_state = "COAST_DOWN"
            
    #     if self.control_state == "MAX_RECOVERY" and p_gauge_bar >= 2.25:
    #         self.control_state = "POWER_SAVING"
    #     elif self.control_state == "COAST_DOWN" and p_gauge_bar <= 2.25:
    #         self.control_state = "POWER_SAVING"

    #     # 2. Execute State Logic
    #     if self.control_state == "MAX_RECOVERY":
    #         return 1.0 # 100% PWM
            
    #     elif self.control_state == "COAST_DOWN":
    #         return 0.0 # 0% PWM (Pump Off)
            
    #     elif self.control_state == "POWER_SAVING":
    #         # Match the average ventilator flow demand dynamically
    #         max_available_flow, _ = self.get_pump_metrics(p_gauge_bar)
    #         if max_available_flow > 0:
    #             required_pwm = avg_demand_nlpm / max_available_flow
    #         else:
    #             required_pwm = 1.0
    #         return max(0.0, min(1.0, required_pwm))

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
    

    # def simulate(self, t, dt_s , t_insp=0.5, flow_insp=80.0, t_exp=0.5, flow_exp=80.0, external_inflow_nlpm=None):
    #     # Calculate the feed-forward target average flow for the Power Saving Mode
    #     cycle_time = t_insp + t_exp
    #     avg_demand_nlpm = ((t_insp * flow_insp) + (t_exp * flow_exp)) / cycle_time
        
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
        
    #     # ---> THE GHOST FLOW FIX <---
    #     # If the external advanced compressor gives us real flow, use it. 
    #     # Otherwise, use the simple internal curves.
    #     if external_inflow_nlpm is not None:
    #         actual_inflow_nlpm = external_inflow_nlpm
    #     else:
    #         max_flow, _ = self.get_pump_metrics(p_tank_gauge)
    #         actual_inflow_nlpm = current_pwm * max_flow

    #     # Safety: Cannot pull flow from an empty tank
    #     if p_tank_gauge <= 0.0 and actual_inflow_nlpm < q_vent_demand_nlpm:
    #         q_vent_actual = actual_inflow_nlpm
    #     else:
    #         q_vent_actual = q_vent_demand_nlpm
            
    #     # 4. Integrate System States (Conservation of Mass)
    #     m_dot_in = (actual_inflow_nlpm / 1000.0 / 60.0) * self.rho_normal
    #     m_dot_out = (q_vent_actual / 1000.0 / 60.0) * self.rho_normal
        
    #     self.mass_kg += (m_dot_in - m_dot_out) * dt_s

    #     return p_tank_gauge, current_pwm, actual_inflow_nlpm, q_vent_actual
    

    def simulate(self,t, T_in_K, dt_s, t_insp, flow_insp, t_exp, flow_exp, external_inflow_nlpm=None):
        cycle_time = t_insp + t_exp
        avg_demand_nlpm = ((t_insp * flow_insp) + (t_exp * flow_exp)) / cycle_time

        cycle_t = t % cycle_time
        if cycle_t < t_insp:
            q_vent_demand_nlpm = flow_insp
        else:
            q_vent_demand_nlpm = flow_exp
        self.T_k = T_in_K  # Update the tank temperature for this simulation step



        # Remove this line: self.T_k = T_in_K 

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

        # 2. Fix the Mass Flow Unit Mismatch (Both must be strictly kg/s)
        m_dot_in_kg_s = ( actual_inflow_nlpm/1000.0/60.0) * self.rho_normal
        m_dot_out_kg_s = (q_vent_actual / 1000.0 / 60.0) * self.rho_normal

        # 3. CONSERVATION OF ENERGY (First Law of Thermodynamics)
        # Incoming air brings Enthalpy (Flow Work + Internal Energy -> Cp)
        enthalpy_in = m_dot_in_kg_s * self.cp_air * T_in_K
        
        # Outgoing air leaves with the tank's current Enthalpy
        enthalpy_out = m_dot_out_kg_s * self.cp_air * self.T_k

        # 4. Update the Tank's Internal State
        self.U_joules += (enthalpy_in - enthalpy_out) * dt_s
        self.mass_kg += (m_dot_in_kg_s - m_dot_out_kg_s) * dt_s
        
        # 5. Calculate the true mixed temperature for the NEXT simulation step
        if self.mass_kg > 0:
            self.T_k = self.U_joules / (self.mass_kg * self.cv_air)

        self.last_actual_inflow_nlpm = actual_inflow_nlpm

        # print (current_pwm)

        # return current_pwm
        return p_tank_gauge, current_pwm*100, actual_inflow_nlpm, q_vent_actual



# if __name__ == "__main__":

    # tank = SmartCompressorTankTwin(volume_liters=2.0, motor_voltage_v=24.0)
    # total_time_s = 10.0 

    # s = tank.simulate(t=0.0, T_in_K=293.15, dt_s=0.1, t_insp=0.5, flow_insp=80.0, t_exp=0.5, flow_exp=80.0)
    # print (s)

    # pwm = tank.calculate_controller_pwm(p_gauge_bar=3.50, avg_demand_nlpm=10.0)
    # print (pwm)



        # p_tank_abs = (self.mass_kg * self.R_air * self.T_k) / self.V_m3
        # p_tank_gauge = (p_tank_abs - self.P_atm_pa) / 1e5

        # current_pwm = self.calculate_controller_pwm(p_tank_gauge, avg_demand_nlpm)

        # if external_inflow is not None:
        #     actual_inflow_nlpm = external_inflow
        # else:
        #     max_flow, _ = self.get_pump_metrics(p_tank_gauge)
        #     actual_inflow_nlpm = current_pwm * max_flow

        # if p_tank_gauge <= 0.0 and actual_inflow_nlpm < q_vent_demand_nlpm:
        #     q_vent_actual = actual_inflow_nlpm
        # else:
        #     q_vent_actual = q_vent_demand_nlpm

        # m_dot_in = actual_inflow_nlpm
        # m_dot_out = (q_vent_actual / 1000.0 / 60.0) * self.rho_normal

        # self.mass_kg += (m_dot_in - m_dot_out) * dt_s

        # # --- NEW: feed this step's REAL delivered flow back for next step's control ---
        # self.last_actual_inflow_nlpm = actual_inflow_nlpm

        # return p_tank_gauge, current_pwm, actual_inflow_nlpm, q_vent_actual
    

