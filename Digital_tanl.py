import numpy as np
import matplotlib.pyplot as plt

class CompressorTankDigitalTwin:
    def __init__(self, volume_liters=2.0, motor_voltage_v=24.0):
        # Thermodynamic Constants
        self.R_air = 287.05          # J/(kg*K)
        self.T_k = 293.15            # 20 C Isothermal assumption
        self.P_atm_pa = 101325.0
        self.rho_normal = 1.204      # kg/m^3 (Density at standard conditions)
        
        # Tank State
        self.V_m3 = volume_liters / 1000.0
        self.mass_kg = (self.P_atm_pa * self.V_m3) / (self.R_air * self.T_k) # Starts empty (0 bar gauge)
        
        # Electrical Specs
        self.voltage_v = motor_voltage_v
        
        # --- NEW EMPIRICAL DATA ARRAYS ---
        pressure_points_bar = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        flow_points_nlpm = np.array([133, 121, 108, 90, 79, 67, 60, 52])
        current_points_amps = np.array([9.9, 16.1, 20.8, 22.9, 24.0, 24.5, 24.9, 24.5])
        
        # Curve Fitting (2nd-Order Polynomials for smooth interpolation)
        self.pump_curve_flow = np.polyfit(pressure_points_bar, flow_points_nlpm, 2)
        self.pump_curve_current = np.polyfit(pressure_points_bar, current_points_amps, 2)
        
        self.total_energy_joules = 0.0
        self.is_initial_charging = True

    def get_pump_metrics(self, p_gauge_bar):
        """Returns the dynamic (max_flow_nlpm, max_current_amps) at current backpressure."""
        p_lookup = max(0.0, min(8.0, p_gauge_bar))
        flow_nlpm = max(0.0, float(np.polyval(self.pump_curve_flow, p_lookup)))
        current_amps = max(0.0, float(np.polyval(self.pump_curve_current, p_lookup)))
        return flow_nlpm, current_amps

    def calculate_controller_pwm(self, p_gauge_bar, demand_nlpm):
        """
        Hybrid Control Logic:
        1. Bang-Bang between 2 and 3 bar for initial charging and deep recovery.
        2. Feed-forward PWM matching to maintain steady flow otherwise.
        """
        max_available_flow, _ = self.get_pump_metrics(p_gauge_bar)
        
        # Hysteresis Safety Logic
        if p_gauge_bar <= 2.0:
            self.is_initial_charging = True
        elif p_gauge_bar >= 3.0:
            self.is_initial_charging = False
            
        if self.is_initial_charging:
            return 1.0 # 100% Duty Cycle to aggressively charge
            
        # Active PWM Matching (Holding pressure steady)
        if max_available_flow > 0:
            required_pwm = demand_nlpm / max_available_flow
        else:
            required_pwm = 1.0
            
        return max(0.0, min(1.0, required_pwm))

    def simulate(self, total_time_s=30.0, dt_s=0.01):
        time = np.arange(0, total_time_s, dt_s)
        
        # Data Loggers
        hist_p_tank, hist_pwm, hist_q_pump, hist_q_vent = [], [], [], []
        hist_power, hist_energy, hist_amps = [], [], []
        
        for t in time:
            # 1. Ventilator Demand Mode
            if t < 12.0:
                # Mode 1: Continuous 70 NLPM 
                q_vent_demand_nlpm = 70.0
            else:
                # Mode 2: 120 NLPM for 0.5s, 0 NLPM for 1.0s
                cycle_t = (t - 12.0) % 1.5
                if cycle_t < 0.5:
                    q_vent_demand_nlpm = 120.0
                else:
                    q_vent_demand_nlpm = 0.0

            # 2. Read Pneumatic State
            p_tank_abs = (self.mass_kg * self.R_air * self.T_k) / self.V_m3
            p_tank_gauge = (p_tank_abs - self.P_atm_pa) / 1e5
            
            # 3. Control Logic & Motor State
            current_pwm = self.calculate_controller_pwm(p_tank_gauge, q_vent_demand_nlpm)
            max_flow, max_amps = self.get_pump_metrics(p_tank_gauge)
            
            # Apply PWM Duty Cycle Scaling
            q_pump_nlpm = current_pwm * max_flow
            actual_amps = current_pwm * max_amps  # Assuming current scales with average PWM load
            power_w = actual_amps * self.voltage_v
            
            # Safety: Prevent pulling flow if tank is empty
            if p_tank_gauge <= 0.0 and q_pump_nlpm < q_vent_demand_nlpm:
                q_vent_actual = q_pump_nlpm
            else:
                q_vent_actual = q_vent_demand_nlpm
                
            # 4. Integrate System States (Mass and Energy Conservation)
            m_dot_in = (q_pump_nlpm / 1000.0 / 60.0) * self.rho_normal
            m_dot_out = (q_vent_actual / 1000.0 / 60.0) * self.rho_normal
            
            self.mass_kg += (m_dot_in - m_dot_out) * dt_s
            self.total_energy_joules += power_w * dt_s
            
            # 5. Log variables
            hist_p_tank.append(p_tank_gauge)
            hist_pwm.append(current_pwm * 100.0)
            hist_q_pump.append(q_pump_nlpm)
            hist_q_vent.append(q_vent_actual)
            hist_amps.append(actual_amps)
            hist_power.append(power_w)
            hist_energy.append(self.total_energy_joules / 1000.0)
            
        return time, hist_p_tank, hist_pwm, hist_q_pump, hist_q_vent, hist_power, hist_energy, hist_amps

# Run the Simulation (assuming 24V system)
sim = CompressorTankDigitalTwin(volume_liters=2.0, motor_voltage_v=24.0)
t, p_tank, pwm, q_pump, q_vent, power, energy, amps = sim.simulate(total_time_s=30.0)

# Plotting the 4 Required Graphs
fig, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True)

# Graph 1: Tank Pressure
axs[0].plot(t, p_tank, 'b-', linewidth=2, label='Tank Pressure')
axs[0].axhline(3.0, color='r', linestyle='--', label='Upper Controller Bound (3 Bar)')
axs[0].axhline(2.0, color='orange', linestyle='--', label='Lower Recovery Bound (2 Bar)')
axs[0].set_ylabel('Pressure (Bar Gauge)')
axs[0].set_title('Pneumatic & Electrical Dynamics (Using Empirical Pump Data)')
axs[0].legend(loc='lower right')
axs[0].grid(True)
axs[0].axvline(12.0, color='k', linestyle=':', alpha=0.5)

# Graph 2: Flow Rates
axs[1].plot(t, q_vent, 'm-', linewidth=2, label='Ventilator Demand Out')
axs[1].plot(t, q_pump, 'c--', linewidth=2, label='Compressor Flow In')
axs[1].set_ylabel('Flow Rate (NLPM)')
axs[1].legend(loc='upper right')
axs[1].grid(True)
axs[1].axvline(12.0, color='k', linestyle=':', alpha=0.5)

# Graph 3: Control & Current
ax3_2 = axs[2].twinx()
axs[2].plot(t, pwm, 'g-', linewidth=2, label='Compressor PWM Duty Cycle (%)')
ax3_2.plot(t, amps, 'k--', linewidth=2, label='Current Draw (Amps)')
axs[2].set_ylabel('Duty Cycle (%)', color='g')
ax3_2.set_ylabel('Motor Current (Amps)', color='k')
axs[2].grid(True)
axs[2].axvline(12.0, color='k', linestyle=':', alpha=0.5)
lines, labels = axs[2].get_legend_handles_labels()
lines2, labels2 = ax3_2.get_legend_handles_labels()
axs[2].legend(lines + lines2, labels + labels2, loc='upper right')

# Graph 4: Power and Energy
ax4_2 = axs[3].twinx()
axs[3].plot(t, power, 'r-', linewidth=2, label='Instantaneous Power (W)')
ax4_2.plot(t, energy, 'b--', linewidth=2, label='Total Energy (kJ)')
axs[3].set_ylabel('Power (Watts)', color='r')
ax4_2.set_ylabel('Energy Consumed (kJ)', color='b')
axs[3].set_xlabel('Time (Seconds)')
axs[3].grid(True)
axs[3].axvline(12.0, color='k', linestyle=':', alpha=0.5)

lines, labels = axs[3].get_legend_handles_labels()
lines2, labels2 = ax4_2.get_legend_handles_labels()
axs[3].legend(lines + lines2, labels + labels2, loc='upper left')

plt.tight_layout()
plt.show()