import numpy as np

class AdvancedVentilatorBMS:
    def __init__(self, s_config=6, p_config=4, cell_capacity_ah=5.14, initial_soc=1.0):
        # --- 1. Pack Architecture (6S4P Samsung 53G) ---
        self.S = s_config
        self.P = p_config
        self.pack_capacity_ah = cell_capacity_ah * self.P
        self.pack_capacity_amp_seconds = self.pack_capacity_ah * 3600
        self.soc = initial_soc
        
        # --- 2. Limits & Constraints (Datasheet) ---
        self.max_charge_temp = 45.0      # °C
        self.max_discharge_temp = 60.0   # °C
        self.target_voltage = 4.15 * self.S # 24.9V (Cycle Life limit)
        self.empty_voltage = 3.0 * self.S   # 18.0V (Cycle Life limit)
        
        self.max_charge_current = 1.75 * self.P     # 7.0A (0.33C)
        self.cutoff_charge_current = 0.265 * self.P # 1.06A (0.05C)
        
        # --- 3. Electrical & Thermal Baselines ---
        self.cell_r0_base = 0.025 
        self.temp_c = 25.0
        self.t_amb = 25.0
        self.thermal_mass = 1.85 * 1050.0  # Joules/°C
        
        # From 600W lab calibration
        i_test_avg = 27.55 
        q_test_watts = (i_test_avg ** 2) * (self.cell_r0_base * (self.S / self.P))
        self.r_th_no_fan = 44.0 / q_test_watts
        self.r_th_60mm_fan = 27.0 / q_test_watts
        
        self.r_th = self.r_th_60mm_fan
        self.fan_pwm = 0
        self.status = "INITIALIZED"

    def get_pack_r0(self):
        """Temperature adjusted internal DC resistance"""
        temp_factor = max(0.75, min(1.5, 1.0 - 0.005 * (self.temp_c - 25.0)))
        return (self.cell_r0_base * (self.S / self.P)) * temp_factor

    def calculate_voc(self):
        """5th Order Polynomial for NMC 21700 Open Circuit Voltage"""
        a0, a1, a2, a3, a4, a5 = 3.00, 4.30, -16.5, 31.0, -25.5, 7.90
        x = self.soc
        cell_voc = a0 + (a1*x) + (a2*(x**2)) + (a3*(x**3)) + (a4*(x**4)) + (a5*(x**5))
        return cell_voc * self.S

    def manage_thermal_system(self):
        """Controls the 60mm fan based on pack temperature"""
        if self.temp_c < 35.0:
            self.fan_pwm = 0
            self.r_th = self.r_th_no_fan
        else:
            # Above 35C, run the 60mm fan to hold 0.95 C/W resistance
            self.fan_pwm = 100
            self.r_th = self.r_th_60mm_fan

    def calculate_derating(self):
        """
        Calculates a power multiplier (0.0 to 1.0) to send to the compressor.
        1.0 = Full performance allowed.
        < 1.0 = Compressor must reduce PWM/RPM to save the battery.
        0.0 = Hard Stop (Fault)
        """
        multiplier = 1.0
        
        # 1. Thermal Derating
        if self.temp_c >= self.max_discharge_temp:
            multiplier = 0.0  # 60C limit hit. Hard fault.
        elif self.temp_c >= 55.0:
            multiplier = 0.5  # Warning zone. Halve compressor power.
            
        # 2. State of Charge Derating
        if self.soc <= 0.05:
            multiplier = min(multiplier, 0.5) # Battery almost dead, throttle down
            
        return multiplier

    def step(self, ac_connected, compressor_draw_amps, dt):
        """
        Main loop. Call this every time-step from your main compressor script.
        """
        self.manage_thermal_system()
        pack_r0 = self.get_pack_r0()
        voc = self.calculate_voc()
        
        battery_amps = 0.0
        time_remaining_mins = 0.0
        
        # ==========================================
        # LOGIC 1: AC MAINS CONNECTED (CHARGING)
        # ==========================================
        if ac_connected:
            if self.temp_c >= self.max_charge_temp:
                self.status = "AC CONNECTED: COOLING (CHARGE BLOCKED)"
                battery_amps = 0.0
                time_remaining_mins = 999.0
            elif voc >= self.target_voltage:
                # Constant Voltage (CV) Phase
                cv_current = (self.target_voltage - voc) / pack_r0
                battery_amps = -min(abs(cv_current), self.max_charge_current)
                
                if abs(battery_amps) <= self.cutoff_charge_current:
                    self.status = "AC CONNECTED: STANDBY (100% FULL)"
                    battery_amps = 0.0
                    time_remaining_mins = 0.0
                else:
                    self.status = "AC CONNECTED: CHARGING (CV TAPER)"
                    # Estimate remaining time in CV phase (~45 mins usually)
                    capacity_needed = (1.0 - self.soc) * self.pack_capacity_ah
                    time_remaining_mins = (capacity_needed / max(0.1, abs(battery_amps))) * 60.0
            else:
                # Constant Current (CC) Phase
                self.status = "AC CONNECTED: CHARGING (CC FAST)"
                battery_amps = -self.max_charge_current
                capacity_needed = (1.0 - self.soc) * self.pack_capacity_ah
                time_remaining_mins = (capacity_needed / self.max_charge_current) * 60.0 + 45.0 # Add 45m for CV
                
        # ==========================================
        # LOGIC 2: BATTERY MODE (DISCHARGING)
        # ==========================================
        else:
            if self.temp_c >= self.max_discharge_temp:
                self.status = "BATTERY FAULT: OVER-TEMP"
                battery_amps = 0.0
                time_remaining_mins = 0.0
            elif voc <= self.empty_voltage or self.soc <= 0.0:
                self.status = "BATTERY FAULT: EMPTY"
                battery_amps = 0.0
                time_remaining_mins = 0.0
            else:
                self.status = "BATTERY MODE: DISCHARGING"
                battery_amps = compressor_draw_amps
                
                # Predict time to empty at current load
                if battery_amps > 0:
                    usable_capacity = self.soc * self.pack_capacity_ah
                    time_remaining_mins = (usable_capacity / battery_amps) * 60.0
                else:
                    time_remaining_mins = 999.0 # No load

        # --- UPDATE PHYSICS VIA DIFFERENTIAL EQUATIONS ---
        
        # 1. Thermal Update
        q_gen = (battery_amps ** 2) * pack_r0
        q_diss = (self.temp_c - self.t_amb) / self.r_th
        self.temp_c += ((q_gen - q_diss) * dt) / self.thermal_mass
        
        # 2. Electrical Update
        self.soc -= (battery_amps * dt) / self.pack_capacity_amp_seconds
        self.soc = max(0.0, min(1.0, self.soc))
        v_out = voc - (battery_amps * pack_r0)
        
        # Calculate safe performance limits to send back to compressor
        derating = self.calculate_derating()
        
        # Package everything nicely for your main loop
        bms_data = {
            "v_out": v_out,
            "soc_percent": self.soc * 100.0,
            "temperature_c": self.temp_c,
            "status": self.status,
            "time_remaining_min": time_remaining_mins,
            "derating_multiplier": derating,
            "fan_pwm": self.fan_pwm
        }
        
        return bms_data

# ==========================================
# SIMULATION: Power Outage & Recovery Test
# ==========================================
if __name__ == "__main__":
    bms = AdvancedVentilatorBMS(initial_soc=0.20) # Start at 20% Battery
    
    print("TEST: Battery Mode (High Load) -> Power Restored -> Recharging")
    print(f"{'Time(m)':<8} | {'Status':<35} | {'V_Out':<6} | {'SoC(%)':<7} | {'Temp(°C)':<9} | {'Est.Time':<9} | {'Derating':<8}")
    print("-" * 100)
    
    dt = 1.0 
    
    for step in range(1, 7200): # Simulate 2 hours (7200 seconds)
        time_min = step / 60.0
        
        # Phase 1: 0 to 15 mins -> AC is OFF, Compressor pulling heavy 20A load
        if time_min < 15:
            ac_power = False
            requested_load = 20.0
        # Phase 2: 15 mins onwards -> AC Power Restored, Compressor runs on mains
        else:
            ac_power = True
            requested_load = 10.0 # Handled by wall power, battery ignores this
            
        data = bms.step(ac_connected=ac_power, compressor_draw_amps=requested_load, dt=dt)
        
        # Print status every 3 minutes
        if step % 180 == 0:
            est_t = f"{data['time_remaining_min']:.1f}m"
            print(f"{time_min:<8.1f} | {data['status']:<35} | {data['v_out']:<6.1f} | {data['soc_percent']:<7.1f} | {data['temperature_c']:<9.1f} | {est_t:<9} | {data['derating_multiplier']:<8.1f}")