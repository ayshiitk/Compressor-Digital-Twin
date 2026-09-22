# Compressor Digital Twin

A physics-based **digital twin of a pneumatic compressor system for ventilator applications**, developed in Python to simulate the coupled behavior of the compressor, pneumatic components, thermal management system, storage tank, leakage paths, and battery/BMS.

The model combines **empirical compressor characterization** with first-principles fluid dynamics, thermodynamics, heat transfer, psychrometrics, pressure-loss models, control logic, and transient state updates.

A browser-based dashboard provides real-time control of simulation parameters and visualization of pressure, flow, temperature, leakage, power, compressor speed, and battery state.

---

## Dashboard Demo

The following video demonstrates the real-time digital twin dashboard, including compressor control, tank pressure evolution, thermal behavior, blower control, and battery/BMS interaction.

**[▶️ Watch the Compressor Digital Twin Dashboard Demo](https://drive.google.com/file/d/1bBHHkNhhvxhWxw3FlXnh24Enpcu0qvEF/view?usp=sharing)**


---

## Overview

The system represents the complete compressed-air path from atmospheric intake to the storage tank:

```text
Ambient Air
    │
    ▼
HEPA Filter
    │
    ▼
Silencer / Muffler
    │
    ▼
Intake Hose
    │
    ▼
Compressor
    │
    ▼
Discharge Hose
    │
    ▼
Copper Cooling Coil + Fans
    │
    ▼
Non-Return Valve (NRV)
    │
    ▼
Particulate Air Filter
    │
    ▼
Water Separator
    │
    ▼
Mist Separator
    │
    ▼
Storage Tank
    │
    ▼
Ventilator Demand
```

The digital twin tracks the interaction between the **pneumatic, thermal, electrical, and control subsystems** at every simulation timestep.

---

## Key Features

* Physics-based transient simulation of the complete compressor train
* Empirically calibrated compressor performance curves
* Support for **120RND-ED** and **140RND-ED** compressor models
* Compressor flow and current prediction as a function of pressure
* Thermal model for compressor motor and pump head
* Thermal safety throttling based on motor temperature
* HEPA filter pressure-drop model with filter-age dependence
* Intake and discharge hose pressure-loss models
* Compressible-flow and minor-loss calculations
* Copper cooling-coil heat-transfer model
* Dual-fan cooling model
* Non-return valve model based on ISO 6358 pneumatic-flow relations
* Air-filter pressure drop and drain leakage modeling
* Water condensation and separation model
* Micro-mist separator with dynamic saturation tracking
* Choked/subsonic drain leakage calculations
* Storage-tank mass and energy balance
* Tank pressure dynamics
* Automatic compressor control with recovery, power-saving, and coast-down states
* Thermal PID control for cooling blower
* Battery and BMS simulation
* Battery thermal behavior and state-of-charge tracking
* Battery-based compressor power derating
* AC mains / battery operating modes
* Real-time WebSocket communication
* Browser-based interactive simulation dashboard
* Configurable operating conditions and component parameters

---

# System Architecture

The simulation is organized as modular Python component models.

```text
                         ┌──────────────────────┐
                         │   Simulation Server  │
                         │      Server.py       │
                         └──────────┬───────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              │                     │                     │
              ▼                     ▼                     ▼
      Pneumatic System        Thermal Control       Battery / BMS
              │                     │                     │
              └─────────────────────┼─────────────────────┘
                                    │
                                    ▼
                              Live Telemetry
                                    │
                                    ▼
                           WebSocket @ :8765
                                    │
                                    ▼
                          dashboard.html
```

At each timestep, the following sequence is evaluated:

```text
1. Determine compressor command
2. Apply minimum-RPM constraint
3. Apply battery derating
4. Calculate compressor flow/current/temperature
5. Calculate discharge-hose losses
6. Calculate cooling-coil heat transfer and pressure drop
7. Calculate NRV pressure loss
8. Calculate air-filter pressure drop and leakage
9. Calculate water separation and condensation
10. Calculate mist-separator pressure drop and leakage
11. Update tank mass, energy and pressure
12. Update cooling-fan controller
13. Update battery/BMS state
14. Calculate system leakage and telemetry
15. Send results to dashboard
```

---

# Component Models

## 1. Compressor Model

**File:** `Pump.py`

The `SmartCompressor` model represents the G&M Tech compressor and combines empirical pneumatic performance with a transient thermal model.

Two compressor configurations are available:

```text
120RND
140RND
```

The compressor uses experimentally characterized:

* Pressure-flow data
* Pressure-current data
* Thermal test data
* Motor temperature data
* Pump-head temperature data
* Cooling blower data

### Pneumatic Model

Compressor flow is represented using a polynomial fit to empirical pressure-flow data:

```text
Q = f(ΔP)
```

where pressure is represented in gauge bar and flow in NLPM.

The predicted flow is then scaled by commanded compressor PWM.

The model also calculates compressor current from the empirical pressure-current relationship.

### Thermal Model

The compressor thermal model estimates:

* Motor temperature
* Pump-head temperature
* Gas outlet temperature
* Electrical power
* Cooling effect of the blower

Thermal behavior is calibrated using experimentally measured temperature data.

Multiple candidate regression feature sets are evaluated using **leave-one-out cross-validation (LOOCV)** to select thermal models.

Relevant variables include:

```text
Pump speed
Pressure
Flow
Ambient temperature
Blower speed
```

### Thermal Protection

The compressor implements temperature-based throttling:

```text
Motor temperature < throttle threshold
        ↓
Full requested PWM

Motor temperature in throttle region
        ↓
Progressive PWM reduction

Motor temperature ≥ thermal limit
        ↓
Compressor shutdown
```

---

## 2. HEPA Filter

**File:** `Hepa2.py`

The `ZF111_HEPA_Filter` models the intake HEPA filter.

Pressure-drop characteristics are represented using empirical datasets for different filter usage levels:

```text
0 hours
500 hours
1000 hours
1500 hours
```

For each operating age, a second-order polynomial is fitted to the measured pressure-drop data.

The model automatically:

1. Selects the appropriate filter-age dataset
2. Limits flow to the valid experimental range
3. Calculates pressure drop
4. Prevents negative pressure-drop predictions

This allows filter loading to be included directly in the compressor's inlet-side performance.

---

## 3. Silencer / Muffler

**File:** `Silencer.py`

The `AcousticSilencerChamber` models the pneumatic volume between the HEPA filter and compressor inlet.

The model tracks:

* Chamber mass
* Chamber pressure
* Inlet flow
* Compressor demand
* Transient pressure respo
