# Compressor Digital Twin

A physics-based **digital twin of a pneumatic compressor system for ventilator applications**, developed in Python to simulate the coupled behavior of the compressor, pneumatic components, thermal management system, storage tank, leakage paths, and battery/BMS.

The model combines **empirical compressor characterization** with first-principles fluid dynamics, thermodynamics, heat transfer, psychrometrics, pressure-loss models, control logic, and transient state updates.

A browser-based dashboard provides real-time control of simulation parameters and visualization of pressure, flow, temperature, leakage, power, compressor speed, and battery state.

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

The digital twin tracks the interaction between the pneumatic, thermal, electrical, and control subsystems at every simulation timestep.

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

## System Architecture

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

`Server.py` creates a simulation engine and coordinates the individual component models.

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

### Pneumatic model

Compressor flow is represented using a polynomial fit to empirical pressure-flow data:

```text
Q = f(ΔP)
```

where pressure is represented in gauge bar and flow in NLPM.

The predicted flow is then scaled by commanded compressor PWM.

The model also calculates compressor current from the empirical pressure-current relationship.

### Thermal model

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

### Thermal protection

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
* Transient pressure response

The silencer therefore acts as a dynamic pneumatic storage volume rather than being treated as a simple static pressure drop.

---

## 4. Intake Hose

**File:** `Intake_hose.py`

The intake hose model includes multiple flow restrictions:

* Upstream arc tube
* 135° bend
* Sudden contraction
* 600 mm HiPoFlex hose
* Three 90° bends
* Outlet elbow

Pressure losses are calculated using:

* Darcy-Weisbach friction losses
* Minor-loss coefficients
* Reynolds-number-dependent friction factor
* Dynamic pressure

The model also estimates heat transfer between the intake air and surroundings using an NTU-based approach.

Air properties vary with temperature using **Sutherland's law**.

---

## 5. Discharge Hose

**File:** `Discharge_hose.py`

The discharge hose model represents the high-pressure hose downstream of the compressor.

It calculates:

* Friction pressure drop
* Bend losses
* Fitting losses
* Air density
* Reynolds number
* Heat transfer to ambient

The model therefore couples pneumatic pressure loss and thermal behavior.

---

## 6. Cooling Coil

**File:** `cooling_coil2.py`

The cooling system uses a copper tube coil surrounded by two SUNON blowers.

The model represents:

```text
Compressed hot air
        ↓
Copper tube
        ↓
Forced external airflow
        ↓
Ambient air
```

### Internal convection

The model calculates:

* Air viscosity
* Thermal conductivity
* Prandtl number
* Reynolds number
* Dean number
* Internal Nusselt number
* Internal heat-transfer coefficient

The helical geometry is accounted for through a correction to the internal heat-transfer correlation.

### External convection

External airflow around the tube is modeled using the **Churchill-Bernstein correlation** for crossflow over a cylinder.

### NTU-based thermal model

The model calculates the required coil length to achieve a specified temperature approach to ambient.

It also calculates the actual outlet temperature for a selected coil length.

### Pressure loss

The coil pressure loss includes a helical-flow friction correction based on the Dean number.

---

## 7. Non-Return Valve

**File:** `NRV.py`

The `SMC_AKH10_NRV` model represents a pneumatic check valve.

The model uses the **ISO 6358 pneumatic-flow formulation** to determine whether the valve is operating in:

* Subsonic flow
* Choked flow

The model also includes the valve cracking pressure.

Outputs include:

* Outlet pressure
* Pressure drop
* Outlet temperature

---

## 8. Air Filter

**File:** `air_filter.py`

The downstream particulate filter model calculates:

* Pressure drop
* Air leakage through the drain
* Effective downstream mass flow

The model accounts for compressible air properties and temperature-dependent viscosity.

---

## 9. Water Separator

**File:** `water_separater.py`

The water separator combines:

### Psychrometric modeling

The model determines the amount of water that can remain in vapor form using:

* Ambient relative humidity
* Ambient temperature
* Pressure
* Coil outlet temperature
* Saturation pressure

The saturation pressure is calculated using the **Antoine equation**.

The humidity ratio is then used to determine:

```text
Water vapor entering
        ↓
Maximum vapor capacity
        ↓
Condensed liquid water
        ↓
Remaining vapor
```

### Centrifugal separation

The model additionally estimates water-removal efficiency as a function of flow rate.

### Drain leakage

The separator drain is coupled to the restrictor-flow model.

---

## 10. Mist Separator

**File:** `mist_separator.py`

The mist separator models an SMC AFM20-type micro-mist separator.

The model tracks:

* Pressure drop
* Drain leakage
* Effective downstream mass flow
* Aerosol capture
* Filter saturation

A dynamic saturation state is maintained:

```text
saturation_ratio = retained_water / element_capacity
```

The separator uses a nominal **99.9% coalescing efficiency** for the modeled aerosol stream.

The model also includes aerodynamic clearing behavior at higher flow rates.

---

## 11. Drain Leakage Model

**File:** `Drain_leak_calc.py`

The `RestrictorFlowSolver` calculates leakage through component drain restrictors.

The model handles compressible flow through a small restriction and determines the appropriate flow regime.

The implementation includes:

* Restrictor geometry
* Pressure ratio
* Temperature
* Compressible flow
* Mach-number solution
* Fanno-flow relation
* Choked/subsonic behavior

This model is reused by multiple pneumatic components.

---

# Storage Tank Digital Twin

**File:** `Digital_tank2.py`

The `SmartCompressorTankTwin` represents a 2 L compressed-air storage tank.

The tank state is updated using conservation of:

* Mass
* Internal energy

The model calculates:

```text
P = mRT / V
```

while dynamically updating tank temperature from the internal-energy state.

### Energy balance

The tank uses the first law of thermodynamics:

```text
dU/dt =
    enthalpy in
  - enthalpy out
  - leakage enthalpy
```

The model therefore captures temperature changes caused by charging and discharging.

---

# Compressor Control

The tank controller uses a three-state control strategy:

```text
                 Pressure ≤ minimum
                         │
                         ▼
                  MAX_RECOVERY
                    100% PWM
                         │
                         │ pressure rises
                         ▼
                  POWER_SAVING
                Closed-loop control
                         │
                         │ pressure ≥ 4 bar
                         ▼
                    COAST_DOWN
                     Pump off
```

The recovery thresholds are configurable through the dashboard.

During `POWER_SAVING`, the controller uses the actual pneumatic inflow to correct the compressor PWM based on the difference between required and delivered flow.

---

# Cooling-Fan Control

The cooling blower can operate in:

```text
AUTO
MANUAL RPM
```

In automatic operation, a PID controller uses compressor-head temperature as feedback.

The implemented controller contains:

* Proportional term
* Integral term
* Derivative term
* Output saturation
* Anti-windup behavior

The blower RPM is mapped from experimentally characterized PWM-RPM data.

Two SUNON blowers are represented in the cooling-system airflow calculation.

---

# Battery and BMS Model

**File:** `battery.py`

The `AdvancedVentilatorBMS` model represents a rechargeable battery pack and its interaction with the compressor.

The modeled pack configuration is:

```text
6S4P
```

The BMS tracks:

* State of charge
* Pack voltage
* Internal resistance
* Battery temperature
* Charge current
* Discharge current
* Estimated remaining runtime
* Cooling-fan state
* Compressor derating

### Electrical model

Open-circuit voltage is estimated from a fifth-order polynomial of SoC.

Terminal voltage is modeled as:

```text
Vout = Voc - I R0
```

### Thermal model

Battery heat generation is represented using:

```text
Q̇ = I²R
```

Battery temperature evolves from the balance between electrical heat generation and thermal dissipation.

### Protection and derating

The BMS can reduce compressor power based on:

* Battery temperature
* State of charge
* Battery fault conditions

The resulting derating multiplier is fed directly back into the compressor command.

---

# Dashboard

**Files:**

```text
dashboard.html
Server.py
```

The project includes a browser-based interface called:

**G&M Tech Digital Twin Studio**

The dashboard communicates with the Python simulation engine using a WebSocket connection:

```text
ws://localhost:8765
```

### Configurable parameters

The dashboard allows the user to configure:

* Compressor model
* Simulation duration
* Ambient temperature
* HEPA filter usage hours
* Minimum compressor RPM
* Tank recovery pressure
* Tank maximum pressure
* Inspiratory flow
* Expiratory flow
* Inspiration time
* Expiration time
* Pump operating mode
* Pump PWM
* Pump RPM
* Blower operating mode
* Blower RPM
* Component drain/restrictor diameters

### Pump operating modes

```text
AUTO
MANUAL PWM
MANUAL RPM
```

### Blower operating modes

```text
AUTO
MANUAL RPM
```

### Power-source control

The dashboard can switch between:

```text
AC Mains
Battery
```

This allows the BMS and compressor interaction during battery operation to be simulated.

---

# Simulation Outputs

The simulation continuously generates telemetry including:

### Pneumatic

* Compressor flow
* Tank flow
* Delivered flow
* Tank pressure
* Pressure cascade
* Component pressure drops
* Drain leakage

### Thermal

* Compressor motor temperature
* Pump-head temperature
* Gas temperature
* Cooling performance
* Battery temperature

### Electrical

* Compressor power
* Compressor current
* Battery voltage
* Battery SoC
* Battery derating

### Control

* Compressor PWM
* Compressor RPM
* Blower RPM
* Blower CFM
* Tank control state
* BMS status

---

# Repository Structure

```text
Compressor_model/
│
├── Server.py
├── Main2.py
├── dashboard.html
│
├── Pump.py
├── Digital_tank2.py
├── battery.py
│
├── Intake_hose.py
├── Discharge_hose.py
├── cooling_coil.py
├── cooling_coil2.py
│
├── Hepa2.py
├── air_filter.py
├── Silencer.py
├── NRV.py
├── water_separater.py
├── mist_separator.py
├── Drain_leak_calc.py
│
├── requirements.txt
├── start_server.bat
└── .gitignore
```

### Main files

| File                 | Purpose                                               |
| -------------------- | ----------------------------------------------------- |
| `Server.py`          | Main real-time simulation engine and WebSocket server |
| `Pump.py`            | Compressor pneumatic, electrical and thermal model    |
| `Digital_tank2.py`   | Storage-tank mass/energy/pressure model               |
| `battery.py`         | Battery and BMS model                                 |
| `cooling_coil2.py`   | Copper coil heat-transfer and pressure-drop model     |
| `Intake_hose.py`     | Intake-side pneumatic and thermal losses              |
| `Discharge_hose.py`  | Discharge-side pneumatic and thermal losses           |
| `Hepa2.py`           | Intake HEPA filter model                              |
| `air_filter.py`      | Downstream particulate-filter model                   |
| `Silencer.py`        | Intake silencer transient model                       |
| `NRV.py`             | Non-return valve model                                |
| `water_separater.py` | Condensation and water-separation model               |
| `mist_separator.py`  | Micro-mist separator model                            |
| `Drain_leak_calc.py` | Common drain/restrictor leakage model                 |
| `dashboard.html`     | Interactive browser dashboard                         |
| `Main2.py`           | Standalone simulation/plotting workflow               |
| `requirements.txt`   | Python dependencies                                   |
| `start_server.bat`   | Windows launcher                                      |

---

# Installation

## 1. Clone the repository

```bash
git clone <your-repository-url>
cd Compressor_model
```

## 2. Create a virtual environment

Windows:

```bash
python -m venv venv
venv\Scripts\activate
```

Linux/macOS:

```bash
python3 -m venv venv
source venv/bin/activate
```

## 3. Install dependencies

```bash
pip install -r requirements.txt
```

The main numerical and scientific dependencies include:

```text
NumPy
SciPy
Pandas
Matplotlib
Plotly
CoolProp
fluids
websockets
```

---

# Running the Digital Twin

## Option 1 — Windows

Run:

```text
start_server.bat
```

The server starts on:

```text
ws://localhost:8765
```

and opens the dashboard in the browser.

## Option 2 — Command Line

Start the server with:

```bash
python Server.py
```

Then open:

```text
dashboard.html
```

in a browser.

---

# Running Individual Component Tests

Most component files contain standalone test blocks.

For example:

```bash
python Pump.py
```

or:

```bash
python Digital_tank2.py
```

Individual component models can therefore be tested independently before integrating them into the complete simulation.

---

# Simulation Timestep

The integrated real-time server currently uses:

```text
Δt = 0.05 s
```

Simulation outputs are streamed at each simulation step while the dashboard visualizes the system state.

---

# Modeling Approach

The digital twin follows a hybrid modeling approach.

### Empirical models

Used where experimental characterization is available:

* Compressor pressure-flow behavior
* Compressor current
* Compressor thermal behavior
* HEPA filter pressure drop
* Blower PWM-RPM relationship

### First-principles models

Used for physical components and state evolution:

* Ideal-gas relations
* Conservation of mass
* Conservation of energy
* Darcy-Weisbach pressure losses
* Minor-loss coefficients
* Reynolds number
* Sutherland's law
* Nusselt correlations
* NTU heat-transfer modeling
* Dean-number corrections
* ISO 6358 pneumatic flow
* Psychrometric calculations
* Antoine equation
* Compressible restrictor flow
* Battery electrical and thermal balances

This combination allows experimentally measured component behavior to be integrated with physics-based system-level dynamics.

---

# Engineering Applications

The digital twin can be used for:

* Compressor sizing
* Pneumatic-system pressure-drop analysis
* Tank sizing
* Compressor duty-cycle analysis
* Thermal-management studies
* Cooling-coil sizing
* Filter pressure-drop analysis
* Leakage estimation
* Battery runtime estimation
* BMS interaction studies
* Control-system tuning
* Component sensitivity studies
* System-level design iteration
* Experimental-data comparison

---

# Current Model Assumptions

The model contains a mixture of measured, empirical, and engineering-assumption-based parameters.

Important assumptions include:

* Air is treated primarily using ideal-gas relations.
* Compressor performance is represented using polynomial fits to empirical data.
* Several minor-loss coefficients are engineering estimates.
* Cooling airflow through the coil is represented using an estimated operating fraction of fan free-air capacity.
* Ambient humidity is currently represented by a fixed relative-humidity value in the integrated server.
* Some component characteristics are based on manufacturer specifications or fitted correlations.
* The digital twin is intended for engineering analysis and design iteration and should be validated against hardware measurements before being used for final safety-critical decisions.

---

# Validation Strategy

A useful validation workflow is:

```text
Experimental Test
       ↓
Measured P / Q / T / I
       ↓
Compare with Digital Twin
       ↓
Calculate Model Error
       ↓
Identify dominant mismatch
       ↓
Update component model
       ↓
Re-run system simulation
```

The modular architecture makes it possible to calibrate individual components without rewriting the entire system model.

---

# Future Development

Potential extensions include:

* Automated experimental-data ingestion
* Automated model calibration
* Parameter sensitivity analysis
* Uncertainty propagation
* Automated design-space exploration
* Hardware-in-the-loop testing
* Real sensor integration
* More detailed compressor thermodynamics
* More detailed humidity and condensation dynamics
* Patient-side ventilator coupling
* Automated model-vs-experiment validation plots
* Optimization of tank volume and compressor control thresholds

---

# Technologies

```text
Python
NumPy
SciPy
Pandas
Matplotlib
Plotly
CoolProp
fluids
WebSockets
HTML / CSS / JavaScript
```

---

# Author

Developed as a compressor-system digital-twin and simulation framework for pneumatic and thermal system analysis.

---

## Disclaimer

This project is an engineering simulation and development model. Component correlations, empirical fits, assumptions, and control logic should be validated against experimental hardware data before the model is used for production decisions or safety-critical applications.
