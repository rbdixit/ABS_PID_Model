Author: Rajiv Dixit  
Version: 0.1.0  
Tested on: OpenModelica 1.25.4

# 🛞 ABS_PID_Model

A single-wheel Anti-lock Braking System (ABS) built in **OpenModelica**, using a **PID-based slip control** strategy and realistic **slip-peak tire friction**.  
The library is modular, reusable, and designed for educational and research use.

---

## 🚗 Overview

The model simulates a **single-wheel ABS** where brake torque is actively controlled to maintain an optimal slip ratio (~0.2).  
It includes subsystems for the wheel, vehicle body, controller, actuator, driver input, and road friction — each written as a reusable Modelica class.

The simulation can represent:
- Braking on **constant-friction (dry)** surfaces.
- **Dry → wet** transitions (μ drop) with dynamic torque capping.
- Optional **visual block diagram** in OMEdit for demonstration.

---

## ⚙️ How to Use

1. Open `package.mo` in **OMEdit** or **OpenModelica**.
2. Load the library:  
   `File → Open Model/Library → ABS_PID_Model/package.mo`
3. Simulate one of the systems:
   - `ABS_PID_Model.ABSSystem` – constant dry road.
   - `ABS_PID_Model.ABSSystem_MuStep` – dry → wet transition at 2 s.
4. Plot:
   - `wheel.slipRatio`
   - `T_b`, `Troad_max`
   - `mu`
   - `vehicleBody.vehicleSpeed` and `wheel.wheelSpeed*wheel.Rw`

---

## 🧩 Model Summary

| **Model** | **Description / Function** | **Key Equations / Logic** | **Main Assumptions** | **Purpose** |
|:--|:--|:--|:--|:--|
| **VehicleBody.mo** | Quarter-vehicle longitudinal model; computes vehicle velocity under braking. | \( m \cdot \dot{v} = -F_x \) | No drag, slope, or tire compliance. Constant mass. | Provides vehicle speed feedback to other subsystems. |
| **Wheel.mo** | Rotational dynamics of a single wheel; computes angular velocity and slip ratio. | \( J \cdot \dot{\omega} = T_b - F_x R_w \); \( λ = (v - ωR_w)/v \) | Rigid tire; constant normal load; no lateral slip. | Converts brake torque into slip ratio for ABS control. |
| **PIDController.mo** | Closed-loop slip controller regulating wheel slip around target value. | \( T_{cmd} = K_p e + K_i \int e + K_d \dot{e} \) | Continuous-time; ideal sensors; includes anti-windup and torque saturation. | Adjusts braking torque to maintain optimal slip (≈0.2). |
| **BrakeActuator.mo** | First-order dynamic brake actuator; converts pedal signal to torque. | \( T = \frac{T_{max} u}{1 + sτ} \) | Linear hydraulic response; no hysteresis. | Simulates brake pressure delay and limit. |
| **DriverPedalRamp.mo** | Driver input model; ramp from 0 to 1 simulating pedal press. | Linear ramp: \( u(t) = (t-t_0)/T \) | Smooth pedal application; no release phase. | Provides consistent driver input for testing. |
| **RoadFrictionSlipPeak.mo** | Slip-dependent tire-road friction model with single peak. | \( μ(λ)=μ_p(|λ|/λ_p)e^{1-|λ|/λ_p} \) | Friction depends only on slip ratio; constant μ_peak, λ_peak. | Represents tire behavior on a uniform road (dry). |
| **RoadFrictionSlipPeakStep.mo** | Same as above, but μ_peak changes at a defined time (dry→wet). | μ scales by smooth tanh step at \( t = t_{step} \). | Uniform friction drop; smooth transition (τ smoothing). | Tests controller stability during road condition change. |
| **RoadFrictionStep.mo** | Simple constant μ with step change (no slip dependency). | \( μ = μ_1 \) or \( μ_2 \) depending on time. | Idealized two-level road friction. | Early testing of torque limiting and event handling. |
| **ABSSystem.mo** | Integrates all subsystems (PID, wheel, body, actuator, constant μ road). | Combines dynamics + torque limiter \( T_{b} ≤ R_w μ_p F_z \) | One wheel; constant μ; no coupling effects. | Baseline ABS system — constant surface. |
| **ABSSystem_MuStep.mo** | Full ABS with μ step (dry→wet) using adaptive torque cap. | Uses `RoadFrictionSlipPeakStep` for dynamic μ_peak. | Road friction changes mid-simulation. | Tests adaptation under friction change. |
| **ABSSystem_Block.mo** | Same as ABSSystem but with layout annotations for OMEdit diagram. | Visual only (no equation change). | — | For schematic visualization and training use. |

---

## 🌍 Global Modeling Assumptions

| **Aspect** | **Assumption** |
|:--|:--|
| Vehicle model | Single-wheel (¼ vehicle) representation. |
| Motion | Longitudinal only; no lateral or pitch effects. |
| Load | Constant vertical load \( F_z = m g / 4 \). |
| Friction | μ depends only on slip ratio; no velocity or temperature dependence. |
| Brake actuator | Linear first-order system; no hysteresis. |
| Controller | Continuous PID; no sampling or quantization. |
| Sensors | Perfect (no noise, delay, or bias). |
| Terrain | Flat road; no gradient or aerodynamic drag. |

---

## 🧠 Key Takeaways

- The ABS maintains slip ratio near **0.15–0.20**, maximizing tire-road friction.  
- When road μ drops, the controller **automatically reduces torque** to avoid wheel lockup.  
- The modular structure makes it easy to extend:
  - Add multi-wheel dynamics,
  - Include weight transfer,
  - Replace PID with fuzzy logic or MPC.

---

## 🧩 License

Released under the **MIT License**.  
Feel free to reuse and extend for academic or educational projects.

---

## 🧰 Credits

Developed in **OpenModelica 1.25.4**, using **Modelica Standard Library 4.0.0**.  
© 2025 — *Rajiv Dixit*  




