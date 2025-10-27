# ABS_PID_Model

Single-wheel ABS in Modelica (OMEdit 1.25.4).  
Includes PID slip control, constant-/step-μ roads, slip-peak tire, and μ-step adaptation.

## Structure
- `ABSSystem.mo` – base system with constant torque cap at μ_peak
- `ABSSystem_MuStep.mo` – road changes dry→wet smoothly (t=2 s)
- `Wheel.mo`, `VehicleBody.mo`, `PIDController.mo`, `BrakeActuator.mo`, `DriverPedalRamp.mo`
- `RoadFriction*.mo` – constant, step, and slip-peak tire models

## How to run (OMEdit)
1. `File → Open Model/Library` → open `package.mo`
2. Simulate `ABS_PID_Model.ABSSystem` (or `_MuStep`)
3. Plot `wheel.slipRatio`, `T_b`, `Troad_max`, `mu`, `vehicleBody.vehicleSpeed`

## Tested
- OpenModelica 1.25.4
- Modelica Standard Library 4.0.0
