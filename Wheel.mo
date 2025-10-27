within ABS_PID_Model;
model Wheel
  import Modelica.Blocks.Interfaces.*;
  parameter Real J=1.2  "Wheel inertia [kg·m2]";
  parameter Real Rw=0.3 "Wheel effective radius [m]";
  parameter Real v_init=30 "Initial vehicle speed for omega start [m/s]";
  parameter Real small=1e-3 "Avoid div-by-zero";

  RealInput  vehicleSpeed   "Vehicle speed [m/s]";
  RealInput  frictionForce  "Longitudinal tire friction force [N]";
  RealInput  brakeTorque    "Brake torque [N·m]";
  RealOutput wheelSpeed     "Wheel angular velocity [rad/s]";
  RealOutput slipRatio      "Wheel slip ratio λ";

protected 
  Real omega(start=v_init/Rw);

equation
  // Dynamics: braking reduces ω
  J*der(omega) = Rw*frictionForce - brakeTorque;
  wheelSpeed   = omega;
  slipRatio    = (vehicleSpeed - Rw*omega) / (abs(vehicleSpeed) + small);
end Wheel;
