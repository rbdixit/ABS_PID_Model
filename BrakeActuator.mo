within ABS_PID_Model;
model BrakeActuator
  import Modelica.Blocks.Interfaces.*;
  RealInput  pedal;
  RealOutput torque;
  parameter Real maxTorque=2500;
  parameter Real tau=0.03;
protected
  Real y(start=0);
equation
  tau*der(y) = -y + maxTorque*pedal;
  torque = y;
end BrakeActuator;
