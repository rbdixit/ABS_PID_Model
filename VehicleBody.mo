within ABS_PID_Model;
model VehicleBody
  import Modelica.Blocks.Interfaces.*;
  parameter Real m=300 "Vehicle mass [kg]";
  parameter Real v0=30 "Initial speed [m/s]";

  RealInput  frictionForce "Longitudinal tire friction force [N]";
  RealOutput vehicleSpeed  "Vehicle longitudinal speed [m/s]";

protected 
  Real v(start=v0);

equation
  m*der(v) = -frictionForce;
  vehicleSpeed = v;
end VehicleBody;
