within ABS_PID_Model;
model RoadFrictionStep
  import Modelica.Blocks.Interfaces.*;
  RealOutput mu "Step friction";
  parameter Real mu_before=0.8;
  parameter Real mu_after=0.4;
  parameter Real tStep=2.0 "Step time [s]";
equation
  mu = if time < tStep then mu_before else mu_after;
end RoadFrictionStep;
