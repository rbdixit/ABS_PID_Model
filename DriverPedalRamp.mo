within ABS_PID_Model;
model DriverPedalRamp
  import Modelica.Blocks.Interfaces.*;
  RealOutput cmd;
  parameter Real height=1;
  parameter Real duration=0.3;
  parameter Real startTime=0.2;
  parameter Real offset=0;
equation
  cmd = min(max(offset + (if time < startTime then 0 else min((time - startTime)/duration, 1)*height), 0), 1);
end DriverPedalRamp;
