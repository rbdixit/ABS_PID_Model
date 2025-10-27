within ABS_PID_Model;
model RoadFriction
  import Modelica.Blocks.Interfaces.*;
  RealOutput mu "Constant friction coefficient";
  parameter Real mu_const=0.8 "Friction coefficient";
equation
  mu = mu_const;
end RoadFriction;
