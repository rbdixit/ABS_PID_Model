within ABS_PID_Model;
model RoadFrictionSlipPeak
  import Modelica.Blocks.Interfaces.*;
  RealInput  slip "Slip ratio λ";
  RealOutput mu   "Friction coefficient";
  parameter Real mu_peak=0.8   "Peak μ";
  parameter Real lambda_peak=0.2 "Slip at peak μ";
  parameter Real eps=1e-6;
equation
  // μ(λ)=μp*(|λ|/λp)*exp(1-|λ|/λp), smooth single peak at λ=λp
  mu = mu_peak * (abs(slip)/max(lambda_peak,eps)) *
       exp(1 - abs(slip)/max(lambda_peak,eps));
end RoadFrictionSlipPeak;
