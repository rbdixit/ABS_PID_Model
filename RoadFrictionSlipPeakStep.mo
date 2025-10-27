within ABS_PID_Model;
model RoadFrictionSlipPeakStep
  import Modelica.Blocks.Interfaces.*;

  // Inputs / outputs
  RealInput  slip           "Slip ratio λ";
  RealOutput mu             "Friction coefficient";
  RealOutput mu_cap         "Effective peak μ used for torque cap";

  // Parameters
  parameter Real mu_dry=0.8  "Dry-road peak μ";
  parameter Real mu_wet=0.4  "Wet-road peak μ";
  parameter Real lambda_peak=0.2 "Slip at peak μ";
  parameter Real tStep=2.0   "Time when road becomes wet [s]";
  parameter Real tau=0.05    "Smoothing time for the step [s]";
  parameter Real eps=1e-6;

protected
  Real r = mu_wet/mu_dry "Wet/dry peak ratio";
  // Smooth step: scale ≈ 1 (dry) → r (wet)
  Real s  = 0.5*(1 - tanh((time - tStep)/tau));
  Real scale = r + (1 - r)*s;
  Real mu_peak_eff;

equation
  mu_peak_eff = mu_dry*scale;
  mu_cap      = mu_peak_eff;

  // μ(λ)=μp*(|λ|/λp)*exp(1 - |λ|/λp)
  mu = mu_peak_eff * (abs(slip)/max(lambda_peak,eps)) *
       exp(1 - abs(slip)/max(lambda_peak,eps));
end RoadFrictionSlipPeakStep;
