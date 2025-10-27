within ABS_PID_Model;
model PIDController
  "PID with event-free anti-windup (back-calculation)"
  import Modelica.Blocks.Interfaces.*;

  RealInput  slipRatio   "Measured slip";
  RealInput  targetSlip  "Desired slip";
  RealOutput brakeTorqueCmd "Controller output torque [N·m]";

  parameter Real Kp=40000;
  parameter Real Ki=5000;
  parameter Real Kd=200;
  parameter Real torqueMax=2500;
  parameter Real tau=0.02     "Derivative filter time [s]";
  parameter Real tau_aw=0.10  "Anti-windup back-calc time [s]";

protected
  Real e;
  Real e_f(start=0) "Filtered error";
  Real integral(start=0);
  Real dterm;
  Real rawOutput;
  Real saturatedOutput;

equation
  e        = targetSlip - slipRatio;
  der(e_f) = (e - e_f)/tau;
  dterm    = der(e_f);

  rawOutput        = Kp*e + Ki*integral + Kd*dterm;
  saturatedOutput  = noEvent(max(0, min(rawOutput, torqueMax)));
  der(integral)    = e + (saturatedOutput - rawOutput)/tau_aw;

  brakeTorqueCmd = saturatedOutput;
end PIDController;
