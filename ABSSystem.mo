within ABS_PID_Model;
model ABSSystem
  "Single-wheel ABS with slip-peak tire and constant peak torque cap"
  import Modelica.Constants.g_n;

  // Components
  ABS_PID_Model.VehicleBody          vehicleBody(m=300, v0=30);
  ABS_PID_Model.Wheel                wheel(J=1.2, Rw=0.3, v_init=30);
  ABS_PID_Model.PIDController        pid(Kp=30000, Ki=5000, Kd=200, torqueMax=2500, tau=0.02, tau_aw=0.10);
  ABS_PID_Model.RoadFrictionSlipPeak road(mu_peak=0.8, lambda_peak=0.2);
  ABS_PID_Model.DriverPedalRamp      pedal(height=1, duration=0.3, startTime=0.2);
  ABS_PID_Model.BrakeActuator        actuator(maxTorque=2500, tau=0.03);

  // Exposed (optional) outputs
  Modelica.Blocks.Interfaces.RealOutput Fx "Longitudinal tire force [N]";
  Modelica.Blocks.Interfaces.RealOutput T_b "Applied brake torque [N·m]";

  // Params
  parameter Real lambda_ref = 0.2 "Reference slip";
  parameter Real Fz = vehicleBody.m * g_n / 4 "Normal load [N]";
  parameter Real v_eps = 0.05 "Smoothing speed for tanh [m/s]";
  parameter Real v_stop = 0.5 "Disable braking below this speed [m/s]";
  parameter Real maxTotalTorque = 4000 "Absolute cap [N·m]";

  // Internals
  Real mu "μ from slip-peak tire";
  Real wheelTangentialSpeed "Wheel surface speed [m/s]";
  Real lambda "Slip ratio";
  Real slipError "λ_ref - λ";
  Real Troad_max "Peak-based torque limit [N·m]";
  Real T_b_raw "Torque after limiter [N·m]";

equation
  // Wiring
  connect(vehicleBody.vehicleSpeed, wheel.vehicleSpeed);
  connect(pid.slipRatio,            wheel.slipRatio);
  pid.targetSlip = lambda_ref;
  connect(pedal.cmd, actuator.pedal);
  connect(wheel.slipRatio, road.slip);

  // Tire μ(λ)
  mu = road.mu;

  // Helper signals
  wheelTangentialSpeed = wheel.wheelSpeed * wheel.Rw;
  lambda    = wheel.slipRatio;
  slipError = lambda_ref - lambda;

  // Smoothed friction (opposes slip v - Rw*ω); uses TRUE μ(λ)
  Fx = mu * Fz * tanh((vehicleBody.vehicleSpeed - wheelTangentialSpeed)/v_eps);

  // Constant torque cap at tire peak (no μ(λ) feedback here)
  Troad_max = wheel.Rw * road.mu_peak * Fz;

  // Event-free limiter against actuator + PID authority
  T_b_raw = noEvent(min(max(actuator.torque + pid.brakeTorqueCmd, 0),
                        min(Troad_max, maxTotalTorque)));

  // Disable braking near standstill
  T_b = if vehicleBody.vehicleSpeed < v_stop then 0 else T_b_raw;

  // Apply to wheel/body
  connect(Fx,  wheel.frictionForce);
  connect(Fx,  vehicleBody.frictionForce);
  connect(T_b, wheel.brakeTorque);

annotation (experiment(StartTime=0, StopTime=5, Tolerance=1e-6, Interval=0.002));
end ABSSystem;
