within;
package ABS_PID_Model
annotation(
  version="0.1.0",
  uses(Modelica(version="4.0.0")),
  Documentation(info="
  <html>
  <h2>🛞 ABS_PID_Model – Single-Wheel ABS Simulation</h2>
  <p>This library implements a <b>single-wheel Anti-lock Braking System (ABS)</b> using a PID-based slip control logic. 
  The model demonstrates how wheel slip can be maintained near the optimal value (~0.2) for maximum friction on both constant and variable road surfaces.</p>

  <h3>🚗 Overview</h3>
  <p>The system models the following subsystems:</p>
  <ul>
    <li><b>VehicleBody:</b> Longitudinal quarter-car model providing vehicle speed.</li>
    <li><b>Wheel:</b> Rotational dynamics and slip computation.</li>
    <li><b>PIDController:</b> Regulates slip ratio by adjusting brake torque.</li>
    <li><b>BrakeActuator:</b> Simulates first-order brake response.</li>
    <li><b>DriverPedalRamp:</b> Provides smooth pedal input.</li>
    <li><b>RoadFriction models:</b> Define tire-road interaction under constant or variable friction.</li>
  </ul>

  <h3>🧩 Model Summary</h3>
  <table border='1' cellspacing='0' cellpadding='4'>
  <tr><th>Model</th><th>Description / Function</th><th>Key Assumptions</th></tr>
  <tr><td><b>VehicleBody.mo</b></td><td>Quarter-vehicle longitudinal model; computes vehicle velocity under braking.</td><td>No drag, slope, or tire compliance; constant mass.</td></tr>
  <tr><td><b>Wheel.mo</b></td><td>Rotational wheel model; computes angular velocity and slip ratio.</td><td>Rigid tire, constant normal load, no lateral slip.</td></tr>
  <tr><td><b>PIDController.mo</b></td><td>PID-based slip controller with anti-windup and torque saturation.</td><td>Continuous-time ideal control; perfect sensing.</td></tr>
  <tr><td><b>BrakeActuator.mo</b></td><td>First-order actuator converting pedal input to torque.</td><td>Linear hydraulic response; no hysteresis.</td></tr>
  <tr><td><b>DriverPedalRamp.mo</b></td><td>Driver brake input ramp from 0→1 (normalized).</td><td>Smooth pedal press; no release phase.</td></tr>
  <tr><td><b>RoadFrictionSlipPeak.mo</b></td><td>Slip-dependent μ(λ) model with single peak.</td><td>Constant μ<sub>peak</sub> and λ<sub>peak</sub>; no temperature or load effects.</td></tr>
  <tr><td><b>RoadFrictionSlipPeakStep.mo</b></td><td>Same as above but μ<sub>peak</sub> drops at a defined time (dry→wet).</td><td>Uniform friction change; smooth transition.</td></tr>
  <tr><td><b>ABSSystem.mo</b></td><td>Integrates all subsystems with constant μ road.</td><td>Single wheel; constant road friction.</td></tr>
  <tr><td><b>ABSSystem_MuStep.mo</b></td><td>ABS with dynamic μ step (dry→wet) using adaptive torque capping.</td><td>μ changes smoothly at 2 s.</td></tr>
  <tr><td><b>ABSSystem_Block.mo</b></td><td>Visual version with diagram annotations for OMEdit.</td><td>Layout only; same equations.</td></tr>
  </table>

  <h3>⚙️ Global Assumptions</h3>
  <ul>
    <li>Single-wheel (¼ vehicle) representation.</li>
    <li>Longitudinal motion only; no lateral or pitch effects.</li>
    <li>Constant normal load F<sub>z</sub> = m·g/4.</li>
    <li>μ depends only on slip ratio; no temperature or velocity dependence.</li>
    <li>Brake actuator is linear, first-order; no hysteresis.</li>
    <li>Controller is continuous; sensors are ideal (no noise or delay).</li>
    <li>Flat terrain; no aerodynamic drag or slope forces.</li>
  </ul>

  <h3>🧠 Simulation Scenarios</h3>
  <ul>
    <li><b>ABSSystem:</b> Constant dry road (μ=0.8).</li>
    <li><b>ABSSystem_MuStep:</b> Dry→wet transition (μ: 0.8 → 0.4 at 2 s).</li>
  </ul>

  <h3>✅ Expected Results</h3>
  <p>Slip ratio stabilizes between <b>0.15–0.20</b>, with brake torque adapting dynamically to road friction. 
  Wheel never locks, and vehicle deceleration remains smooth through friction changes.</p>

  <h3>📦 Usage</h3>
  <ol>
    <li>Load this library in OMEdit (File → Open Model/Library).</li>
    <li>Simulate <b>ABS_PID_Model.ABSSystem</b> or <b>ABS_PID_Model.ABSSystem_MuStep</b>.</li>
    <li>Plot: <code>wheel.slipRatio</code>, <code>mu</code>, <code>T_b</code>, <code>Troad_max</code>, and <code>vehicleBody.vehicleSpeed</code>.</li>
  </ol>

  <h3>📜 License</h3>
  <p>Released under the MIT License. Suitable for educational and research use.</p>
  </html>"));
end ABS_PID_Model;
