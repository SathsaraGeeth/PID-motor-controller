A PID motor controller.

Two parts.

I. Simulation of a PID controller.
pid-motor-sim/

II. Circuit Implementation.
1. Cicuit Simulation.
analog-circuit/sim
- current circuit is adapted from https://github.com/vscheyer/Analog_PID_MC.git.


2. Hardware Implementation

2.1. Analog Circuit
- Motor with speed sensor
- Motor driver
- Adder (summing P, I, D signals)
- Scalar multiplier (gain adjustment)
- Integrator
- Differentiator
- Power supply

2.2. Sampler (Arduino)
- Sampling parameters:
  i. Setpoint
  ii. Process variable: speed
  iii. Control signal (P, I, D, after sum)
  iv. PID gains (Kp, Ki, Kd)
- Fixed-rate sampling over UART

2.3. Application Layer (Java GUI)
- Real-time data plotting
- Logging for post-analysis
- System identification
- Model-based analysis and prediction
- PID estimation (Ziegler–Nichols / Cohen–Coon)
- Intelli mode: outputs PWM control signal computed by the model