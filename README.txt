A PID motor controller.

Two parts.

I. Simulation of a PID controller.
pid-motor-sim/

II. Circuit Implementation.
1. Cicuit Simulation.
analog-circuit/sim
- current circuit is adapted from https://github.com/vscheyer/Analog_PID_MC.git.

2. Breadboard Prototype.

2.1. Analog circuit
- motor + position sensor + speed sensor
- motor driver
- adder
- scalar multiplier
- integrator
- differentiator
- power

2.2. Sampler
- sampling params
i. setpoint
ii. process variables (speed and position)
iii. process vaiable control choose toggle
iv. control signal (P, I, D, and after add)
v. PID params
- fixed rate over uart
- frame_format = [timestamp, mode, setpoint, speed, position, kp, kd, ki, control signal, delimeter]
timestamp - 32bits
mode - 8bits
else - 16bits
delimieter - 8bits 10101010
2.3. Applicatin layer
- plot data
- log
- system identification
- model based analysis and prediciton
- pid estimation (Ziegler–Nichols/Cohen–Coon)
