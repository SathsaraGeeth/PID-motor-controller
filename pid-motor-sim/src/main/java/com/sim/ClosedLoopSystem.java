/**
 * System:
 * - closed loop system with a PID controller and a motor
 *
 * setpoint → PID → control signal (u) → Motor → velocity → feedback → PID
 */

package com.sim;

public class ClosedLoopSystem {
    private PIDController pid;
    private Motor motor;

    public ClosedLoopSystem(PIDController pid, Motor motor) {
        this.pid = pid;
        this.motor = motor;
    }

    public void update(double setpoint, double dt) {
        double velocity = motor.getVelocity();
        double controlSignal = pid.update(setpoint, velocity, dt);
        motor.update(controlSignal, dt);
    }
}