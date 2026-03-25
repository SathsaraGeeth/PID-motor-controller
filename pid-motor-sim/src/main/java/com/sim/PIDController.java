/**
 * In simple terms PID do this.
 * 1. At every time step.
 *      - measure the motor velocity.
 *      - compare it with the desired speed (setpoint).
 *      - error(e) = setpoint - velocity.
 * 2. PID computes the cotrol signal u.
 * u = K_p * e + K_i * time_integral(e) + K_d * time_derivative(e);
 * u = P + I + D;
 * 
 * Here:
 * 1. Proportional(P) - reacts to current error. (how far away from the target velocty?)
 * 2. Integral(I) - accumulates past error. (have been missing the target over time?)
 * 3. Derivative(D) - reacts to rate of change. (approaching too fast?)
 */

package com.sim;

public class PIDController {
    private double kp;
    private double ki;
    private double kd;

    // init conditions
    private double integral = 0.0;
    private double previousError = 0.0;

    // to saturate
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    public PIDController(double kp, double ki, double kd, double minOutput, double maxOutput) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public double update(double setpoint, double measured, double dt) {
        double error = setpoint - measured;

        double P = kp * error;

        double derivative = (error - previousError) / dt;
        double D = kd * derivative;
        previousError = error;

        double newIntegral = integral + error * dt;
        double I = ki * newIntegral;

        double output = P + I + D;
        double clampedOutput = Math.max(minOutput, Math.min(maxOutput, output));

        // anti windup
        if (output > minOutput && output < maxOutput) {
            integral = newIntegral;
        }

        return clampedOutput;
    }
}