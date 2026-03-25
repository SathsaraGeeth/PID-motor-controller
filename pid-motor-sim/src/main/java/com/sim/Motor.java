/**
 * Simplified model of a DC motor.
 * 
 * In a DC motor.
 * -> when voltage is applied, a torque is produced.
 * -> the torque results in a acceleration.
 * -> inturn acceleration makes velocity increase.
 * -> voltage integrate over time.
 * 
 * Linear Model:
 * velocity += input;
 * 
 * A More Realistic Model:
 * v(t + dt) = v(t) + (u - b * v) * dt
 * where v = velocity, u = control input, b = damping (friction coeff.)
 * we have implement this model here.
*/

package com.sim;

/**
 * v(t + dt) = v(t) + (u - b * v) * dt
 * -> v(t + dt) - v(t) = (u - b * v) * dt
 */
public class Motor {
    // default values
    private double v = 0.0; 
    private double b  = 0.1;

    public Motor (double initialVelocity, double dampingFactor) {
        this.v = initialVelocity;
        this.b = dampingFactor;
    }

    public void update(double u, double dt) {
        v += (u - b * v) * dt;
    }
    public double getVelocity() {
        return v;
    }
}
