package com.sim;

public class Main {
    public static void main(String[] args) {
        double kp = 0.8;
        double ki = 0.2;
        double kd = 0.05;
        double initialVelocity = 0.0;
        double dampingFactor = 0.1;
        double minOutput = -1.0;
        double maxOutput = 1.0;
        double dt = 0.01;
        double setpoint = 1.0;

        PIDController pid = new PIDController(kp, ki, kd, minOutput, maxOutput);
        Motor motor = new Motor(initialVelocity, dampingFactor);
        ClosedLoopSystem system = new ClosedLoopSystem(pid, motor);

        // Simulate for 1000 steps
        for (int i = 0; i < 1000; i++) {
            system.update(setpoint, dt);
            System.out.printf("Time: %.2f s, Velocity: %.4f m/s%n", i * dt, motor.getVelocity());
        }
    }
}