package com.pidmonitor;

public class Frame {
    public long timestamp;
    public int setpoint, speed, kp, ki, kd, control_m;

    public Frame(long timestamp, int setpoint, int speed, int kp, int ki, int kd, int control_m) {
        this.timestamp = timestamp;
        this.setpoint = setpoint;
        this.speed = speed;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.control_m = control_m;
    }

    @Override
    public String toString() {
        return timestamp + "," + setpoint + "," + speed + "," + kp + "," + ki + "," + kd + "," + control_m;
    }
}