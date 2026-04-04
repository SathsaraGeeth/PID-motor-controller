package com.pidmonitor;

public class PWMController {
    private int pwmValue = 0;
    private boolean controlEnabled = false;

    public synchronized void setPWM(int pwm) {
        pwmValue = Math.max(0, Math.min(1023, pwm));
    }

    public synchronized int getPWM() {
        return pwmValue;
    }

    public synchronized void enableControl(boolean enable) {
        controlEnabled = enable;
    }

    public synchronized boolean isControlEnabled() {
        return controlEnabled;
    }
}