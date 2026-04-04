package com.pidmonitor;

public class Main {
    public static void main(String[] args) throws Exception {
        UARTSampler uart = new UARTSampler();
        if (!uart.connect()) {
            System.err.println("Arduino not found!");
            return;
        }

        PWMController pwm = new PWMController();
        pwm.enableControl(false); // keep PWM code, but disabled

        PIDAnalyzer analyzer = new PIDAnalyzer();

        while (true) {
            Frame f = uart.readSensorFrame();
            if (f != null) {
                System.out.println(f); // just print the incoming frame
                analyzer.addFrame(f);
            }

            uart.sendControlFrame(pwm); // just placeholder, no actual sending

            Thread.sleep(1); // adjust as needed
        }
    }
}