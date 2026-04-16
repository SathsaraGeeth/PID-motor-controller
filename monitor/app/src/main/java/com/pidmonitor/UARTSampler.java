package com.pidmonitor;

import com.fazecast.jSerialComm.SerialPort;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class UARTSampler {

    private SerialPort port;
    private InputStream in;
    private OutputStream out;

<<<<<<< Updated upstream
    private static final int FRAME_SIZE = 17;
    private static final byte DELIMITER = (byte) 0xAA;
    private static final byte CONTROL_DELIMITER = (byte) 0x77;
    private static final byte ACK = (byte) 0xFF;

    private final byte[] buffer = new byte[FRAME_SIZE];
    private int idx = 0;

    public UARTSampler() {}

=======
    public UARTSampler() {}

    // Connect to Arduino on Mac/Linux/Windows
>>>>>>> Stashed changes
    public boolean connect() {
        SerialPort[] ports = SerialPort.getCommPorts();

        System.out.println("Detected serial ports:");
        for (SerialPort p : ports) {
<<<<<<< Updated upstream
            System.out.println("  " + p.getSystemPortName());
=======
            System.out.println("Port: " + p.getSystemPortName() +
                               ", Description: " + p.getPortDescription() +
                               ", Open: " + p.isOpen());
>>>>>>> Stashed changes
        }

        for (SerialPort p : ports) {
            String name = p.getSystemPortName().toLowerCase();
<<<<<<< Updated upstream
            String desc = p.getDescriptivePortName().toLowerCase();

            if (name.contains("bluetooth") ||
                name.contains("debug") ||
                name.contains("wlan") ||
                name.contains("rfcomm")) continue;

            if (!(name.contains("usb") || desc.contains("usb") ||
                  name.contains("serial") || desc.contains("serial"))) {
                continue;
            }
=======

            // OS-agnostic detection
            boolean likelyArduino = name.contains("usbserial") || name.contains("ttyacm") || name.contains("ttyusb") || name.startsWith("cu.");
            if (!likelyArduino) continue;
>>>>>>> Stashed changes

            p.setBaudRate(230400);
            p.setNumDataBits(8);
            p.setParity(SerialPort.NO_PARITY);
            p.setNumStopBits(SerialPort.ONE_STOP_BIT);

<<<<<<< Updated upstream
            p.setComPortTimeouts(
                    SerialPort.TIMEOUT_READ_SEMI_BLOCKING,
                    200,
                    0
            );

            System.out.println("Trying port: " + p.getSystemPortName());

            if (p.openPort()) {
                port = p;
                in = port.getInputStream();
                out = port.getOutputStream();

=======
            System.out.println("Trying port: " + p.getSystemPortName());
            if (p.openPort()) {
                port = p;
                try {
                    in = port.getInputStream();
                    out = port.getOutputStream();
                } catch (Exception e) {
                    System.err.println("Failed to get streams: " + e.getMessage());
                    return false;
                }
>>>>>>> Stashed changes
                System.out.println("Connected to: " + port.getSystemPortName());
                return true;
            } else {
                System.out.println("Failed to open: " + p.getSystemPortName());
            }
        }

        System.err.println("Arduino not found on any serial port!");
        return false;
    }

    public void close() {
        if (port != null) port.closePort();
    }

<<<<<<< Updated upstream
    public Frame readSensorFrame() {
        try {
            while (true) {
                int b = in.read();
                if (b == -1) return null;

                buffer[idx++] = (byte) b;

                if (idx < FRAME_SIZE) continue;

                if (buffer[FRAME_SIZE - 1] == DELIMITER) {
                    Frame f = parseFrame(buffer);
                    idx = 0;
                    return f;
                } else {
                    System.arraycopy(buffer, 1, buffer, 0, FRAME_SIZE - 1);
                    idx = FRAME_SIZE - 1;
                }
            }
        } catch (IOException e) {
            return null;
        }
    }

    private Frame parseFrame(byte[] buf) {
        int index = 0;

=======
    // Read sensor frame from Arduino
    public Frame readSensorFrame() throws IOException {
        if (in == null) return null;

        byte[] buf = new byte[17]; // 4 + 6*2 + 1
        int read = 0;
        while (read < buf.length) {
            int r = in.read(buf, read, buf.length - read);
            if (r > 0) read += r;
        }

        int index = 0;
>>>>>>> Stashed changes
        long timestamp = ((buf[index++] & 0xFFL)) |
                         ((buf[index++] & 0xFFL) << 8) |
                         ((buf[index++] & 0xFFL) << 16) |
                         ((buf[index++] & 0xFFL) << 24);

        int setpoint = ((buf[index++] & 0xFF) | ((buf[index++] & 0xFF) << 8));
        int speed    = ((buf[index++] & 0xFF) | ((buf[index++] & 0xFF) << 8));
        int kp       = ((buf[index++] & 0xFF) | ((buf[index++] & 0xFF) << 8));
        int ki       = ((buf[index++] & 0xFF) | ((buf[index++] & 0xFF) << 8));
        int kd       = ((buf[index++] & 0xFF) | ((buf[index++] & 0xFF) << 8));
        int controlM = ((buf[index++] & 0xFF) | ((buf[index++] & 0xFF) << 8));

<<<<<<< Updated upstream
        return new Frame(timestamp, setpoint, speed, kp, ki, kd, controlM);
    }

    public boolean sendControlFrame(PWMController pwm) {
        if (out == null || in == null) return false;

        int value = pwm.getPWM();
        byte lo = (byte) (value & 0xFF);
        byte hi = (byte) ((value >> 8) & 0xFF);

        byte[] frame = new byte[] { lo, hi, CONTROL_DELIMITER };

        try {
            long start = System.currentTimeMillis();

            while (true) {
                out.write(frame);
                out.flush();

                long waitStart = System.currentTimeMillis();

                while (System.currentTimeMillis() - waitStart < 100) {
                    if (in.available() > 0) {
                        int b = in.read();
                        if (b == (ACK & 0xFF)) {
                            return true;
                        }
                    }
                }

                if (System.currentTimeMillis() - start > 1000) {
                    return false;
                }
            }
        } catch (IOException e) {
            return false;
        }
=======
        byte delimiter = buf[index];
        if ((delimiter & 0xFF) != 0xAA) {
            System.err.println("Frame error: wrong delimiter " + delimiter);
            return null;
        }

        return new Frame(timestamp, setpoint, speed, kp, ki, kd, controlM);
    }

    // Keep PWM control code, but temporarily disabled
    public boolean sendControlFrame(PWMController pwm) {
        if (!pwm.isControlEnabled()) return true; // do nothing if control disabled

        try {
            if (out == null) return false;
            int val = pwm.getPWM();
            byte[] buf = new byte[3];
            buf[0] = (byte)(val & 0xFF);
            buf[1] = (byte)((val >> 8) & 0xFF);
            buf[2] = 0x77; // delimiter

            out.write(buf);
            out.flush();

            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 100) {
                if (in.available() > 0) {
                    int ack = in.read();
                    if (ack == 0xFF) return true;
                }
            }
        } catch (IOException e) {
            System.err.println("Error sending control frame: " + e.getMessage());
        }
        return false;
>>>>>>> Stashed changes
    }
}