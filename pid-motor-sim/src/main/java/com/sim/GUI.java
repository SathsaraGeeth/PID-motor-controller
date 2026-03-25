package com.sim;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class GUI {

    public static void main(String[] args) {
        SwingUtilities.invokeLater(App::new);
    }
}

class App extends JFrame {
    // default parameters
    private double kp = 0.8;
    private double ki = 0.2;
    private double kd = 0.05;

    private double damping = 0.1;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double setpoint = 1.0;
    private double dt = 0.01;

    private PIDController pid;
    private Motor motor;
    private ClosedLoopSystem system;


    private boolean running = false;
    private final List<Double> velocityHistory = new ArrayList<>();

    private final PlotPanel plotPanel = new PlotPanel();

    public App() {
        setTitle("PID Motor Controller Simulation");
        setSize(900, 600);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setLayout(new BorderLayout());

        initSimulation();

        add(buildControlPanel(), BorderLayout.WEST);
        add(plotPanel, BorderLayout.CENTER);

        add(buildButtons(), BorderLayout.SOUTH);

        setVisible(true);
    }

    private void initSimulation() {
        pid = new PIDController(kp, ki, kd, minOutput, maxOutput);
        motor = new Motor(0.0, damping);
        system = new ClosedLoopSystem(pid, motor);
    }

    private JPanel buildControlPanel() {
        JPanel panel = new JPanel();
        panel.setLayout(new GridLayout(0, 1));

        panel.add(createSlider("Kp", 0, 200, (int)(kp * 100), val -> kp = val / 100.0));
        panel.add(createSlider("Ki", 0, 200, (int)(ki * 100), val -> ki = val / 100.0));
        panel.add(createSlider("Kd", 0, 200, (int)(kd * 100), val -> kd = val / 100.0));

        panel.add(createSlider("Setpoint", 0, 200, (int)(setpoint * 100), val -> setpoint = val / 100.0));
        panel.add(createSlider("Damping", 0, 100, (int)(damping * 100), val -> damping = val / 100.0));

        return panel;
    }

    private JPanel buildButtons() {
        JPanel panel = new JPanel();

        JButton startBtn = new JButton("Start");
        JButton stopBtn = new JButton("Stop");
        JButton resetBtn = new JButton("Reset");

        startBtn.addActionListener(e -> running = true);
        stopBtn.addActionListener(e -> running = false);

        resetBtn.addActionListener(e -> {
            running = false;
            velocityHistory.clear();
            initSimulation();
            plotPanel.repaint();
        });

        panel.add(startBtn);
        panel.add(stopBtn);
        panel.add(resetBtn);

        return panel;
    }

    private JSlider createSlider(String name, int min, int max, int value, SliderCallback callback) {
        JSlider slider = new JSlider(min, max, value);
        slider.setBorder(BorderFactory.createTitledBorder(name));
        slider.addChangeListener(e -> callback.onChange(slider.getValue()));
        return slider;
    }

    private interface SliderCallback {
        void onChange(int value);
    }

    // Simulation
    {
        Timer timer = new Timer((int)(dt * 1000), e -> {
            if (running) {
                pid = new PIDController(kp, ki, kd, minOutput, maxOutput);
                system = new ClosedLoopSystem(pid, motor);

                system.update(setpoint, dt);

                velocityHistory.add(motor.getVelocity());

                if (velocityHistory.size() > 500) {
                    velocityHistory.remove(0);
                }

                plotPanel.setData(velocityHistory);
            }
        });
        timer.start();
    }

    // plot
    static class PlotPanel extends JPanel {
        private List<Double> data = new ArrayList<>();

        public void setData(List<Double> data) {
            this.data = new ArrayList<>(data);
            repaint();
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);

            if (data.isEmpty()) return;

            int w = getWidth();
            int h = getHeight();
            double max = data.stream().mapToDouble(v -> v).max().orElse(1.0);
            double min = data.stream().mapToDouble(v -> v).min().orElse(0.0);
            int n = data.size();

            for (int i = 1; i < n; i++) {
                int x1 = (i - 1) * w / n;
                int x2 = i * w / n;
                int y1 = (int) (h - ((data.get(i - 1) - min) / (max - min + 1e-6)) * h);
                int y2 = (int) (h - ((data.get(i) - min) / (max - min + 1e-6)) * h);
                g.drawLine(x1, y1, x2, y2);
            }
        }
    }
}