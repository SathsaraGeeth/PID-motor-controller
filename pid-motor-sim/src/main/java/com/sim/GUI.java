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

    private JLabel kpLabel = new JLabel();
    private JLabel kiLabel = new JLabel();
    private JLabel kdLabel = new JLabel();
    private JLabel spLabel = new JLabel();
    private JLabel dampingLabel = new JLabel();

    public App() {
        setTitle("PID Motor Controller Simulation");
        setSize(1000, 700);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setLayout(new BorderLayout());

        initSimulation();

        add(buildTopPanel(), BorderLayout.NORTH);
        add(buildControlPanel(), BorderLayout.WEST);
        add(buildPlotWrapper(), BorderLayout.CENTER);
        add(buildButtons(), BorderLayout.SOUTH);

        startTimer();

        setVisible(true);
    }

    private void initSimulation() {
        pid = new PIDController(kp, ki, kd, minOutput, maxOutput);
        motor = new Motor(0.0, damping);
        system = new ClosedLoopSystem(pid, motor);
    }

    private JPanel buildTopPanel() {
        JPanel panel = new JPanel(new GridLayout(1, 5));
        panel.setBorder(BorderFactory.createTitledBorder("Current Parameters"));

        panel.add(kpLabel);
        panel.add(kiLabel);
        panel.add(kdLabel);
        panel.add(spLabel);
        panel.add(dampingLabel);

        updateLabels();

        return panel;
    }

    private void updateLabels() {
        kpLabel.setText("Kp: " + String.format("%.3f", kp));
        kiLabel.setText("Ki: " + String.format("%.3f", ki));
        kdLabel.setText("Kd: " + String.format("%.3f", kd));
        spLabel.setText("Setpoint: " + String.format("%.3f", setpoint));
        dampingLabel.setText("Damping: " + String.format("%.3f", damping));
    }

    private JPanel buildControlPanel() {
        JPanel panel = new JPanel(new GridLayout(0, 1));

        panel.add(createSlider("Kp", 0, 200, (int)(kp * 100), val -> kp = val / 100.0));
        panel.add(createSlider("Ki", 0, 200, (int)(ki * 100), val -> ki = val / 100.0));
        panel.add(createSlider("Kd", 0, 200, (int)(kd * 100), val -> kd = val / 100.0));
        panel.add(createSlider("Setpoint", 0, 200, (int)(setpoint * 100), val -> setpoint = val / 100.0));
        panel.add(createSlider("Damping", 0, 100, (int)(damping * 100), val -> damping = val / 100.0));

        panel.setPreferredSize(new Dimension(250, 0));

        return panel;
    }

    private JSlider createSlider(String name, int min, int max, int value, SliderCallback callback) {
        JSlider slider = new JSlider(min, max, value);
        slider.setBorder(BorderFactory.createTitledBorder(name));

        slider.addChangeListener(e -> {
            callback.onChange(slider.getValue());
            updateLabels();
        });

        return slider;
    }

    private interface SliderCallback {
        void onChange(int value);
    }

    private JPanel buildPlotWrapper() {
        JPanel wrapper = new JPanel(new FlowLayout(FlowLayout.CENTER));
        plotPanel.setPreferredSize(new Dimension(600, 400));
        plotPanel.setBorder(BorderFactory.createTitledBorder(""));
        wrapper.add(plotPanel);
        return wrapper;
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

    private void startTimer() {
        Timer timer = new Timer((int)(dt * 1000), e -> {
            if (!running) return;

            motor.setDamping(damping);

            pid = new PIDController(kp, ki, kd, minOutput, maxOutput);
            system = new ClosedLoopSystem(pid, motor);

            system.update(setpoint, dt);

            velocityHistory.add(motor.getVelocity());

            if (velocityHistory.size() > 500) {
                velocityHistory.remove(0);
            }

            plotPanel.setData(velocityHistory);
        });

        timer.start();
    }

    static class PlotPanel extends JPanel {

        private List<Double> data = new ArrayList<>();

        private final double yMin = -2.0;
        private final double yMax = 2.0;
        private final int maxPoints = 500;

        public PlotPanel() {
            setDoubleBuffered(true);
        }

        public void setData(List<Double> data) {
            this.data = new ArrayList<>(data);
            repaint();
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);

            Graphics2D g2 = (Graphics2D) g.create();

            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            int w = getWidth();
            int h = getHeight();

            g2.setColor(Color.WHITE);
            g2.fillRect(0, 0, w, h);

            int yGridLines = 20;
            int xGridLines = 20;

            g2.setColor(Color.LIGHT_GRAY);

            for (int i = 0; i <= yGridLines; i++) {
                int y = i * h / yGridLines;
                g2.drawLine(0, y, w, y);

                double yValue = yMax - (i * (yMax - yMin) / yGridLines);
                g2.setColor(Color.DARK_GRAY);
                g2.drawString(String.format("%.2f", yValue), 5, y - 2);
                g2.setColor(Color.LIGHT_GRAY);
            }

            int n = data.size();
            if (n == 0) {
                g2.dispose();
                return;
            }

            int visiblePoints = Math.min(n, maxPoints);
            int startIndex = n - visiblePoints;

            for (int i = 0; i <= xGridLines; i++) {
                int x = i * w / xGridLines;
                g2.drawLine(x, 0, x, h);

                int dataIndex = startIndex + (i * visiblePoints / xGridLines);
                g2.setColor(Color.DARK_GRAY);
                g2.drawString(String.valueOf(dataIndex), x + 2, h - 5);
                g2.setColor(Color.LIGHT_GRAY);
            }

            g2.setColor(Color.BLACK);

            for (int i = startIndex + 1; i < n; i++) {
                int idx = i - startIndex;

                int x1 = (idx - 1) * w / visiblePoints;
                int x2 = idx * w / visiblePoints;

                double v1 = data.get(i - 1);
                double v2 = data.get(i);

                int y1 = (int) (h - ((v1 - yMin) / (yMax - yMin)) * h);
                int y2 = (int) (h - ((v2 - yMin) / (yMax - yMin)) * h);

                g2.drawLine(x1, y1, x2, y2);
            }

            g2.setColor(Color.BLACK);
            g2.drawString("Velocity", 10, 15);
            g2.drawString("Time", w - 140, h - 10);

            g2.dispose();
        }
    }
}