package com.pidmonitor;

import java.util.ArrayList;
import java.util.List;

public class PIDAnalyzer {
    private List<Frame> frames = new ArrayList<>();

    public void addFrame(Frame f) {
        frames.add(f);
    }

    public double computeAverageSpeed() {
        return frames.stream().mapToInt(f -> f.speed).average().orElse(0.0);
    }

    public int getLastControlM() {
        if (frames.isEmpty()) return 0;
        return frames.get(frames.size() - 1).control_m;
    }
}