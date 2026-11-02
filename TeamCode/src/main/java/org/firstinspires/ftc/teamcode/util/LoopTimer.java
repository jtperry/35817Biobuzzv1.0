package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import java.util.Locale;

/**
 * Loop-time stats over a sliding window: mean, 95th percentile and max, in milliseconds.
 * tick() every loop (cheap); compute() only when the numbers are shown.
 */
public final class LoopTimer {

    private static final int CAPACITY = 2048;

    private final long windowNanos;
    private final long[] durations = new long[CAPACITY];
    private final long[] stamps = new long[CAPACITY];
    private final double[] scratch = new double[CAPACITY];
    private int head = 0;
    private int size = 0;
    private long lastTick = -1;

    private double meanMs = 0;
    private double p95Ms = 0;
    private double maxMs = 0;
    private int count = 0;

    public LoopTimer(double windowSeconds) {
        this.windowNanos = (long) (windowSeconds * 1e9);
    }

    /** Call once per loop with System.nanoTime(). */
    public void tick(long nowNanos) {
        if (lastTick >= 0) {
            durations[head] = nowNanos - lastTick;
            stamps[head] = nowNanos;
            head = (head + 1) % CAPACITY;
            if (size < CAPACITY) {
                size++;
            }
        }
        lastTick = nowNanos;
    }

    /** Recompute the stats from loops that ended inside the window. */
    public void compute() {
        count = 0;
        double sum = 0;
        double max = 0;
        for (int i = 0; i < size; i++) {
            int index = (head - 1 - i + CAPACITY) % CAPACITY;
            if (lastTick - stamps[index] > windowNanos) {
                break;
            }
            double ms = durations[index] / 1e6;
            scratch[count++] = ms;
            sum += ms;
            max = Math.max(max, ms);
        }
        if (count == 0) {
            meanMs = 0;
            p95Ms = 0;
            maxMs = 0;
            return;
        }
        Arrays.sort(scratch, 0, count);
        meanMs = sum / count;
        maxMs = max;
        p95Ms = scratch[(int) Math.ceil(0.95 * count) - 1];
    }

    public double meanMs() {
        return meanMs;
    }

    public double p95Ms() {
        return p95Ms;
    }

    public double maxMs() {
        return maxMs;
    }

    public int count() {
        return count;
    }

    /** "mean 12.3 / p95 15.1 / max 22.0 ms (180 loops)" from the last compute(); always a dot (Locale.US). */
    public String summary() {
        return String.format(Locale.US, "mean %.1f / p95 %.1f / max %.1f ms (%d loops)", meanMs, p95Ms, maxMs, count);
    }
}
