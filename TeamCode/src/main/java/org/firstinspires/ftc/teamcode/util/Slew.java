package org.firstinspires.ftc.teamcode.util;

/**
 * A value that moves toward a target at a limited rate. Used for the turret and hood "where is the servo
 * now" estimates and for the flower arm's slow move.
 */
public final class Slew {

    /** Longest time one step may cover, so a slow loop never makes a big jump. */
    private static final double MAX_STEP_S = 0.1;

    private double value;
    private long lastNanos = 0;
    private boolean hasTime = false;

    public Slew(double initial) {
        value = initial;
    }

    /** Move toward target by at most ratePerSec x elapsed time. The first step after creation or reset() does not move. */
    public double step(double target, double ratePerSec, long nowNanos) {
        double dt = hasTime ? Math.min((nowNanos - lastNanos) / 1e9, MAX_STEP_S) : 0;
        lastNanos = nowNanos;
        hasTime = true;
        double maxStep = ratePerSec * dt;
        value += MathUtil.clamp(target - value, -maxStep, maxStep);
        return value;
    }

    /** Jump to a value and restart the clock. */
    public void reset(double newValue) {
        value = newValue;
        hasTime = false;
    }

    public double value() {
        return value;
    }
}
