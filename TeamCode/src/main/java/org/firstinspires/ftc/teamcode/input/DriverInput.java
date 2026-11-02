package org.firstinspires.ftc.teamcode.input;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Stick shaping for driving (spec 6.1): a radial deadzone rescaled from its edge, plus an optional curve.
 * Pedro's driveOrHold treats input over 0.1 as driving, so stick drift above that would stop it from holding.
 */
@Configurable
public final class DriverInput {

    public static double DEADZONE = 0.08;
    public static double TURN_DEADZONE = 0.08;

    /** 1 = linear, 2 = squared (finer control near the center). */
    public static double CURVE = 1.0;

    /**
     * Deadzones are used clamped to [0, MAX_DEADZONE]: with a Panels value of 1 or more the rescale would divide by
     * zero or by a negative number, and with one below 0 a stick at rest would move the robot.
     */
    private static final double MAX_DEADZONE = 0.95;

    private DriverInput() {
    }

    /** Multiply both stick axes by this. 0 inside the deadzone; result magnitude 0..1 outside. */
    public static double stickScale(double x, double y) {
        return stickScale(x, y, DEADZONE, CURVE);
    }

    public static double stickScale(double x, double y, double deadzone, double curve) {
        double zone = MathUtil.clamp(deadzone, 0, MAX_DEADZONE);
        double magnitude = Math.hypot(x, y);
        if (magnitude <= zone || magnitude == 0) {
            return 0;
        }
        double shaped = Math.pow((Math.min(magnitude, 1.0) - zone) / (1.0 - zone), curve);
        return shaped / magnitude;
    }

    /** Turn stick with its own deadzone, same curve. */
    public static double turn(double value) {
        return axis(value, TURN_DEADZONE, CURVE);
    }

    public static double axis(double value, double deadzone, double curve) {
        double zone = MathUtil.clamp(deadzone, 0, MAX_DEADZONE);
        double magnitude = Math.abs(value);
        if (magnitude <= zone) {
            return 0;
        }
        double shaped = Math.pow((Math.min(magnitude, 1.0) - zone) / (1.0 - zone), curve);
        return Math.copySign(shaped, value);
    }
}
