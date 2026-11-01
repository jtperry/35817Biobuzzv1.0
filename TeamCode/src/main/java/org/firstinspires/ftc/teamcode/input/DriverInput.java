package org.firstinspires.ftc.teamcode.input;

import com.bylazar.configurables.annotations.Configurable;

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

    private DriverInput() {
    }

    /** Multiply both stick axes by this. 0 inside the deadzone; result magnitude 0..1 outside. */
    public static double stickScale(double x, double y) {
        return stickScale(x, y, DEADZONE, CURVE);
    }

    public static double stickScale(double x, double y, double deadzone, double curve) {
        double magnitude = Math.hypot(x, y);
        if (magnitude <= deadzone || magnitude == 0) {
            return 0;
        }
        double shaped = Math.pow((Math.min(magnitude, 1.0) - deadzone) / (1.0 - deadzone), curve);
        return shaped / magnitude;
    }

    /** Turn stick with its own deadzone, same curve. */
    public static double turn(double value) {
        return axis(value, TURN_DEADZONE, CURVE);
    }

    public static double axis(double value, double deadzone, double curve) {
        double magnitude = Math.abs(value);
        if (magnitude <= deadzone) {
            return 0;
        }
        double shaped = Math.pow((Math.min(magnitude, 1.0) - deadzone) / (1.0 - deadzone), curve);
        return Math.copySign(shaped, value);
    }
}
