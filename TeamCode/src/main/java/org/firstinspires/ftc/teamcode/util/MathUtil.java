package org.firstinspires.ftc.teamcode.util;

/** Small math helpers shared by every subsystem. */
public final class MathUtil {

    private MathUtil() {
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Motor power safe to send: clamped to [-1, 1], NaN becomes 0 (NextFTC throws outside [-1, 1]). */
    public static double safePower(double power) {
        return Double.isNaN(power) ? 0.0 : clamp(power, -1.0, 1.0);
    }

    /** Wrap radians to [-PI, PI). */
    public static double wrapRadians(double radians) {
        double wrapped = (radians + Math.PI) % (2 * Math.PI);
        if (wrapped < 0) {
            wrapped += 2 * Math.PI;
        }
        return wrapped - Math.PI;
    }

    /** Wrap degrees to [-180, 180). */
    public static double wrapDegrees(double degrees) {
        double wrapped = (degrees + 180) % 360;
        if (wrapped < 0) {
            wrapped += 360;
        }
        return wrapped - 180;
    }
}
