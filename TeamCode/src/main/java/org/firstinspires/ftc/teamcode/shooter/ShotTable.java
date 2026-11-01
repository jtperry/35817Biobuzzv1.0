package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.LinearTable;

/**
 * Shot tables keyed by distance (inches) from the turret to the aim point.
 * RPM and hood come from Shot Tuner; flight time from slow-mo video (spec 5.4).
 */
@Configurable
public final class ShotTable {

    /** Scales the whole RPM table. 0.95 = every shot 5% slower. */
    public static double RPM_SCALE = 1.0;

    // Physics fallback for flight time, used only while FLIGHT_S is empty.
    public static double BALL_EXIT_EFFICIENCY = 0.5;
    /** Flywheel center above the floor, inches (CAD). */
    public static double LAUNCH_HEIGHT_IN = 12.5;
    /** Middle of the up cell's opening, inches (manual figure 9-10). */
    public static double CELL_HEIGHT_IN = 59.5;
    public static final double FLYWHEEL_DIAMETER_IN = 72 / 25.4;
    public static final double GRAVITY_IN_PER_S2 = 386.09;

    /** Distance -> flywheel RPM. v1 values until re-tuned on the new robot. */
    public static final LinearTable RPM = new LinearTable()
            .put(20.7, 2900).put(34.9, 2950).put(42.5, 3000).put(51.3, 3000).put(55.1, 3200)
            .put(60.7, 3200).put(64.7, 3250).put(70.2, 3300).put(91.1, 3500);

    /** Distance -> hood servo position. v1 values until re-tuned on the new robot. */
    public static final LinearTable HOOD = new LinearTable()
            .put(20.7, 0.03).put(27.5, 0.15).put(34.9, 0.2).put(42.5, 0.22).put(51.3, 0.31)
            .put(55.1, 0.35).put(60.7, 0.41).put(64.7, 0.42).put(70.2, 0.45).put(91.1, 0.45);

    /** Distance -> ball flight time, seconds. Fill from slow-mo video; empty = physics fallback. */
    public static final LinearTable FLIGHT_S = new LinearTable();

    private ShotTable() {
    }

    public static double rpm(double distanceIn) {
        return RPM.get(distanceIn) * RPM_SCALE;
    }

    public static double hood(double distanceIn) {
        return HOOD.get(distanceIn);
    }

    public static double flightTimeS(double distanceIn) {
        if (!FLIGHT_S.isEmpty()) {
            return FLIGHT_S.get(distanceIn);
        }
        return physicsFlightTimeS(distanceIn, rpm(distanceIn));
    }

    /** v1 estimate: exit speed = flywheel surface speed x efficiency, on the low arc that reaches cell height. */
    static double physicsFlightTimeS(double distanceIn, double rpm) {
        double speed = BALL_EXIT_EFFICIENCY * Math.PI * FLYWHEEL_DIAMETER_IN * rpm / 60.0;
        if (speed <= 0 || distanceIn <= 0) {
            return 0;
        }
        double g = GRAVITY_IN_PER_S2;
        double rise = CELL_HEIGHT_IN - LAUNCH_HEIGHT_IN;
        double v2 = speed * speed;
        double root = v2 * v2 - g * (g * distanceIn * distanceIn + 2 * rise * v2);
        double angle = root < 0 ? Math.PI / 4 : Math.atan((v2 - Math.sqrt(root)) / (g * distanceIn));
        return distanceIn / (speed * Math.cos(angle));
    }
}
