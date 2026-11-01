package org.firstinspires.ftc.teamcode.shooter;

/** One loop's shot. ShotSolver reuses one instance (no allocation per loop). */
public final class ShotSolution {

    /** Turret angle, robot frame, degrees: 0 = forward, positive = left. Wrapped to [-180, 180). */
    public double turretDeg;

    /** How fast turretDeg is changing, degrees per second (turret lag lead). */
    public double turretRateDegPerS;

    /** Turret position at release to the aim point, inches. Drives the RPM and hood tables. */
    public double distanceIn;

    /** Aim point (cell minus lead), field inches. */
    public double aimX;
    public double aimY;

    /** Turret position on the field at ball release, inches. */
    public double releaseX;
    public double releaseY;

    /** Predicted robot speed when the ball leaves, in/s. 0 when standing still or stopping first. */
    public double releaseSpeed;
}
