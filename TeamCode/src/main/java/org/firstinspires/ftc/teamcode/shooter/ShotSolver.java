package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.function.DoubleUnaryOperator;

/**
 * Shot math, no hardware (spec 5.5). Each loop: robot state in, ShotSolution out.
 * A moving shot leads the cell by the velocity the ball will carry when it leaves the robot:
 * acceleration is estimated from loop-to-loop velocity, speed at release is predicted
 * RELEASE_LATENCY_S ahead, and braking can shrink the lead to zero but never past a stop.
 */
@Configurable
public final class ShotSolver {

    public static boolean SHOOT_ON_MOVE = true;

    /** Aim to ball leaving the robot, seconds (gate delay + ball travel). */
    public static double RELEASE_LATENCY_S = 0.15;

    /** Lead multiplier. Near 1.0 once flight times are measured. */
    public static double LEAD_GAIN = 1.0;

    /** Weight of the newest acceleration sample, 0-1. Lower = smoother but slower. */
    public static double ACCEL_SMOOTHING = 0.3;

    /** Acceleration estimates are clamped to this, in/s^2. */
    public static double MAX_ACCEL_IN_S2 = 250;

    /** Slower than this counts as standing still: no lead. */
    public static double MIN_SPEED_IN_S = 3;

    /** Weight of the newest turret-rate sample, 0-1. */
    public static double RATE_SMOOTHING = 0.3;

    /** Turret axis relative to the robot center, inches, robot frame (CAD, spec 4.6). */
    public static final double TURRET_OFFSET_FORWARD_IN = 0.31;
    public static final double TURRET_OFFSET_LEFT_IN = 0.0;

    private final DoubleUnaryOperator flightTimeS;
    private final ShotSolution solution = new ShotSolution();

    private boolean hasLastVelocity = false;
    private double lastVx = 0;
    private double lastVy = 0;
    private double ax = 0;
    private double ay = 0;
    private boolean hasLastAngle = false;
    private double lastTurretDeg = 0;

    /** flightTimeS maps distance (inches) to ball flight time (seconds); Robot passes ShotTable::flightTimeS. */
    public ShotSolver(DoubleUnaryOperator flightTimeS) {
        this.flightTimeS = flightTimeS;
    }

    /** Forget filter history (new OpMode or pose reset). */
    public void reset() {
        hasLastVelocity = false;
        hasLastAngle = false;
        ax = 0;
        ay = 0;
        solution.turretRateDegPerS = 0;
    }

    public ShotSolution solution() {
        return solution;
    }

    /**
     * One loop. Pose and velocity in the field frame (in, rad, in/s, rad/s); dtS = seconds since the previous call.
     * A non-finite input calls reset() (acceleration and turret-rate filters cleared, turret rate zeroed) and returns
     * the last solution, unchanged apart from that zero turret rate.
     */
    public ShotSolution update(double x, double y, double heading, double vx, double vy, double omega,
                               double dtS, double cellX, double cellY) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)
                || !Double.isFinite(vx) || !Double.isFinite(vy) || !Double.isFinite(omega)
                || !Double.isFinite(dtS) || !Double.isFinite(cellX) || !Double.isFinite(cellY)) {
            reset();
            return solution;
        }
        estimateAcceleration(vx, vy, dtS);

        double latency = SHOOT_ON_MOVE ? RELEASE_LATENCY_S : 0;
        double releaseHeading = heading + omega * latency;
        double moveX = 0;
        double moveY = 0;
        double leadVx = 0;
        double leadVy = 0;
        double releaseSpeed = 0;

        double speed = Math.hypot(vx, vy);
        if (SHOOT_ON_MOVE && speed > 1e-6 && speed >= MIN_SPEED_IN_S) {
            double ux = vx / speed;
            double uy = vy / speed;
            double alongAccel = ax * ux + ay * uy;
            releaseSpeed = speed + alongAccel * latency;
            double travel;
            if (releaseSpeed <= 0) {
                travel = alongAccel < 0 ? speed * speed / (-2 * alongAccel) : 0;
                releaseSpeed = 0;
            } else {
                travel = (speed + releaseSpeed) / 2 * latency;
            }
            moveX = ux * travel;
            moveY = uy * travel;
            leadVx = ux * releaseSpeed;
            leadVy = uy * releaseSpeed;
        }

        double cos = Math.cos(releaseHeading);
        double sin = Math.sin(releaseHeading);
        double rx = x + moveX + TURRET_OFFSET_FORWARD_IN * cos - TURRET_OFFSET_LEFT_IN * sin;
        double ry = y + moveY + TURRET_OFFSET_FORWARD_IN * sin + TURRET_OFFSET_LEFT_IN * cos;

        double aimX = cellX;
        double aimY = cellY;
        for (int i = 0; i < 3; i++) {
            double flight = flightTimeS.applyAsDouble(Math.hypot(aimX - rx, aimY - ry));
            aimX = cellX - leadVx * flight * LEAD_GAIN;
            aimY = cellY - leadVy * flight * LEAD_GAIN;
        }

        double turretDeg = MathUtil.wrapDegrees(Math.toDegrees(Math.atan2(aimY - ry, aimX - rx) - releaseHeading));
        if (hasLastAngle && dtS > 0) {
            double rate = MathUtil.wrapDegrees(turretDeg - lastTurretDeg) / dtS;
            solution.turretRateDegPerS += RATE_SMOOTHING * (rate - solution.turretRateDegPerS);
        } else {
            solution.turretRateDegPerS = 0;
        }
        lastTurretDeg = turretDeg;
        hasLastAngle = true;

        solution.turretDeg = turretDeg;
        solution.distanceIn = Math.hypot(aimX - rx, aimY - ry);
        solution.aimX = aimX;
        solution.aimY = aimY;
        solution.releaseX = rx;
        solution.releaseY = ry;
        solution.releaseSpeed = releaseSpeed;
        return solution;
    }

    private void estimateAcceleration(double vx, double vy, double dtS) {
        if (hasLastVelocity && dtS > 0) {
            double rawAx = (vx - lastVx) / dtS;
            double rawAy = (vy - lastVy) / dtS;
            double magnitude = Math.hypot(rawAx, rawAy);
            if (magnitude > MAX_ACCEL_IN_S2) {
                rawAx *= MAX_ACCEL_IN_S2 / magnitude;
                rawAy *= MAX_ACCEL_IN_S2 / magnitude;
            }
            ax += ACCEL_SMOOTHING * (rawAx - ax);
            ay += ACCEL_SMOOTHING * (rawAy - ay);
        }
        lastVx = vx;
        lastVy = vy;
        hasLastVelocity = true;
    }
}
