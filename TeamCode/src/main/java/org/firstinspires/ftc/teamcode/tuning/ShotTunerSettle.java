package org.firstinspires.ftc.teamcode.tuning;

/**
 * Shot Tuner settle decision: after a move's final hold, decides each loop whether the move has arrived (the drive cap
 * may be lifted for the recoil hold), should keep waiting under the cap, or should stop short (hold where the robot
 * is, never push on toward the spot). Pure: the OpMode measures the errors and the progress.
 */
public final class ShotTunerSettle {

    /** What the settle phase does this loop. */
    public enum Decision {
        /** At the spot (or near it once the wait has ended): end the move as arrived. */
        ARRIVE,
        /** Not there yet and still making progress: keep the move running under the drive cap. */
        WAIT,
        /** Stalled or out of time away from the spot, or the errors are not finite: hold where the robot is. */
        STOP_SHORT
    }

    /** Arrived: within this distance and heading of the spot. */
    public static final double ARRIVED_IN = 1.0;
    public static final double ARRIVED_RAD = Math.toRadians(2);
    /** Once the wait has ended (stall or cap), still arrives within this distance and heading: the hold closes it. */
    public static final double NEAR_IN = 3.0;
    public static final double NEAR_RAD = Math.toRadians(10);
    /** The longest settle wait, even while the robot still makes progress. */
    public static final long CAP_NANOS = 10_000_000_000L;

    private ShotTunerSettle() {
    }

    /**
     * positionErrorIn = distance to the spot; headingErrorRad = heading error, already wrapped to [-PI, PI);
     * progressing = the robot moved or turned enough within the no-progress window (the OpMode's watchdog notion);
     * settleNanos = time since the settle phase began. ARRIVE within ARRIVED; WAIT while progressing and below
     * CAP_NANOS; then ARRIVE within NEAR, else STOP_SHORT. Errors that are not finite give STOP_SHORT.
     */
    public static Decision decide(double positionErrorIn, double headingErrorRad, boolean progressing,
                                  long settleNanos) {
        if (!Double.isFinite(positionErrorIn) || !Double.isFinite(headingErrorRad)) {
            return Decision.STOP_SHORT;
        }
        double turnRad = Math.abs(headingErrorRad);
        if (positionErrorIn <= ARRIVED_IN && turnRad <= ARRIVED_RAD) {
            return Decision.ARRIVE;
        }
        if (progressing && settleNanos < CAP_NANOS) {
            return Decision.WAIT;
        }
        return positionErrorIn <= NEAR_IN && turnRad <= NEAR_RAD ? Decision.ARRIVE : Decision.STOP_SHORT;
    }
}
