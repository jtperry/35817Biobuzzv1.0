package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.ARRIVED_IN;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.ARRIVED_RAD;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.CAP_NANOS;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.Decision.ARRIVE;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.Decision.STOP_SHORT;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.Decision.WAIT;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.NEAR_IN;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.NEAR_RAD;
import static org.firstinspires.ftc.teamcode.tuning.ShotTunerSettle.decide;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ShotTunerSettleTest {

    private static final boolean PROGRESSING = true;
    private static final boolean STALLED = false;

    @Test
    public void theTolerancesAndTheCapAreTheDocumentedValues() {
        assertEquals(1.0, ARRIVED_IN, 0.0);
        assertEquals(Math.toRadians(2), ARRIVED_RAD, 0.0);
        assertEquals(3.0, NEAR_IN, 0.0);
        assertEquals(Math.toRadians(10), NEAR_RAD, 0.0);
        assertEquals(10_000_000_000L, CAP_NANOS);
    }

    @Test
    public void withinArrivedArrivesWhateverTheProgressOrTime() {
        for (boolean progressing : new boolean[] {PROGRESSING, STALLED}) {
            for (long nanos : new long[] {0, CAP_NANOS - 1, CAP_NANOS, Long.MAX_VALUE}) {
                assertEquals(ARRIVE, decide(0, 0, progressing, nanos));
                assertEquals(ARRIVE, decide(ARRIVED_IN, ARRIVED_RAD, progressing, nanos));
                assertEquals(ARRIVE, decide(ARRIVED_IN, -ARRIVED_RAD, progressing, nanos));
            }
        }
    }

    @Test
    public void justOutsideArrivedWaitsWhileProgressing() {
        assertEquals(WAIT, decide(Math.nextUp(ARRIVED_IN), 0, PROGRESSING, 0));
        assertEquals(WAIT, decide(0, Math.nextUp(ARRIVED_RAD), PROGRESSING, 0));
        assertEquals(WAIT, decide(0, -Math.nextUp(ARRIVED_RAD), PROGRESSING, 0));
        assertEquals(WAIT, decide(20, Math.PI, PROGRESSING, 0));
    }

    @Test
    public void progressKeepsItWaitingUpToTheCap() {
        assertEquals(WAIT, decide(2, 0, PROGRESSING, CAP_NANOS - 1));
        assertEquals(WAIT, decide(20, Math.toRadians(90), PROGRESSING, CAP_NANOS - 1));
    }

    /** The cap ends the wait even while the robot still makes progress: near arrives, else it stops short. */
    @Test
    public void theCapEndsTheWaitWithProgressContinuing() {
        assertEquals(ARRIVE, decide(2, Math.toRadians(5), PROGRESSING, CAP_NANOS));
        assertEquals(ARRIVE, decide(NEAR_IN, NEAR_RAD, PROGRESSING, CAP_NANOS));
        assertEquals(ARRIVE, decide(NEAR_IN, -NEAR_RAD, PROGRESSING, CAP_NANOS));
        assertEquals(STOP_SHORT, decide(Math.nextUp(NEAR_IN), 0, PROGRESSING, CAP_NANOS));
        assertEquals(STOP_SHORT, decide(0, Math.nextUp(NEAR_RAD), PROGRESSING, CAP_NANOS));
        assertEquals(STOP_SHORT, decide(20, Math.toRadians(90), PROGRESSING, Long.MAX_VALUE));
    }

    /** A stall ends the wait at once: near arrives (the full-power hold closes a small error), else it stops short. */
    @Test
    public void aStallEndsTheWaitNearArrivesElseStopsShort() {
        assertEquals(ARRIVE, decide(Math.nextUp(ARRIVED_IN), 0, STALLED, 0));
        assertEquals(ARRIVE, decide(NEAR_IN, NEAR_RAD, STALLED, 0));
        assertEquals(ARRIVE, decide(NEAR_IN, -NEAR_RAD, STALLED, 0));
        assertEquals(STOP_SHORT, decide(Math.nextUp(NEAR_IN), 0, STALLED, 0));
        assertEquals(STOP_SHORT, decide(0, Math.nextUp(NEAR_RAD), STALLED, 0));
        assertEquals(STOP_SHORT, decide(0, -Math.nextUp(NEAR_RAD), STALLED, 0));
        assertEquals(STOP_SHORT, decide(5, 0, STALLED, CAP_NANOS - 1));
    }

    /** Not finite: stop short (fail closed), even while progressing at the start of the wait. */
    @Test
    public void nonFiniteErrorsStopShort() {
        double[] bad = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (boolean progressing : new boolean[] {PROGRESSING, STALLED}) {
            for (long nanos : new long[] {0, CAP_NANOS}) {
                for (double value : bad) {
                    assertEquals(STOP_SHORT, decide(value, 0, progressing, nanos));
                    assertEquals(STOP_SHORT, decide(0, value, progressing, nanos));
                }
            }
        }
    }
}
