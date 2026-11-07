package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TurretTest {

    private static final double EPS = 1e-9;
    private static final double MIN = -200.5;
    private static final double MAX = 200.5;
    private static final double HYST = 10;

    private static double choose(double degrees, double current) {
        return Turret.chooseAngle(degrees, current, MIN, MAX, HYST);
    }

    @Test
    public void exactInRangeAngleWinsWhenTheTurretIsNotNearTheLimit() {
        assertEquals(-155.0, choose(-155, 5), EPS);
        assertEquals(-155.0, choose(205, 5), EPS);
    }

    @Test
    public void bearingJumpGoesToTheExactAngleNotALimit() {
        assertEquals(-152.0, choose(-152, 5), EPS);
        assertEquals(152.0, choose(152, -5), EPS);
    }

    @Test
    public void waitsAtTheLimitWhenAlreadyThere() {
        assertEquals(MAX, choose(205, MAX), EPS);
        assertEquals(MAX, choose(205, 195), EPS);
        assertEquals(MIN, choose(-205, MIN), EPS);
        assertEquals(MIN, choose(-205, -195), EPS);
    }

    @Test
    public void swingsAroundOncePastTheHysteresis() {
        assertEquals(-149.0, choose(211, MAX), EPS);
        assertEquals(149.0, choose(-211, MIN), EPS);
    }

    @Test
    public void asymmetricRangeDoesNotTreatTheSignAsTheSide() {
        assertEquals(-105.0, Turret.chooseAngle(255, 5, -150, 250, HYST), EPS);
        assertEquals(250.0, Turret.chooseAngle(255, 245, -150, 250, HYST), EPS);
    }

    @Test
    public void setCalibrationMovesTheMapping() {
        double center = Turret.centerPosition();
        double degPerPosition = Turret.DEG_PER_POSITION;
        try {
            Turret.setCalibration(0.6, -350.0);
            assertEquals(0.6, Turret.centerPosition(), EPS);
            assertEquals(0.6, Turret.positionAt(0), EPS);
            assertEquals(-35.0, Turret.degreesAt(0.7), EPS);
        } finally {
            Turret.setCalibration(center, degPerPosition);
        }
    }

    @Test
    public void onTargetSeesAHoldShortOfTheRequest() {
        assertFalse(Turret.onTarget(205, MAX, 2.0));
        assertFalse(Turret.onTarget(-205, MIN, 2.0));
        assertTrue(Turret.onTarget(202, MAX, 2.0));
    }

    @Test
    public void onTargetIsModuloATurn() {
        assertTrue(Turret.onTarget(-155, -155, 2.0));
        assertTrue(Turret.onTarget(205, -155, 2.0));
        assertTrue(Turret.onTarget(MAX + 360, MAX, 2.0));
        assertFalse(Turret.onTarget(0, 20, 2.0));
    }

    @Test
    public void onTargetIsFalseForNaN() {
        assertFalse(Turret.onTarget(Double.NaN, 0, 2.0));
    }
}
