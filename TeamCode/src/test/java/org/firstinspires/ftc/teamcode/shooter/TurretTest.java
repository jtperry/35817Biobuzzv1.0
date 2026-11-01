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
        // 205 is past max by 4.5, but the turret is at +5: the exact answer -155 is in range.
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
        // Range -150..250 (calibrated off center): at +5 the turret is nowhere near max.
        assertEquals(-105.0, Turret.chooseAngle(255, 5, -150, 250, HYST), EPS);
        assertEquals(250.0, Turret.chooseAngle(255, 245, -150, 250, HYST), EPS);
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
