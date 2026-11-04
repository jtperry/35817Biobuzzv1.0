package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class ShotTableTest {

    private double savedRpmScale;
    private double savedExitEfficiency;
    private double savedLaunchHeight;
    private double savedCellHeight;
    private double[] savedFlightKeys;
    private double[] savedFlightValues;

    @Before
    public void setUp() {
        savedRpmScale = ShotTable.RPM_SCALE;
        savedExitEfficiency = ShotTable.BALL_EXIT_EFFICIENCY;
        savedLaunchHeight = ShotTable.LAUNCH_HEIGHT_IN;
        savedCellHeight = ShotTable.CELL_HEIGHT_IN;
        int points = ShotTable.FLIGHT_S.size();
        savedFlightKeys = new double[points];
        savedFlightValues = new double[points];
        for (int i = 0; i < points; i++) {
            savedFlightKeys[i] = ShotTable.FLIGHT_S.keyAt(i);
            savedFlightValues[i] = ShotTable.FLIGHT_S.valueAt(i);
        }
        ShotTable.RPM_SCALE = 1.0;
        ShotTable.BALL_EXIT_EFFICIENCY = 0.5;
        ShotTable.LAUNCH_HEIGHT_IN = 12.5;
        ShotTable.CELL_HEIGHT_IN = 59.5;
        ShotTable.FLIGHT_S.clear();
    }

    @After
    public void tearDown() {
        ShotTable.RPM_SCALE = savedRpmScale;
        ShotTable.BALL_EXIT_EFFICIENCY = savedExitEfficiency;
        ShotTable.LAUNCH_HEIGHT_IN = savedLaunchHeight;
        ShotTable.CELL_HEIGHT_IN = savedCellHeight;
        ShotTable.FLIGHT_S.clear();
        for (int i = 0; i < savedFlightKeys.length; i++) {
            ShotTable.FLIGHT_S.put(savedFlightKeys[i], savedFlightValues[i]);
        }
    }

    @Test
    public void physicsFlightTimeOfARealShot() {
        // Hand check: 60 in at 3200 RPM is about 0.43 s.
        double seconds = ShotTable.physicsFlightTimeS(60, 3200);
        assertTrue("flight time " + seconds, seconds > 0.4 && seconds < 0.5);
    }

    @Test
    public void physicsFlightTimeOfANaNInputIsZero() {
        assertEquals(0.0, ShotTable.physicsFlightTimeS(Double.NaN, 3200), 0.0);
        assertEquals(0.0, ShotTable.physicsFlightTimeS(60, Double.NaN), 0.0);
    }

    @Test
    public void rpmAppliesTheScale() {
        double base = ShotTable.rpm(60.7);
        assertTrue("table rpm " + base, base > 0);
        ShotTable.RPM_SCALE = 0.9;
        assertEquals(base * 0.9, ShotTable.rpm(60.7), 1e-9);
    }

    @Test
    public void flightTimePrefersTheTableWhenItHasPoints() {
        ShotTable.FLIGHT_S.put(60.7, 0.77);
        assertEquals(0.77, ShotTable.flightTimeS(60.7), 1e-9);
    }

    @Test
    public void flightTimeFallsBackToPhysicsWhileTheTableIsEmpty() {
        assertTrue(ShotTable.FLIGHT_S.isEmpty());
        ShotTable.RPM_SCALE = 0.9;
        double seconds = ShotTable.flightTimeS(60.7);
        assertTrue("flight time " + seconds, seconds > 0 && !Double.isInfinite(seconds));
        // The scaled rpm must be what feeds physics, not the raw table value.
        assertEquals(ShotTable.physicsFlightTimeS(60.7, 0.9 * ShotTable.RPM.get(60.7)), seconds, 1e-9);
        assertTrue(Math.abs(seconds - ShotTable.physicsFlightTimeS(60.7, ShotTable.RPM.get(60.7))) > 1e-3);
    }
}
