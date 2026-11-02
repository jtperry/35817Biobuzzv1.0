package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

public class ShotTableTest {

    @Before
    public void setUp() {
        ShotTable.BALL_EXIT_EFFICIENCY = 0.5;
        ShotTable.LAUNCH_HEIGHT_IN = 12.5;
        ShotTable.CELL_HEIGHT_IN = 59.5;
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
}
