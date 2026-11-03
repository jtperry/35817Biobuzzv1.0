package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class HoodTest {

    private static final double EPS = 1e-9;

    private Hood hood;
    private double written;

    @Before
    public void setUp() {
        Hood.MIN_POSITION = 0.0254;
        Hood.MAX_POSITION = 0.45;
        Hood.STOW_POSITION = 0.0254;
        Hood.SLEW_POS_PER_SEC = 1.5;
        Hood.SETTLE_TOLERANCE = 0.01;
        hood = new Hood();
        written = Double.NaN;
        hood.output = position -> written = position;
    }

    @After
    public void tearDown() {
        Hood.MIN_POSITION = 0.0254;
        Hood.MAX_POSITION = 0.45;
        Hood.STOW_POSITION = 0.0254;
        Hood.SLEW_POS_PER_SEC = 1.5;
        Hood.SETTLE_TOLERANCE = 0.01;
    }

    @Test
    public void holdBelowMinimumWritesMinimum() {
        hood.hold(0.0);
        hood.periodic();
        assertEquals(Hood.MIN_POSITION, written, EPS);
        assertEquals(Hood.MIN_POSITION, hood.commanded(), EPS);
    }

    @Test
    public void holdAboveMaximumWritesMaximum() {
        hood.hold(0.9);
        hood.periodic();
        assertEquals(Hood.MAX_POSITION, written, EPS);
    }

    @Test
    public void holdInsideTheRangeIsUnchanged() {
        hood.hold(0.2);
        hood.periodic();
        assertEquals(0.2, written, EPS);
    }

    @Test
    public void stowHoldStaysInsideTheRangeEvenIfStowIsMisset() {
        Hood.STOW_POSITION = 0.0;
        hood.stow();
        hood.periodic();
        assertEquals(Hood.MIN_POSITION, written, EPS);
    }

    @Test
    public void rawKeepsTheFullServoRange() {
        double[] raw = {0.0};
        hood.raw(() -> raw[0]);
        hood.periodic();
        assertEquals(0.0, written, EPS);
        raw[0] = 1.0;
        hood.periodic();
        assertEquals(1.0, written, EPS);
        raw[0] = 1.5;
        hood.periodic();
        assertEquals(1.0, written, EPS);
        raw[0] = -0.5;
        hood.periodic();
        assertEquals(0.0, written, EPS);
    }

    @Test
    public void resetWritesAStowInsideTheRange() {
        Hood.STOW_POSITION = 0.0;
        hood.hold(0.3);
        hood.reset();
        assertEquals(Hood.MIN_POSITION, written, EPS);
        assertEquals(Hood.MIN_POSITION, hood.commanded(), EPS);
        assertEquals(Hood.Mode.TRACK, hood.mode());
        Hood.STOW_POSITION = 0.9;
        hood.reset();
        assertEquals(Hood.MAX_POSITION, written, EPS);
    }
}
