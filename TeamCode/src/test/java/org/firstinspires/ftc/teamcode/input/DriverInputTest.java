package org.firstinspires.ftc.teamcode.input;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class DriverInputTest {

    private static final double EPS = 1e-9;

    @Before
    public void setUp() {
        DriverInput.DEADZONE = 0.08;
        DriverInput.TURN_DEADZONE = 0.08;
        DriverInput.CURVE = 1.0;
    }

    @Test
    public void insideTheDeadzoneIsZero() {
        assertEquals(0.0, DriverInput.stickScale(0.05, 0.03), EPS);
        assertEquals(0.0, DriverInput.turn(-0.07), EPS);
    }

    @Test
    public void fullDeflectionStaysFull() {
        double scale = DriverInput.stickScale(0, -1);
        assertEquals(1.0, scale, EPS);
        double diagonal = DriverInput.stickScale(1, 1);
        assertEquals(1.0, Math.hypot(diagonal, diagonal), EPS);
    }

    @Test
    public void motionStartsSmoothlyAtTheDeadzoneEdge() {
        double magnitude = 0.54;
        double scale = DriverInput.stickScale(magnitude, 0);
        assertEquals(0.5, magnitude * scale, EPS);
        assertEquals(-0.5, DriverInput.turn(-0.54), EPS);
    }

    @Test
    public void deadzoneOfOneOrMoreFromPanelsStaysFinite() {
        DriverInput.DEADZONE = 1.0;
        double diagonal = DriverInput.stickScale(1, 1);
        assertEquals(1.0, Math.hypot(diagonal, diagonal), EPS);
        DriverInput.TURN_DEADZONE = 1.5;
        assertEquals(1.0, DriverInput.turn(1.0), EPS);
        assertEquals(-1.0, DriverInput.turn(-1.0), EPS);
    }

    @Test
    public void negativeDeadzoneFromPanelsKeepsARestingStickStill() {
        DriverInput.TURN_DEADZONE = -0.5;
        assertEquals(0.0, DriverInput.turn(0.0), EPS);
    }

    @Test
    public void curveTwoSquaresTheOutput() {
        DriverInput.CURVE = 2.0;
        assertEquals(0.25, 0.54 * DriverInput.stickScale(0.54, 0), EPS);
        assertEquals(0.25, DriverInput.turn(0.54), EPS);
    }
}
