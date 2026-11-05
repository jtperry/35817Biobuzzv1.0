package org.firstinspires.ftc.teamcode.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class AutoCommandsTest {

    private static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    @Test
    public void headingWithinToleranceTheShortWayRound() {
        assertTrue(AutoCommands.headingWithin(rad(268.5), rad(270), 3));
        assertTrue("across 0", AutoCommands.headingWithin(rad(359), rad(1), 3));
        assertTrue("same heading written two ways", AutoCommands.headingWithin(rad(-90), rad(270), 3));
    }

    @Test
    public void headingOutsideToleranceIsNotWithin() {
        assertFalse("start of the turn at the B spot", AutoCommands.headingWithin(rad(226), rad(270), 3));
        assertFalse(AutoCommands.headingWithin(rad(266), rad(270), 3));
    }

    @Test
    public void notANumberIsNeverWithin() {
        assertFalse(AutoCommands.headingWithin(Double.NaN, rad(270), 3));
        assertFalse(AutoCommands.headingWithin(rad(270), rad(270), Double.NaN));
    }
}
