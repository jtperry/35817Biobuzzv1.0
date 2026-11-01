package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SlewTest {

    private static final long MS = 1_000_000L;
    private static final double EPS = 1e-9;

    @Test
    public void firstStepDoesNotMove() {
        Slew slew = new Slew(0);
        assertEquals(0.0, slew.step(10, 100, 5 * MS), EPS);
    }

    @Test
    public void movesTowardTheTargetAtTheRate() {
        Slew slew = new Slew(0);
        slew.step(10, 100, 1000 * MS);
        assertEquals(5.0, slew.step(10, 100, 1050 * MS), EPS);
    }

    @Test
    public void movesDownToo() {
        Slew slew = new Slew(10);
        slew.step(0, 100, 1000 * MS);
        assertEquals(5.0, slew.step(0, 100, 1050 * MS), EPS);
    }

    @Test
    public void neverOvershoots() {
        Slew slew = new Slew(0);
        slew.step(1, 100, 1000 * MS);
        assertEquals(1.0, slew.step(1, 100, 1050 * MS), EPS);
    }

    @Test
    public void longGapsAreCappedAtATenthOfASecond() {
        Slew slew = new Slew(0);
        slew.step(100, 100, 1000 * MS);
        assertEquals(10.0, slew.step(100, 100, 3000 * MS), EPS);
    }

    @Test
    public void resetJumpsAndRestartsTheClock() {
        Slew slew = new Slew(0);
        slew.step(10, 100, 1000 * MS);
        slew.reset(7);
        assertEquals(7.0, slew.value(), EPS);
        assertEquals(7.0, slew.step(10, 100, 1050 * MS), EPS);
    }
}
