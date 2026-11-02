package org.firstinspires.ftc.teamcode.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

public class FeedGateTest {

    private static final long MS = 1_000_000L;

    private boolean ready;
    private boolean aimSettled;
    private double rpmDeficit;
    private long nowNanos;
    private ShootCommands.FeedGate gate;

    @Before
    public void setUp() {
        ShootCommands.GATE_PER_BALL = true;
        ShootCommands.DIP_RPM = 150;
        ShootCommands.READY_TIMEOUT_MS = 2000;
        ShootCommands.VOLLEY_FEED_MS = 1000;
        ShootCommands.VOLLEY_TIMEOUT_MS = 5000;
        ready = false;
        aimSettled = true;
        rpmDeficit = 0;
        nowNanos = 1000 * MS;
        gate = new ShootCommands.FeedGate(() -> ready, () -> aimSettled, () -> rpmDeficit, () -> nowNanos);
        gate.reset();
    }

    private boolean at(long ms) {
        nowNanos = 1000 * MS + ms * MS;
        return gate.feeding();
    }

    @Test
    public void waitsForReadyThenFeeds() {
        assertFalse(at(100));
        ready = true;
        assertTrue(at(400));
    }

    @Test
    public void neverReadyIsForcedAfterTheTimeout() {
        assertFalse(at(1000));
        assertFalse(at(1999));
        assertTrue(at(2000));
        rpmDeficit = 500;
        assertTrue(at(2100));
    }

    @Test
    public void dipPastTheTimeoutStillPausesOnceItWasReady() {
        ready = true;
        assertTrue(at(400));
        ready = false;
        rpmDeficit = 300;
        assertFalse(at(1500));
        assertFalse(at(2100));
        assertFalse(at(2500));
        rpmDeficit = 0;
        ready = true;
        assertTrue(at(2600));
        rpmDeficit = 300;
        ready = false;
        assertFalse(at(2700));
    }

    @Test
    public void aimLossPausesFeeding() {
        ready = true;
        assertTrue(at(400));
        aimSettled = false;
        ready = false;
        assertFalse(at(500));
        aimSettled = true;
        ready = true;
        assertTrue(at(600));
    }

    @Test
    public void aimLostForGoodAfterReadyIsForcedAfterTheTimeout() {
        ready = true;
        assertTrue(at(400));
        aimSettled = false;
        ready = false;
        assertFalse(at(500));
        assertFalse(at(2000));
        assertFalse(at(2499));
        assertTrue(at(2500));
        assertTrue(at(3000));
        assertFalse(gate.volleyDone());
        assertTrue(at(3400));
        assertTrue(gate.volleyDone());
    }

    @Test
    public void dipThatPersistsAfterReadyIsForcedAfterTheTimeout() {
        ready = true;
        assertTrue(at(400));
        ready = false;
        rpmDeficit = 300;
        assertFalse(at(500));
        assertFalse(at(2000));
        assertFalse(at(2499));
        assertTrue(at(2500));
        assertTrue(at(2600));
    }

    @Test
    public void smallDipWithinDipRpmKeepsFeeding() {
        ready = true;
        assertTrue(at(400));
        ready = false;
        rpmDeficit = 100;
        assertTrue(at(500));
    }

    @Test
    public void ungatedFeedIgnoresDipsAndAim() {
        ShootCommands.GATE_PER_BALL = false;
        ready = true;
        assertTrue(at(400));
        ready = false;
        aimSettled = false;
        rpmDeficit = 500;
        assertTrue(at(500));
    }

    @Test
    public void volleyEndsAfterFeedTimeOrTotalTimeout() {
        ready = true;
        at(0);
        at(500);
        assertFalse(gate.volleyDone());
        at(1000);
        assertTrue(gate.volleyDone());

        gate.reset();
        long volleyStart = nowNanos;
        ready = false;
        aimSettled = false;
        at(1000);
        assertFalse(gate.volleyDone());
        nowNanos = volleyStart + 4999 * MS;
        assertFalse(gate.volleyDone());
        nowNanos = volleyStart + 5000 * MS;
        assertTrue(gate.volleyDone());
    }
}
