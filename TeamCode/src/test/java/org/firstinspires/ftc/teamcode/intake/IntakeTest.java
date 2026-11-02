package org.firstinspires.ftc.teamcode.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class IntakeTest {

    private static final double EPS = 1e-9;

    private Intake intake;
    private double gatePosition;

    @Before
    public void setUp() {
        Intake.GATE_OPEN_POSITION = 0.42;
        Intake.GATE_CLOSED_POSITION = 0.23;
        Intake.GATE_OPEN_DELAY_MS = 100;
        Intake.STAGE1_SIGN = -1.0;
        Intake.STAGE2_SIGN = 1.0;
        Intake.STAGE1_INTAKE_POWER = 1.0;
        Intake.STAGE2_INTAKE_POWER = -0.5;
        Intake.STAGE1_FEED_POWER = 0.8;
        Intake.STAGE2_FEED_POWER = 1.0;
        Intake.EJECT_POWER = -1.0;
        Scheduler.reset();
        intake = new Intake();
        gatePosition = Double.NaN;
        intake.gateOutput = position -> gatePosition = position;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    private void startFeed() {
        intake.feed(() -> true).schedule();
        Scheduler.execute();
        assertTrue(intake.isGateOpen());
        assertEquals(Intake.GATE_OPEN_POSITION, gatePosition, EPS);
    }

    @Test
    public void intakeOverridingAFeedClosesTheGate() {
        startFeed();
        intake.intake().schedule();
        Scheduler.execute();
        assertFalse(intake.isGateOpen());
        assertEquals(Intake.GATE_CLOSED_POSITION, gatePosition, EPS);
    }

    @Test
    public void ejectOverridingAFeedClosesTheGate() {
        startFeed();
        intake.eject().schedule();
        Scheduler.execute();
        assertFalse(intake.isGateOpen());
        assertEquals(Intake.GATE_CLOSED_POSITION, gatePosition, EPS);
    }

    @Test
    public void cancellingAFeedClosesTheGate() {
        Command feed = intake.feed(() -> true);
        feed.schedule();
        Scheduler.execute();
        assertTrue(intake.isGateOpen());
        feed.cancel();
        assertFalse(intake.isGateOpen());
        assertEquals(Intake.GATE_CLOSED_POSITION, gatePosition, EPS);
    }

    @Test
    public void feedEndedByUntilClosesTheGate() {
        boolean[] done = {false};
        intake.feed(() -> true).until(() -> done[0]).schedule();
        Scheduler.execute();
        assertTrue(intake.isGateOpen());
        done[0] = true;
        Scheduler.execute();
        assertFalse(intake.isGateOpen());
        assertEquals(Intake.GATE_CLOSED_POSITION, gatePosition, EPS);
    }
}
