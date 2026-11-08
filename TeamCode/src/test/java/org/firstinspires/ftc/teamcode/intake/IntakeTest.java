package org.firstinspires.ftc.teamcode.intake;

import static org.junit.Assert.assertEquals;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class IntakeTest {

    private static final double EPS = 1e-9;

    private Intake intake;

    private double savedStage1Sign;
    private double savedStage2Sign;
    private double savedStage1Intake;
    private double savedStage2Intake;
    private double savedStage1Feed;
    private double savedStage2Feed;
    private double savedEject;

    @Before
    public void setUp() {
        savedStage1Sign = Intake.STAGE1_SIGN;
        savedStage2Sign = Intake.STAGE2_SIGN;
        savedStage1Intake = Intake.STAGE1_INTAKE_POWER;
        savedStage2Intake = Intake.STAGE2_INTAKE_POWER;
        savedStage1Feed = Intake.STAGE1_FEED_POWER;
        savedStage2Feed = Intake.STAGE2_FEED_POWER;
        savedEject = Intake.EJECT_POWER;
        Intake.STAGE1_SIGN = -1.0;
        Intake.STAGE2_SIGN = 1.0;
        Intake.STAGE1_INTAKE_POWER = 1.0;
        Intake.STAGE2_INTAKE_POWER = -0.5;
        Intake.STAGE1_FEED_POWER = 0.8;
        Intake.STAGE2_FEED_POWER = 1.0;
        Intake.EJECT_POWER = -1.0;
        Scheduler.reset();
        intake = new Intake();
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Intake.STAGE1_SIGN = savedStage1Sign;
        Intake.STAGE2_SIGN = savedStage2Sign;
        Intake.STAGE1_INTAKE_POWER = savedStage1Intake;
        Intake.STAGE2_INTAKE_POWER = savedStage2Intake;
        Intake.STAGE1_FEED_POWER = savedStage1Feed;
        Intake.STAGE2_FEED_POWER = savedStage2Feed;
        Intake.EJECT_POWER = savedEject;
    }

    private void assertStages(double stage1, double stage2) {
        assertEquals(stage1, intake.stage1Power(), EPS);
        assertEquals(stage2, intake.stage2Power(), EPS);
    }

    private void assertFeeding() {
        assertStages(Intake.STAGE1_FEED_POWER, Intake.STAGE2_FEED_POWER);
    }

    private void startFeed() {
        intake.feed(() -> true).schedule();
        Scheduler.execute();
        assertFeeding();
    }

    @Test
    public void feedPushesOnTheFirstLoopWithNoDelay() {
        startFeed();
    }

    @Test
    public void feedStopsTheStagesWhileNotFeeding() {
        boolean[] feeding = {false};
        intake.feed(() -> feeding[0]).schedule();
        Scheduler.execute();
        assertStages(0, 0);
        feeding[0] = true;
        Scheduler.execute();
        assertFeeding();
        feeding[0] = false;
        Scheduler.execute();
        assertStages(0, 0);
    }

    @Test
    public void intakeRunsStageTwoBackwards() {
        intake.intake().schedule();
        Scheduler.execute();
        assertStages(1.0, -0.5);
    }

    @Test
    public void intakeOverridingAFeedRunsTheIntake() {
        startFeed();
        intake.intake().schedule();
        Scheduler.execute();
        assertStages(Intake.STAGE1_INTAKE_POWER, Intake.STAGE2_INTAKE_POWER);
    }

    @Test
    public void ejectOverridingAFeedRunsTheEject() {
        startFeed();
        intake.eject().schedule();
        Scheduler.execute();
        assertStages(Intake.EJECT_POWER, Intake.EJECT_POWER);
    }

    @Test
    public void cancellingAFeedStopsTheStages() {
        Command feed = intake.feed(() -> true);
        feed.schedule();
        Scheduler.execute();
        assertFeeding();
        feed.cancel();
        assertStages(0, 0);
    }

    /** Same shape as ShootCommands.whileHeld: a step, then the feed. */
    private Command heldFeed() {
        Command held = Commands.instant(() -> { }).then(intake.feed(() -> true));
        held.schedule();
        for (int i = 0; i < 3 && intake.stage1Power() == 0; i++) {
            Scheduler.execute();
        }
        assertFeeding();
        return held;
    }

    @Test
    public void cancellingAHeldFeedSequenceStopsTheStages() {
        Command held = heldFeed();
        held.cancel();
        assertStages(0, 0);
    }

    @Test
    public void intakeOverridingAHeldFeedSequenceRunsTheIntake() {
        heldFeed();
        intake.intake().schedule();
        Scheduler.execute();
        assertStages(Intake.STAGE1_INTAKE_POWER, Intake.STAGE2_INTAKE_POWER);
    }

    @Test
    public void feedEndedByUntilStopsTheStages() {
        boolean[] done = {false};
        intake.feed(() -> true).until(() -> done[0]).schedule();
        Scheduler.execute();
        assertFeeding();
        done[0] = true;
        Scheduler.execute();
        assertStages(0, 0);
    }
}
