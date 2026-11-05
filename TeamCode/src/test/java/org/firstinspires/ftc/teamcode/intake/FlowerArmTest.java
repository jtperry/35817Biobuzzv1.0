package org.firstinspires.ftc.teamcode.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;

public class FlowerArmTest {

    /** Slew's longest step is 0.1 s. */
    private static final long STEP_NANOS = 100_000_000L;

    private FlowerArm arm;
    private double[] savedWalk;

    @Before
    public void setUp() throws Exception {
        savedWalk = new double[] {
                FlowerArm.STOWED_POSITION, FlowerArm.DEPLOYED_POSITION, FlowerArm.SPEED_POS_PER_SEC};
        FlowerArm.STOWED_POSITION = 1.0;
        FlowerArm.DEPLOYED_POSITION = 0.0;
        FlowerArm.SPEED_POS_PER_SEC = 0.7;
        arm = new FlowerArm();
        arm.raw(() -> 0.5, true, false);
        assertTrue(rawMode());
    }

    @After
    public void tearDown() {
        FlowerArm.STOWED_POSITION = savedWalk[0];
        FlowerArm.DEPLOYED_POSITION = savedWalk[1];
        FlowerArm.SPEED_POS_PER_SEC = savedWalk[2];
    }

    /** The mode has no getter; the tests read it by name. */
    private boolean rawMode() throws Exception {
        Field field = FlowerArm.class.getDeclaredField("rawMode");
        field.setAccessible(true);
        return field.getBoolean(arm);
    }

    @Test
    public void deployLeavesRaw() throws Exception {
        arm.deploy();
        assertFalse(rawMode());
        assertTrue(arm.isDeployed());
    }

    @Test
    public void stowLeavesRaw() throws Exception {
        arm.deploy();
        arm.raw(() -> 0.5, true, false);
        arm.stow();
        assertFalse(rawMode());
        assertFalse(arm.isDeployed());
    }

    @Test
    public void toggleLeavesRaw() throws Exception {
        arm.toggle();
        assertFalse(rawMode());
        assertTrue(arm.isDeployed());
    }

    @Test
    public void atTargetOnlyOnceTheWalkReachesTheTarget() {
        FlowerArm fresh = new FlowerArm();
        assertFalse("not at target before the first step", fresh.atTarget());
        long now = 0;
        fresh.step(now);
        assertTrue("the first step of an OpMode jumps to the stow target", fresh.atTarget());

        fresh.deploy();
        assertFalse("deploy starts a walk", fresh.atTarget());
        int steps = 0;
        while (!fresh.atTarget() && steps < 100) {
            now += STEP_NANOS;
            fresh.step(now);
            steps++;
        }
        // 1.0 -> 0.0 at 0.7 /s in 0.1 s steps: the first step after the jump has no elapsed time,
        // then 15 steps of 0.07; the 15th lands on the target.
        assertEquals(16, steps);
    }

    @Test
    public void atTargetIsFalseInRaw() {
        FlowerArm fresh = new FlowerArm();
        fresh.step(0);
        assertTrue(fresh.atTarget());
        fresh.raw(() -> 0.5, true, true);
        assertFalse(fresh.atTarget());
    }

    @Test
    public void atTargetRightAfterOneStepWhenTheWalkIsOff() {
        FlowerArm.SPEED_POS_PER_SEC = 0;
        FlowerArm fresh = new FlowerArm();
        fresh.step(0);
        fresh.deploy();
        assertFalse("deploy() alone does not move the arm", fresh.atTarget());
        fresh.step(STEP_NANOS);
        assertTrue(fresh.atTarget());
    }
}
