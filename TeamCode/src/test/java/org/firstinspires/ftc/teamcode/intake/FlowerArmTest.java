package org.firstinspires.ftc.teamcode.intake;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;

public class FlowerArmTest {

    private FlowerArm arm;

    @Before
    public void setUp() throws Exception {
        arm = new FlowerArm();
        arm.raw(() -> 0.5, true, false);
        assertTrue(rawMode());
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
}
