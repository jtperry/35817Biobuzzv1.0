package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.fail;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.junit.Before;
import org.junit.Test;

/**
 * An INIT error must reach the Driver Station. NextFTC builds the OpMode by reflection, so an exception thrown in its
 * constructor arrives wrapped and the SDK never shows it. Robot keeps the error and its init-check hook throws it from
 * afterConstruction(), which NextFTC calls directly, so the SDK shows the original message.
 */
public class RobotInitCheckTest {

    private final Pose start = new Pose(8, 7, Math.PI / 2);

    /** What the SDK throws for a device missing from the robot configuration. */
    private IllegalArgumentException missing;
    private HardwareMap noOctoQuad;
    private Robot robot;

    @Before
    public void setUp() {
        missing = new IllegalArgumentException("Unable to find a hardware device with name \"octoquad\"");
        noOctoQuad = new HardwareMap(null, null) {
            @Override
            public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
                throw missing;
            }
        };
        robot = new Robot();
        robot.bindHardware(noOctoQuad);
    }

    /** Runs the hook as NextFTC does right after construction; returns what it threw, or null. */
    private RuntimeException afterConstruction() {
        try {
            robot.initCheck().afterConstruction();
            return null;
        } catch (RuntimeException e) {
            return e;
        }
    }

    @Test
    public void followerErrorIsThrownByTheHookNotTheConstructor() {
        try {
            robot.initFollower(noOctoQuad, start);
        } catch (RuntimeException e) {
            fail("initFollower threw inside the OpMode constructor: " + e);
        }
        // The same object, so the DS shows its own message.
        assertSame(missing, afterConstruction());
    }

    @Test
    public void failedAutoInitLeavesNoHandoffForTeleop() {
        robot.beginAuto(noOctoQuad, Alliance.RED, start);
        assertFalse(robot.hasAutoHandoff());
        assertSame(missing, afterConstruction());
    }

    @Test
    public void resetAfterAFailedFollowerIsSkippedAndTheFirstErrorKept() {
        robot.initFollower(noOctoQuad, start);
        try {
            robot.resetInInit();
        } catch (RuntimeException e) {
            fail("resetInInit threw inside the OpMode constructor: " + e);
        }
        assertSame(missing, afterConstruction());
    }

    @Test
    public void resetErrorIsKeptForTheHook() {
        // On the laptop NextFTC has no event loop, so the first motor write fails like a misnamed motor would.
        try {
            robot.resetInInit();
        } catch (RuntimeException e) {
            fail("resetInInit threw inside the OpMode constructor: " + e);
        }
        assertNotNull(afterConstruction());
    }

    @Test
    public void nextOpModesBindHardwareClearsTheError() {
        robot.initFollower(noOctoQuad, start);
        robot.bindHardware(noOctoQuad);
        assertSame(null, afterConstruction());
    }
}
