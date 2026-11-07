package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

/** Robot's auto-to-teleop pose handoff, in the order Robot and the OpModes call it. */
public class AutoHandoffTest {

    private final Pose fallback = new Pose(8, 7, Math.PI / 2);
    private final Pose autoStart = new Pose(136, 137, -Math.PI / 2);
    private final Pose autoEnd = new Pose(100, 90, Math.PI);

    private AutoHandoff handoff;

    @Before
    public void setUp() {
        handoff = new AutoHandoff();
    }

    /** Robot.beginAuto: initFollower (stops recording), then the auto starts recording. */
    private void autoInit(Pose startPose) {
        handoff.stopRecording();
        handoff.beginAuto(startPose);
    }

    /** Robot.initFollower from any OpMode that is not an auto. */
    private void otherInit() {
        handoff.stopRecording();
    }

    @Test
    public void noAutoMeansTheFallbackPose() {
        otherInit();
        assertFalse(handoff.pending());
        assertSame(fallback, handoff.poseOr(fallback));
    }

    @Test
    public void autoEndPoseCarriesIntoTeleop() {
        autoInit(autoStart);
        handoff.record(new Pose(120, 120, 0));
        handoff.record(autoEnd);
        otherInit();
        assertTrue(handoff.pending());
        assertSame(autoEnd, handoff.poseOr(fallback));
    }

    @Test
    public void teleopReInitStillStartsFromTheAutoPose() {
        autoInit(autoStart);
        handoff.record(autoEnd);
        otherInit();
        handoff.poseOr(fallback);
        otherInit();
        assertTrue(handoff.pending());
        assertSame(autoEnd, handoff.poseOr(fallback));
    }

    @Test
    public void teleopFirstLoopUsesItUp() {
        autoInit(autoStart);
        handoff.record(autoEnd);
        otherInit();
        handoff.clear();
        assertFalse(handoff.pending());
        assertSame(fallback, handoff.poseOr(fallback));
    }

    @Test
    public void testOpModeInBetweenNeitherUsesUpNorOverwritesTheAutoPose() {
        autoInit(autoStart);
        handoff.record(autoEnd);
        otherInit();
        handoff.record(new Pose(10, 10, 0));
        otherInit();
        assertTrue(handoff.pending());
        assertSame(autoEnd, handoff.poseOr(fallback));
    }

    @Test
    public void newAutoReplacesTheOldPose() {
        autoInit(autoStart);
        handoff.record(autoEnd);
        otherInit();
        handoff.clear();
        Pose secondStart = new Pose(8, 7, Math.PI / 2);
        Pose secondEnd = new Pose(40, 50, 0);
        autoInit(secondStart);
        assertSame(secondStart, handoff.poseOr(fallback));
        handoff.record(secondEnd);
        otherInit();
        assertTrue(handoff.pending());
        assertSame(secondEnd, handoff.poseOr(fallback));
    }
}
