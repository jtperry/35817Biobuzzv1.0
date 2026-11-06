package org.biobuzz.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.EncoderDataBlock;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.EncoderDirection;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerDataBlock;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerStatus;

import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.junit.Test;

/**
 * FakeOctoQuad: the pose the code reads is the plant's true pose through a rigid offset that setLocalizerPose sets
 * (it never moves the plant), the velocity is field frame, and port 2 reports flywheel counts per velocity window.
 */
public class FakeOctoQuadTest {

    private static final double MM_PER_IN = 25.4;
    /** The OctoQuad's heading resolution, rad (its register holds 1/5000 rad steps). */
    private static final double HEADING_STEP = 1 / 5000.0;
    /** Its heading-rate resolution, rad/s. */
    private static final double TURN_STEP = 1 / 600.0;

    private final FakeOctoQuad octoQuad = new FakeOctoQuad();
    private final LocalizerDataBlock localizer = new LocalizerDataBlock();
    private final EncoderDataBlock encoders = new EncoderDataBlock();

    private void read(FakeOctoQuad device) {
        device.readLocalizerDataAndAllEncoderData(localizer, encoders);
    }

    @Test
    public void runningWithGoodDataFromTheStart() {
        // SharedOctoQuadLocalizer's INIT wait for RUNNING never sleeps.
        assertEquals(LocalizerStatus.RUNNING, octoQuad.getLocalizerStatus());
        read(octoQuad);
        assertTrue(localizer.isDataValid());
        assertTrue(encoders.isDataValid());
    }

    @Test
    public void resetReportsZeroWhereverTheRobotStands() {
        octoQuad.setTrueState(30, -12, 1.0, 0, 0, 0);
        octoQuad.resetLocalizerAndCalibrateIMU();
        read(octoQuad);
        assertEquals(0, localizer.posX_mm);
        assertEquals(0, localizer.posY_mm);
        assertEquals(0.0, localizer.heading_rad, HEADING_STEP);
    }

    @Test
    public void setLocalizerPoseReportsThatPoseWhileTheRobotStaysPut() {
        octoQuad.setTrueState(38.3, 61.21, Math.PI / 2, 0, 0, 0);
        octoQuad.resetLocalizerAndCalibrateIMU();
        octoQuad.setLocalizerPose(2801, 3383, (float) (Math.PI / 2));
        for (int i = 0; i < 2; i++) {
            read(octoQuad);
            assertEquals(2801, localizer.posX_mm);
            assertEquals(3383, localizer.posY_mm);
            assertEquals(Math.PI / 2, localizer.heading_rad, HEADING_STEP);
        }
    }

    @Test
    public void reportMovesRigidlyWithTheRobotThroughTheOffset() {
        octoQuad.setTrueState(0, 0, 0, 0, 0, 0);
        // The reported frame is turned +90 deg from the true one: a full rigid transform, not a shift.
        octoQuad.setLocalizerPose(1000, 500, (float) (Math.PI / 2));

        // 10 in forward along true +x is 10 in along reported +y.
        octoQuad.setTrueState(10, 0, 0, 0, 0, 0);
        read(octoQuad);
        assertEquals(1000, localizer.posX_mm);
        assertEquals(500 + 254, localizer.posY_mm);
        assertEquals(Math.PI / 2, localizer.heading_rad, HEADING_STEP);

        // Then 4 in to true +y and a 0.5 rad turn: (10, 4) turned 90 deg is (-4, 10).
        octoQuad.setTrueState(10, 4, 0.5, 0, 0, 0);
        read(octoQuad);
        assertEquals(1000 - 4 * MM_PER_IN, localizer.posX_mm, 0.5);
        assertEquals(500 + 10 * MM_PER_IN, localizer.posY_mm, 0.5);
        assertEquals(Math.PI / 2 + 0.5, localizer.heading_rad, HEADING_STEP);
    }

    @Test
    public void velocityIsFieldFrameInTheReportedFrame() {
        octoQuad.setTrueState(0, 0, 0, 0, 0, 0);
        octoQuad.setLocalizerPose(0, 0, (float) (Math.PI / 2));
        // Facing reported +y: 10 in/s forward is +y, 2 in/s to the left is -x.
        octoQuad.setTrueState(0, 0, 0, 10, 2, 0.75);
        read(octoQuad);
        assertEquals(-2 * MM_PER_IN, localizer.velX_mmS, 0.5);
        assertEquals(10 * MM_PER_IN, localizer.velY_mmS, 0.5);
        assertEquals(0.75, localizer.velHeading_radS, TURN_STEP);
    }

    @Test
    public void headingIsReportedWithinPlusMinusPi() {
        octoQuad.setTrueState(0, 0, 2 * Math.PI + 0.3, 0, 0, 0);
        read(octoQuad);
        assertEquals(0.3, localizer.heading_rad, HEADING_STEP);
        octoQuad.setTrueState(0, 0, -Math.PI - 0.2, 0, 0, 0);
        read(octoQuad);
        assertEquals(Math.PI - 0.2, localizer.heading_rad, HEADING_STEP);
    }

    @Test
    public void portReportsCountsPerVelocityWindowSignedByItsDirection() {
        octoQuad.setSingleVelocitySampleInterval(2, 50);
        octoQuad.setSingleEncoderDirection(2, EncoderDirection.FORWARD);
        octoQuad.setEncoderVelocity(2, -1400);
        read(octoQuad);
        assertEquals(-70, encoders.velocities[2]);

        octoQuad.setSingleEncoderDirection(2, EncoderDirection.REVERSE);
        read(octoQuad);
        assertEquals(70, encoders.velocities[2]);

        octoQuad.setSingleVelocitySampleInterval(2, 25);
        read(octoQuad);
        assertEquals(35, encoders.velocities[2]);
        assertEquals(0, encoders.velocities[1]);
    }

    @Test
    public void flywheelSpunForwardReadsPositiveRpmThroughTheCodesOwnFormula() {
        BiobuzzSim sim = new BiobuzzSim();
        FakeOctoQuad device = sim.octoQuad();
        // As Flywheel.useOctoQuad configures port 2.
        device.setSingleEncoderDirection(Flywheel.OCTOQUAD_PORT,
                Flywheel.ENCODER_REVERSED ? EncoderDirection.REVERSE : EncoderDirection.FORWARD);
        device.setSingleVelocitySampleInterval(Flywheel.OCTOQUAD_PORT, Flywheel.VELOCITY_WINDOW_MS);
        // Forward full power, as Flywheel writes it (MOTOR_1_REVERSED false).
        ((SimMotor) sim.hardwareMap.get(Flywheel.MOTOR_1)).setPower(1.0);
        for (int i = 0; i < 100; i++) {
            sim.step(0.02);
        }
        read(device);
        // Flywheel.readRpm.
        double rpm = encoders.velocities[Flywheel.OCTOQUAD_PORT] * (1000.0 / Flywheel.VELOCITY_WINDOW_MS)
                / Flywheel.COUNTS_PER_REV * 60.0;
        assertTrue("model " + sim.flywheelRpm() + " RPM", sim.flywheelRpm() > 3000);
        // One count per window is 42.9 RPM.
        assertEquals(sim.flywheelRpm(), rpm, 43);
    }

    @Test
    public void placingThePlantNeverShowsUpAsMotionAndSetLocalizerPoseNeverMovesIt() {
        BiobuzzSim sim = new BiobuzzSim();
        sim.placeAt(110.3, 133.21, Math.PI / 2);
        FakeOctoQuad device = sim.octoQuad();
        // INIT, all before the first tick: reset, read, then the start pose.
        device.resetLocalizerAndCalibrateIMU();
        device.setLocalizerPose(2801, 3383, (float) (Math.PI / 2));
        for (int i = 0; i < 5; i++) {
            sim.step(0.02);
        }
        read(device);
        assertEquals(2801, localizer.posX_mm);
        assertEquals(3383, localizer.posY_mm);
        assertEquals(Math.PI / 2, localizer.heading_rad, HEADING_STEP);
        assertEquals(0, localizer.velX_mmS);
        assertEquals(0, localizer.velY_mmS);
        assertEquals(0.0, localizer.velHeading_radS, 0.0);
        // The plant did not move to the pose the code set.
        assertEquals(110.3, sim.pedroX(), 1e-9);
        assertEquals(133.21, sim.pedroY(), 1e-9);
        assertEquals(Math.PI / 2, sim.heading(), 1e-9);
    }

    @Test
    public void methodsOurCodeNeverCallsThrowWithTheirName() {
        try {
            octoQuad.readAllEncoderData();
            fail("not simulated");
        } catch (UnsupportedOperationException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("readAllEncoderData"));
        }
        try {
            octoQuad.setLocalizerHeading(1f);
            fail("not simulated");
        } catch (UnsupportedOperationException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("setLocalizerHeading"));
        }
        assertTrue(OctoQuad.NUM_ENCODERS > Flywheel.OCTOQUAD_PORT);
    }
}
