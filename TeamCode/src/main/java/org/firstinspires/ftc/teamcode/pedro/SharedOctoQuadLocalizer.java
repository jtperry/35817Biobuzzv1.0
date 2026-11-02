package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.EncoderDataBlock;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerDataBlock;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Pedro OctoQuad localizer. Also grab all encoder ports, same I2C read. */
public class SharedOctoQuadLocalizer implements Localizer {
    /** Longest wait for RUNNING after a reset (IMU warm-up and calibration). */
    private static final long RUNNING_TIMEOUT_MS = 2000;
    /** Pause between status reads while waiting. */
    private static final long STATUS_POLL_MS = 10;

    private final LocalizerDataBlock localizer = new LocalizerDataBlock();
    private final EncoderDataBlock encoders = new EncoderDataBlock();
    public final OctoQuad octoQuad;
    private final DistanceUnit globalDistanceUnit;

    private MotionState motionState;

    public SharedOctoQuadLocalizer(HardwareMap hardwareMap, OctoQuadConfig config) {
        octoQuad = hardwareMap.get(OctoQuad.class, config.name.get());

        globalDistanceUnit = config.globalDistanceUnit.get();

        octoQuad.setSingleEncoderDirection(config.xPodPort.get(), config.xPodDirection.get());
        octoQuad.setSingleEncoderDirection(config.yPodPort.get(), config.yPodDirection.get());

        double mmPerUnit = config.encoderResolutionUnit.get().toMm(1.0);
        float ticksPerMM = (float) (config.ticksPerUnit.get() / mmPerUnit);

        octoQuad.setAllLocalizerParameters(
                config.xPodPort.get(),
                config.yPodPort.get(),
                ticksPerMM,
                ticksPerMM,
                (float) -config.offsetUnits.get().toMm(config.yPodOffset.get()),
                (float) -config.offsetUnits.get().toMm(config.xPodOffset.get()),
                config.headingScalar.get().floatValue(),
                config.localizerVelocityIntervalMS.get()
        );
        octoQuad.setI2cRecoveryMode(config.i2cRecoveryMode.get());

        reset();

        waitUntilRunning(config.name.get());

        update();
    }

    /**
     * Wait for RUNNING after reset(), at most RUNNING_TIMEOUT_MS (spec 8). A configured OctoQuad that does not answer
     * (unplugged, wrong I2C port) never gets there: stop INIT with a message instead of hanging (Robot.initFollower
     * keeps it for Robot.initCheck(), which shows it on the Driver Station). STOP interrupts this
     * thread: keep the interrupt flag and return at once, so the OpMode ends the normal way.
     */
    private void waitUntilRunning(String name) {
        long start = System.nanoTime();
        LocalizerStatus status;
        while ((status = octoQuad.getLocalizerStatus()) != LocalizerStatus.RUNNING) {
            if (Thread.currentThread().isInterrupted()) {
                return;
            }
            if (System.nanoTime() - start >= RUNNING_TIMEOUT_MS * 1_000_000L) {
                throw new IllegalStateException("OctoQuad '" + name + "' localizer not RUNNING after "
                        + RUNNING_TIMEOUT_MS + " ms (status " + status + "): " + notRunningHint(status, name));
            }
            try {
                Thread.sleep(STATUS_POLL_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    private static String notRunningHint(LocalizerStatus status, String name) {
        switch (status) {
            case WARMING_UP_IMU:
            case CALIBRATING_IMU:
                return "it answers but the IMU is still calibrating. Keep the robot still during INIT; if this repeats,"
                        + " raise RUNNING_TIMEOUT_MS in SharedOctoQuadLocalizer.";
            case FAULT_NO_IMU:
                return "it reports no working IMU. Check the OctoQuad firmware.";
            default:
                return "it is not answering. Check its cable and power, the I2C port it is on, and that the robot"
                        + " configuration names it '" + name + "'.";
        }
    }

    @Override
    public void update() {
        // One read: pose + all 8 encoder ports.
        octoQuad.readLocalizerDataAndAllEncoderData(localizer, encoders);

        if (!localizer.isDataValid()) {
            return;
        }

        Pose pose = new Pose(
                globalDistanceUnit.fromMm(localizer.posX_mm),
                globalDistanceUnit.fromMm(localizer.posY_mm),
                localizer.heading_rad
        );

        Velocity velocity = new Velocity(
                globalDistanceUnit.fromMm(localizer.velX_mmS),
                globalDistanceUnit.fromMm(localizer.velY_mmS),
                localizer.velHeading_radS
        );

        motionState = MotionState.ofVelocity(pose, velocity);
    }

    /** Encoder data from last update(). Check crcOk before use. */
    public EncoderDataBlock encoderData() {
        return encoders;
    }

    @Override
    public void setPose(Pose pose) {
        octoQuad.setLocalizerPose((int) globalDistanceUnit.toMm(pose.x()), (int) globalDistanceUnit.toMm(pose.y()), (float) pose.heading());

        if (motionState != null) {
            motionState = motionState.withPose(pose);
        } else {
            motionState = MotionState.ofVelocity(pose, Velocity.zero());
        }
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void reset() {
        octoQuad.resetLocalizerAndCalibrateIMU();
    }
}
