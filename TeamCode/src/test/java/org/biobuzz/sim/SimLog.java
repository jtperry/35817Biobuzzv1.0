package org.biobuzz.sim;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * What a logged sim run (OpModeRunner.runLogged) writes to its RLOG for AdvantageScope once a tick, and how a test
 * reads a logged pose back. PsiKit files each key under READ_PREFIX.
 *
 * <ul>
 *   <li>TRUE_POSE: the drive plant, after this tick's world step.</li>
 *   <li>BELIEVED_POSE: the follower's pose as the code holds it after this tick's loop, read from the OctoQuad at the
 *       start of the tick, so it trails TRUE_POSE by one tick.</li>
 *   <li>TARGET_CELL: the hive cell the code aims at, "A" or "B".</li>
 *   <li>FLYWHEEL_RPM: the flywheel model's true speed.</li>
 *   <li>STEP: the routine step started last; "before start" until PLAY, "after routine-end" once it is done.</li>
 * </ul>
 *
 * <p>Poses are PsiKit Pose2d structs (WPILib's layout: x and y in metres, then the heading in radians, three
 * little-endian doubles), which AdvantageScope reads as poses. Frame: the plant's, which is Pedro's axes with the
 * origin moved to the field centre (Pedro minus 72 in on x and y, no rotation; the frame the Zenith traces use),
 * heading counter-clockwise from +x, wrapped to [-pi, pi). Red is logged as it drives, not mirrored. UNVERIFIED: how
 * AdvantageScope's FTC field places this frame (its origin and which way its axes point; SimLoop's FieldFrameTransform
 * and Pedro's FTCCoordinates disagree on the rotation, so none is applied). Check it once against the start pose
 * before trusting the field overlay.
 */
public final class SimLog {

    public static final String TRUE_POSE = "Sim/Pose/True";
    public static final String BELIEVED_POSE = "Sim/Pose/Believed";
    public static final String TARGET_CELL = "Sim/TargetCell";
    public static final String FLYWHEEL_RPM = "Sim/FlywheelRpm";
    public static final String STEP = "Sim/Step";
    public static final List<String> KEYS = Collections.unmodifiableList(
            Arrays.asList(TRUE_POSE, BELIEVED_POSE, TARGET_CELL, FLYWHEEL_RPM, STEP));
    /** Where a logged key is read back (RlogDecodedCompare, RunResult): PsiKit's outputs table. */
    public static final String READ_PREFIX = "RealOutputs/";

    private static final double METRES_PER_INCH = 0.0254;

    private SimLog() {
    }

    /**
     * Records one tick while a logged run is recording, and does nothing otherwise, so run() and runLogged() run the
     * same code. believed ({x in, y in, heading rad}, Pedro frame) and cell are null until the code first reports
     * them; their keys are left out until then.
     */
    public static void record(BiobuzzSim sim, double[] believed, String cell, String step) {
        if (!Logger.isRunning()) {
            return;
        }
        Logger.recordOutput(TRUE_POSE, Pose2d.struct, loggedPose(sim.pedroX(), sim.pedroY(), sim.heading()));
        if (believed != null) {
            Logger.recordOutput(BELIEVED_POSE, Pose2d.struct, loggedPose(believed[0], believed[1], believed[2]));
        }
        if (cell != null) {
            Logger.recordOutput(TARGET_CELL, cell);
        }
        Logger.recordOutput(FLYWHEEL_RPM, sim.flywheelRpm());
        Logger.recordOutput(STEP, step);
    }

    /** A Pedro pose (inches, radians) as it is logged: the plant frame, metres, heading wrapped. */
    public static Pose2d loggedPose(double pedroX, double pedroY, double headingRad) {
        return new Pose2d((pedroX - BiobuzzSim.PEDRO_OFFSET_IN) * METRES_PER_INCH,
                (pedroY - BiobuzzSim.PEDRO_OFFSET_IN) * METRES_PER_INCH,
                new Rotation2d(MathUtil.wrapRadians(headingRad)));
    }

    /**
     * A logged pose read back. RlogDecodedCompare renders a struct value as its raw bytes, Arrays.toString style
     * ("[0, 0, -64, ...]"); a Pose2d is 24 of them.
     */
    public static Pose2d decodePose(String rendered) {
        String text = rendered.trim();
        if (!text.startsWith("[") || !text.endsWith("]")) {
            throw new IllegalArgumentException("Not a logged Pose2d (raw bytes): " + rendered);
        }
        String[] parts = text.substring(1, text.length() - 1).split(",");
        if (parts.length != Pose2d.struct.getSize()) {
            throw new IllegalArgumentException("Not a logged Pose2d: " + parts.length + " bytes, not "
                    + Pose2d.struct.getSize() + ": " + rendered);
        }
        byte[] bytes = new byte[parts.length];
        for (int i = 0; i < parts.length; i++) {
            bytes[i] = Byte.parseByte(parts[i].trim());
        }
        return Pose2d.struct.unpack(ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN));
    }
}
