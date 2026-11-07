package org.biobuzz.sim;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.util.Range;

import org.horizon36596.simloop.fakehardware.AbstractFakeDevice;

import java.util.Arrays;

/**
 * The OctoQuad our code reads: localizer pose and field-frame velocity, plus the flywheel encoder on port 2.
 *
 * <p>The reported pose is the robot's true pose (BiobuzzSim pushes the plant's every tick) through a rigid offset:
 * resetLocalizerAndCalibrateIMU makes it read (0, 0, 0) where the robot stands, and setLocalizerPose makes it read
 * the given pose there. Neither moves the plant. Status is RUNNING and every CRC good from the start, so INIT never
 * waits. Numbers go through the device's registers as OctoQuadImpl unpacks them: mm and mm/s as shorts, heading in
 * 1/5000 rad steps within +-pi, heading rate in 1/600 rad/s steps. Velocity is in the reported (field) frame, as our
 * code assumes (unverified on the real device). Encoder velocities are raw counts per the port's sample interval,
 * signed by the port's configured direction. Methods our code never calls throw UnsupportedOperationException with
 * the method name.
 */
public final class FakeOctoQuad extends AbstractFakeDevice implements OctoQuad {

    private static final double MM_PER_IN = 25.4;
    /** OctoQuadImpl's register scales: heading in 1/5000 rad steps, heading rate in 1/600 rad/s steps. */
    private static final float HEADING_SCALE = 1 / 5000f;
    private static final float TURN_SCALE = 1 / 600f;
    /** Sample interval of a port nobody set, ms (assumed firmware default; our code sets port 2's own). */
    private static final int DEFAULT_VELOCITY_INTERVAL_MS = 50;

    private double trueX;
    private double trueY;
    private double trueHeading;
    private double trueForward;
    private double trueLeft;
    private double trueTurn;

    private double offsetX;
    private double offsetY;
    private double offsetHeading;

    private final EncoderDirection[] directions = new EncoderDirection[NUM_ENCODERS];
    private final int[] intervalsMs = new int[NUM_ENCODERS];
    private final double[] rawCountsPerSecond = new double[NUM_ENCODERS];
    private final double[] rawCounts = new double[NUM_ENCODERS];
    private I2cRecoveryMode i2cRecoveryMode = I2cRecoveryMode.NONE;

    public FakeOctoQuad() {
        Arrays.fill(directions, EncoderDirection.FORWARD);
        Arrays.fill(intervalsMs, DEFAULT_VELOCITY_INTERVAL_MS);
    }

    /** Sets the robot's true state, plant frame: inches, radians CCW, robot-frame velocity (+ left) and turn rate. */
    public void setTrueState(double xIn, double yIn, double headingRad,
                             double forwardInPerS, double leftInPerS, double turnRadPerS) {
        trueX = xIn * MM_PER_IN;
        trueY = yIn * MM_PER_IN;
        trueHeading = headingRad;
        trueForward = forwardInPerS * MM_PER_IN;
        trueLeft = leftInPerS * MM_PER_IN;
        trueTurn = turnRadPerS;
    }

    /** Sets what an encoder counts, raw counts per second, before the port's configured direction. */
    public void setEncoderVelocity(int port, double rawCountsPerSecond) {
        checkPort(port);
        this.rawCountsPerSecond[port] = rawCountsPerSecond;
    }

    /** Counts the encoders on (positions, which our code does not read). */
    @Override
    public void update(double deltaTime) {
        if (!(deltaTime > 0) || Double.isInfinite(deltaTime)) {
            return;
        }
        for (int i = 0; i < NUM_ENCODERS; i++) {
            rawCounts[i] += rawCountsPerSecond[i] * deltaTime;
        }
    }

    private static UnsupportedOperationException notSimulated(String method) {
        return new UnsupportedOperationException("FakeOctoQuad: " + method + " is not simulated");
    }

    /** As OctoQuadImpl checks an encoder index. */
    private static void checkPort(int idx) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
    }

    /** A value as the device's 16-bit register holds it (rounded); a value it cannot hold is a sim bug. */
    private static short register(double value, String name) {
        long rounded = Math.round(value);
        if (rounded < Short.MIN_VALUE || rounded > Short.MAX_VALUE) {
            throw new IllegalStateException("FakeOctoQuad: " + name + " = " + value + " does not fit the register");
        }
        return (short) rounded;
    }

    /** In [-pi, pi). */
    private static double wrap(double angle) {
        return angle - 2 * Math.PI * Math.floor((angle + Math.PI) / (2 * Math.PI));
    }

    /** Sets the offset so the report reads (x mm, y mm, heading rad) where the robot stands now. */
    private void teleportReport(double x, double y, double heading) {
        offsetHeading = heading - trueHeading;
        double cos = Math.cos(offsetHeading);
        double sin = Math.sin(offsetHeading);
        offsetX = x - (cos * trueX - sin * trueY);
        offsetY = y - (sin * trueX + cos * trueY);
    }


    @Override
    public void setSingleEncoderDirection(int idx, EncoderDirection direction) {
        checkPort(idx);
        directions[idx] = direction;
    }

    @Override
    public EncoderDirection getSingleEncoderDirection(int idx) {
        checkPort(idx);
        return directions[idx];
    }

    @Override
    public void setSingleVelocitySampleInterval(int idx, int intvlms) {
        checkPort(idx);
        Range.throwIfRangeIsInvalid(intvlms, MIN_VELOCITY_MEASUREMENT_INTERVAL_MS, MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);
        intervalsMs[idx] = intvlms;
    }

    @Override
    public int getSingleVelocitySampleInterval(int idx) {
        checkPort(idx);
        return intervalsMs[idx];
    }

    /**
     * Checked as the device's setters check them, otherwise ignored: the fake is an ideal localizer at the robot's
     * centre, so pod ports, resolutions, offsets and the heading scalar change nothing it reports.
     */
    @Override
    public void setAllLocalizerParameters(int portX, int portY, float ticksPerMM_x, float ticksPerMM_y,
                                          float tcpOffsetMM_X, float tcpOffsetMM_Y, float headingScalar,
                                          int velocityIntervalMs) {
        checkPort(portX);
        checkPort(portY);
        Range.throwIfRangeIsInvalid(headingScalar, 0, Float.MAX_VALUE);
        Range.throwIfRangeIsInvalid(velocityIntervalMs, MIN_VELOCITY_MEASUREMENT_INTERVAL_MS,
                MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);
    }

    @Override
    public void setI2cRecoveryMode(I2cRecoveryMode mode) {
        i2cRecoveryMode = mode;
    }

    @Override
    public I2cRecoveryMode getI2cRecoveryMode() {
        return i2cRecoveryMode;
    }

    /** Reads (0, 0, 0) where the robot stands; RUNNING again at once (no IMU calibration wait). */
    @Override
    public void resetLocalizerAndCalibrateIMU() {
        teleportReport(0, 0, 0);
    }

    @Override
    public LocalizerStatus getLocalizerStatus() {
        return LocalizerStatus.RUNNING;
    }

    @Override
    public void readLocalizerDataAndAllEncoderData(LocalizerDataBlock localizerOut, EncoderDataBlock encoderOut) {
        double cos = Math.cos(offsetHeading);
        double sin = Math.sin(offsetHeading);
        double x = cos * trueX - sin * trueY + offsetX;
        double y = sin * trueX + cos * trueY + offsetY;
        double heading = trueHeading + offsetHeading;
        double headingCos = Math.cos(heading);
        double headingSin = Math.sin(heading);
        localizerOut.localizerStatus = LocalizerStatus.RUNNING;
        localizerOut.crcOk = true;
        localizerOut.posX_mm = register(x, "posX_mm");
        localizerOut.posY_mm = register(y, "posY_mm");
        localizerOut.heading_rad = register(wrap(heading) / HEADING_SCALE, "heading") * HEADING_SCALE;
        localizerOut.velX_mmS = register(headingCos * trueForward - headingSin * trueLeft, "velX_mmS");
        localizerOut.velY_mmS = register(headingSin * trueForward + headingCos * trueLeft, "velY_mmS");
        localizerOut.velHeading_radS = register(trueTurn / TURN_SCALE, "velHeading") * TURN_SCALE;

        for (int i = 0; i < NUM_ENCODERS; i++) {
            double sign = directions[i] == EncoderDirection.REVERSE ? -1 : 1;
            encoderOut.positions[i] = (int) Math.round(sign * rawCounts[i]);
            encoderOut.velocities[i] = register(sign * rawCountsPerSecond[i] * intervalsMs[i] / 1000.0,
                    "velocities[" + i + "]");
        }
        encoderOut.crcOk = true;
    }

    /** The device holds what OctoQuadImpl.setLocalizerPose writes: shorts, and the heading in 1/5000 rad steps. */
    @Override
    public void setLocalizerPose(int posX_mm, int posY_mm, float heading_rad) {
        short headingSteps = (short) (heading_rad / HEADING_SCALE);
        teleportReport((short) posX_mm, (short) posY_mm, headingSteps * HEADING_SCALE);
    }


    @Override
    public byte getChipId() {
        throw notSimulated("getChipId");
    }

    @Override
    public FirmwareVersion getFirmwareVersion() {
        throw notSimulated("getFirmwareVersion");
    }

    @Override
    public String getFirmwareVersionString() {
        throw notSimulated("getFirmwareVersionString");
    }

    @Override
    public void setAllEncoderDirections(boolean[] reverse) {
        throw notSimulated("setAllEncoderDirections");
    }

    @Override
    public void setChannelBankConfig(ChannelBankConfig config) {
        throw notSimulated("setChannelBankConfig");
    }

    @Override
    public ChannelBankConfig getChannelBankConfig() {
        throw notSimulated("getChannelBankConfig");
    }

    @Override
    public void readAllEncoderData(EncoderDataBlock out) {
        throw notSimulated("readAllEncoderData");
    }

    @Override
    public EncoderDataBlock readAllEncoderData() {
        throw notSimulated("readAllEncoderData");
    }

    @Override
    public void setCachingMode(CachingMode mode) {
        throw notSimulated("setCachingMode");
    }

    @Override
    public void refreshCache() {
        throw notSimulated("refreshCache");
    }

    @Override
    public int readSinglePosition_Caching(int idx) {
        throw notSimulated("readSinglePosition_Caching");
    }

    @Override
    public short readSingleVelocity_Caching(int idx) {
        throw notSimulated("readSingleVelocity_Caching");
    }

    @Override
    public short readSingleVelocity(int idx) {
        throw notSimulated("readSingleVelocity");
    }

    @Override
    public int readSinglePosition(int idx) {
        throw notSimulated("readSinglePosition");
    }

    @Override
    public void resetSinglePosition(int idx) {
        throw notSimulated("resetSinglePosition");
    }

    @Override
    public void resetAllPositions() {
        throw notSimulated("resetAllPositions");
    }

    @Override
    public void resetMultiplePositions(boolean[] resets) {
        throw notSimulated("resetMultiplePositions");
    }

    @Override
    public void resetMultiplePositions(int... indices) {
        throw notSimulated("resetMultiplePositions");
    }

    @Override
    public void setAllVelocitySampleIntervals(int intvlms) {
        throw notSimulated("setAllVelocitySampleIntervals");
    }

    @Override
    public void setSingleChannelPulseWidthParams(int idx, int minLengthUs, int maxLengthUs) {
        throw notSimulated("setSingleChannelPulseWidthParams");
    }

    @Override
    public void setSingleChannelPulseWidthParams(int idx, ChannelPulseWidthParams params) {
        throw notSimulated("setSingleChannelPulseWidthParams");
    }

    @Override
    public ChannelPulseWidthParams getSingleChannelPulseWidthParams(int idx) {
        throw notSimulated("getSingleChannelPulseWidthParams");
    }

    @Override
    public void setSingleChannelPulseWidthTracksWrap(int idx, boolean trackWrap) {
        throw notSimulated("setSingleChannelPulseWidthTracksWrap");
    }

    @Override
    public boolean getSingleChannelPulseWidthTracksWrap(int idx) {
        throw notSimulated("getSingleChannelPulseWidthTracksWrap");
    }

    @Override
    public void setAllChannelsPulseWidthTracksWrap(boolean[] trackWrap) {
        throw notSimulated("setAllChannelsPulseWidthTracksWrap");
    }

    @Override
    public void setLocalizerPortX(int port) {
        throw notSimulated("setLocalizerPortX");
    }

    @Override
    public void setLocalizerPortY(int port) {
        throw notSimulated("setLocalizerPortY");
    }

    @Override
    public void setLocalizerCountsPerMM_X(float ticksPerMM_x) {
        throw notSimulated("setLocalizerCountsPerMM_X");
    }

    @Override
    public void setLocalizerCountsPerMM_Y(float ticksPerMM_y) {
        throw notSimulated("setLocalizerCountsPerMM_Y");
    }

    @Override
    public void setLocalizerTcpOffsetMM_X(float tcpOffsetMM_X) {
        throw notSimulated("setLocalizerTcpOffsetMM_X");
    }

    @Override
    public void setLocalizerTcpOffsetMM_Y(float tcpOffsetMM_Y) {
        throw notSimulated("setLocalizerTcpOffsetMM_Y");
    }

    @Override
    public void setLocalizerImuHeadingScalar(float headingScalar) {
        throw notSimulated("setLocalizerImuHeadingScalar");
    }

    @Override
    public void setLocalizerVelocityIntervalMS(int ms) {
        throw notSimulated("setLocalizerVelocityIntervalMS");
    }

    @Override
    public void readLocalizerData(LocalizerDataBlock out) {
        throw notSimulated("readLocalizerData");
    }

    @Override
    public LocalizerDataBlock readLocalizerData() {
        throw notSimulated("readLocalizerData");
    }

    @Override
    public void setLocalizerHeading(float headingRad) {
        throw notSimulated("setLocalizerHeading");
    }

    @Override
    public LocalizerYawAxis getLocalizerHeadingAxisChoice() {
        throw notSimulated("getLocalizerHeadingAxisChoice");
    }

    @Override
    public int getUptimeMs() {
        throw notSimulated("getUptimeMs");
    }

    @Override
    public int getIMUDiagParam() {
        throw notSimulated("getIMUDiagParam");
    }

    @Override
    public void saveParametersToFlash() {
        throw notSimulated("saveParametersToFlash");
    }

    @Override
    public void resetEverything() {
        throw notSimulated("resetEverything");
    }
}
