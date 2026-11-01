package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Slew;

import java.util.function.DoubleSupplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

/**
 * Two Axon servos turning the turret (spec 5.2). Position only: "settled" comes from a slew estimate.
 * Angles: robot frame, degrees, 0 = forward, positive = left. Mode changes are plain calls.
 */
@Configurable
public class Turret implements Mechanism {

    public static final String LEFT_SERVO = "turretLeft";
    public static final String RIGHT_SERVO = "turretRight";
    /** Servo directions, read in bindServos(): flip live during bring-up, re-init the OpMode, then copy into code. */
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = false;
    /** Below one 1 us pulse step (0.0005), so every step is sent. */
    private static final double CACHE_TOLERANCE = 0.0003;

    /** Axon pulse range, us. */
    public static double PWM_MIN_US = 500;
    public static double PWM_MAX_US = 2500;

    /** Servo position with the turret facing robot forward. From Turret Calibration. */
    public static double CENTER_POSITION = 0.5;
    /** Turret degrees per 1.0 of servo position (sign = direction). CAD predicts about 403. From Turret Calibration. */
    public static double DEG_PER_POSITION = 403.0;
    /** Gap kept from each end of travel. */
    public static double LIMIT_MARGIN_DEG = 1.0;
    /** Past a limit by this much on the current side: wait at the limit instead of swinging around. */
    public static double WRAP_HYSTERESIS_DEG = 10.0;

    public static double SLEW_DEG_PER_SEC = 500;
    public static double SETTLE_TOLERANCE_DEG = 2.0;
    /** Lead the command by turret rate x this so the turret keeps up while turning. Start at 0. */
    public static double LAG_S = 0.0;
    public static double TRIM_STEP_DEG = 1.0;

    public enum Mode { TRACK, HOLD, RAW }

    private NextServo left;
    private NextServo right;
    private Mode mode = Mode.TRACK;
    private double trackingDeg = 0;
    private double trackingRateDegPerS = 0;
    private double holdDeg = 0;
    private DoubleSupplier rawPosition = () -> CENTER_POSITION;
    private boolean rawLeft = true;
    private boolean rawRight = true;
    private boolean leftEnabled = true;
    private boolean rightEnabled = true;
    private double trimDeg = 0;
    private double commandedDeg = 0;
    /** Where the turret probably is: chases the command at SLEW_DEG_PER_SEC. */
    private final Slew estimate = new Slew(0);

    public Turret() {
        bindServos();
    }

    /** Fresh servo objects each OpMode (the SDK resets PWM ranges between OpModes). */
    public void bindServos() {
        left = new NextServo(LEFT_SERVO, CACHE_TOLERANCE,
                LEFT_REVERSED ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
        right = new NextServo(RIGHT_SERVO, CACHE_TOLERANCE,
                RIGHT_REVERSED ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
        left.setPwmRange(PWM_MIN_US, PWM_MAX_US);
        right.setPwmRange(PWM_MIN_US, PWM_MAX_US);
        leftEnabled = true;
        rightEnabled = true;
    }

    /** Known state: tracking, no trim, pointed forward now. */
    public void reset() {
        mode = Mode.TRACK;
        trimDeg = 0;
        trackingRateDegPerS = 0;
        command(0);
        estimate.reset(commandedDeg);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case TRACK:
                command(chooseAngle(trackingDeg + trackingRateDegPerS * LAG_S + trimDeg));
                break;
            case HOLD:
                command(MathUtil.clamp(holdDeg, minDeg(), maxDeg()));
                break;
            case RAW:
                double position = MathUtil.clamp(rawPosition.getAsDouble(), 0, 1);
                writePosition(position, rawLeft, rawRight);
                commandedDeg = degreesAt(position);
                break;
        }
        estimate.step(commandedDeg, SLEW_DEG_PER_SEC, System.nanoTime());
    }

    // ---------------------------------------------------------------- modes

    public void track() {
        mode = Mode.TRACK;
    }

    public void hold(double degrees) {
        holdDeg = degrees;
        mode = Mode.HOLD;
    }

    /** Raw servo position on both servos (tests, calibration). */
    public void raw(DoubleSupplier position) {
        raw(position, true, true);
    }

    /** Raw position on the chosen servos only; the other is disabled (no PWM) so the two cannot fight. */
    public void raw(DoubleSupplier position, boolean useLeft, boolean useRight) {
        rawPosition = position;
        rawLeft = useLeft;
        rawRight = useRight;
        mode = Mode.RAW;
    }

    /** Robot sets this every loop from the shot solution. */
    public void setTracking(double degrees, double rateDegPerS) {
        trackingDeg = degrees;
        trackingRateDegPerS = rateDegPerS;
    }

    /** Init loop only (mechanism periodic does not run before start): point at the tracking angle now. */
    public void applyTrackingNow() {
        command(chooseAngle(trackingDeg + trimDeg));
        estimate.reset(commandedDeg);
    }

    public void trim(double deltaDeg) {
        trimDeg += deltaDeg;
    }

    public void clearTrim() {
        trimDeg = 0;
    }

    // ---------------------------------------------------------------- state

    public boolean settled() {
        return Math.abs(commandedDeg - estimate.value()) <= SETTLE_TOLERANCE_DEG;
    }

    public double commandedDeg() {
        return commandedDeg;
    }

    public double estimatedDeg() {
        return estimate.value();
    }

    public double trimDeg() {
        return trimDeg;
    }

    public Mode mode() {
        return mode;
    }

    public double minDeg() {
        return Math.min(degreesAt(0), degreesAt(1)) + LIMIT_MARGIN_DEG;
    }

    public double maxDeg() {
        return Math.max(degreesAt(0), degreesAt(1)) - LIMIT_MARGIN_DEG;
    }

    public static double degreesAt(double position) {
        return (position - CENTER_POSITION) * DEG_PER_POSITION;
    }

    public static double positionAt(double degrees) {
        return CENTER_POSITION + degrees / DEG_PER_POSITION;
    }

    // ---------------------------------------------------------------- internals

    /**
     * Equivalent angle (+-360) inside the range, closest to where the turret is.
     * Just past a limit on the current side: wait at the limit instead of swinging all the way around.
     */
    private double chooseAngle(double degrees) {
        double min = minDeg();
        double max = maxDeg();
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double candidate = degrees + 360.0 * k;
            if (candidate >= min && candidate <= max
                    && (Double.isNaN(best) || Math.abs(candidate - commandedDeg) < Math.abs(best - commandedDeg))) {
                best = candidate;
            }
        }
        for (int k = -1; k <= 1; k++) {
            double candidate = degrees + 360.0 * k;
            boolean pastMax = candidate > max && candidate <= max + WRAP_HYSTERESIS_DEG;
            boolean pastMin = candidate < min && candidate >= min - WRAP_HYSTERESIS_DEG;
            if ((pastMax && commandedDeg > 0) || (pastMin && commandedDeg < 0)) {
                return MathUtil.clamp(candidate, min, max);
            }
        }
        return Double.isNaN(best) ? MathUtil.clamp(degrees, min, max) : best;
    }

    private void command(double degrees) {
        commandedDeg = degrees;
        writePosition(MathUtil.clamp(positionAt(degrees), 0, 1), true, true);
    }

    /** Enable/disable only on change (each is a hub command), then send the position to the enabled servos. */
    private void writePosition(double position, boolean useLeft, boolean useRight) {
        if (useLeft != leftEnabled) {
            if (useLeft) {
                left.enable();
            } else {
                left.disable();
            }
            leftEnabled = useLeft;
        }
        if (useRight != rightEnabled) {
            if (useRight) {
                right.enable();
            } else {
                right.disable();
            }
            rightEnabled = useRight;
        }
        if (useLeft) {
            left.setPosition(position);
        }
        if (useRight) {
            right.setPosition(position);
        }
    }
}
