package org.firstinspires.ftc.teamcode.intake;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Slew;

import java.util.function.DoubleSupplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

/** Flower arm: two servos walked toward the target at SPEED_POS_PER_SEC. Mode-based: deploy/stow/raw are plain calls. */
@Configurable
public class FlowerArm implements Mechanism {

    public static final String LEFT_SERVO = "flowerLeft";
    public static final String RIGHT_SERVO = "flowerRight";
    public static final double PWM_MIN_US = 500;
    public static final double PWM_MAX_US = 2500;
    /** Below the Servo Tuner's 0.002 fine step, so every jog reaches the servos. */
    private static final double CACHE_TOLERANCE = 0.001;
    /** atTarget() tolerance: Slew's last step lands on the target only up to rounding. */
    private static final double AT_TARGET_EPS = 1e-9;

    /** Servo directions, read in bindServos(): flip live during bring-up, re-init the OpMode, then copy into code. */
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = true;

    public static double STOWED_POSITION = 1.0;
    public static double DEPLOYED_POSITION = 0.0;
    public static double SPEED_POS_PER_SEC = 0.7;

    private NextServo left;
    private NextServo right;
    private boolean deployed = false;
    private final Slew position = new Slew(STOWED_POSITION);
    /** False until the first periodic() of this OpMode: the first command jumps straight to the target. */
    private boolean started = false;
    private boolean rawMode = false;
    private DoubleSupplier rawPosition = () -> STOWED_POSITION;
    private boolean rawLeft = true;
    private boolean rawRight = true;
    private boolean leftEnabled = true;
    private boolean rightEnabled = true;

    public FlowerArm() {
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
        started = false;
    }

    /** Stowed, leaves RAW and enables both servos again (a servo RAW disabled would otherwise stay dead). No hardware call when both are already enabled. */
    public void reset() {
        deployed = false;
        rawMode = false;
        setEnabled(true, true);
    }

    @Override
    public void periodic() {
        if (rawMode) {
            double raw = MathUtil.clamp(rawPosition.getAsDouble(), 0, 1);
            writePosition(raw, rawLeft, rawRight);
            // Leaving RAW walks the arm from here instead of jumping.
            position.reset(raw);
            started = true;
            return;
        }
        writePosition(step(System.nanoTime()), true, true);
    }

    /**
     * Walk toward the deploy/stow target and return the position to command now. The first call of an OpMode (and any
     * call with SPEED_POS_PER_SEC <= 0) jumps straight to the target. Package-private so a JVM test can drive the walk:
     * periodic() also writes the servos, which needs hardware.
     */
    double step(long nowNanos) {
        double target = target();
        if (!started || SPEED_POS_PER_SEC <= 0) {
            position.reset(target);
            started = true;
            return target;
        }
        return position.step(target, SPEED_POS_PER_SEC, nowNanos);
    }

    /**
     * True once the walk has reached the current deploy/stow target, so the arm is where deploy() or stow() sent it.
     * False in RAW and before the first periodic() of this OpMode. Autos wait on it instead of a fixed time.
     */
    public boolean atTarget() {
        return !rawMode && started && Math.abs(position.value() - target()) <= AT_TARGET_EPS;
    }

    private double target() {
        return deployed ? DEPLOYED_POSITION : STOWED_POSITION;
    }

    /** Raw position on the chosen servos only; the other is disabled (no PWM) so the two cannot fight. */
    public void raw(DoubleSupplier position, boolean useLeft, boolean useRight) {
        rawPosition = position;
        rawLeft = useLeft;
        rawRight = useRight;
        rawMode = true;
    }

    /** deploy, stow and toggle leave RAW: the arm walks on from the last raw position (periodic() keeps it in sync). */
    public void deploy() {
        deployed = true;
        rawMode = false;
    }

    public void stow() {
        deployed = false;
        rawMode = false;
    }

    public void toggle() {
        deployed = !deployed;
        rawMode = false;
    }

    public boolean isDeployed() {
        return deployed;
    }

    public static double stowedPosition() {
        return STOWED_POSITION;
    }

    public static double deployedPosition() {
        return DEPLOYED_POSITION;
    }

    private void writePosition(double value, boolean useLeft, boolean useRight) {
        setEnabled(useLeft, useRight);
        if (useLeft) {
            left.setPosition(value);
        }
        if (useRight) {
            right.setPosition(value);
        }
    }

    /** Enable/disable only on change (each is a hub command). */
    private void setEnabled(boolean useLeft, boolean useRight) {
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
    }
}
