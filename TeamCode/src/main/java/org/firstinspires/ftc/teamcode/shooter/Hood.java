package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Slew;

import java.util.function.DoubleSupplier;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

/** Hood servo (spec 5.3): table position + driver trim, with a slew estimate so "settled" covers the hood too. */
@Configurable
public class Hood implements Mechanism {

    public static final String SERVO = "hood";
    private static final double CACHE_TOLERANCE = 0.001;

    public static double MIN_POSITION = 0.0254;
    public static double MAX_POSITION = 0.45;
    /** Inside the range; below MIN the hood slams its hard stop. */
    public static double STOW_POSITION = 0.0254;
    public static double TRIM_STEP = 0.01;
    /** Hood servo speed for the settle estimate, position per second. */
    public static double SLEW_POS_PER_SEC = 1.5;
    public static double SETTLE_TOLERANCE = 0.01;

    public enum Mode { TRACK, HOLD, RAW }

    private NextServo servo;
    private Mode mode = Mode.TRACK;
    private double trackingPosition = STOW_POSITION;
    private double holdPosition = STOW_POSITION;
    private DoubleSupplier rawPosition = () -> STOW_POSITION;
    private double trim = 0;
    private double commanded = STOW_POSITION;
    /** Where the hood probably is: chases the command at SLEW_POS_PER_SEC. */
    private final Slew estimate = new Slew(STOW_POSITION);

    public Hood() {
        bindServos();
    }

    /** Fresh servo object each OpMode. */
    public void bindServos() {
        servo = new NextServo(SERVO, CACHE_TOLERANCE);
    }

    /** Known state: stowed now, then tracking. */
    public void reset() {
        mode = Mode.TRACK;
        trim = 0;
        command(STOW_POSITION);
        estimate.reset(commanded);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case TRACK:
                command(MathUtil.clamp(trackingPosition + trim, MIN_POSITION, MAX_POSITION));
                break;
            case HOLD:
                command(MathUtil.clamp(holdPosition, 0, 1));
                break;
            case RAW:
                command(MathUtil.clamp(rawPosition.getAsDouble(), 0, 1));
                break;
        }
        estimate.step(commanded, SLEW_POS_PER_SEC, System.nanoTime());
    }

    public void track() {
        mode = Mode.TRACK;
    }

    public void hold(double position) {
        holdPosition = position;
        mode = Mode.HOLD;
    }

    public void stow() {
        hold(STOW_POSITION);
    }

    public void raw(DoubleSupplier position) {
        rawPosition = position;
        mode = Mode.RAW;
    }

    /** Robot sets this every loop from the shot table. */
    public void setTracking(double position) {
        trackingPosition = position;
    }

    public void trim(double delta) {
        trim += delta;
    }

    public void clearTrim() {
        trim = 0;
    }

    public boolean settled() {
        return Math.abs(commanded - estimate.value()) <= SETTLE_TOLERANCE;
    }

    public double commanded() {
        return commanded;
    }

    public double trim() {
        return trim;
    }

    public Mode mode() {
        return mode;
    }

    private void command(double position) {
        commanded = position;
        servo.setPosition(position);
    }
}
