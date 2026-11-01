package org.firstinspires.ftc.teamcode.intake;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.Slew;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

/** Flower arm: two servos walked toward the target at SPEED_POS_PER_SEC. Mode-based: deploy/stow are plain calls. */
@Configurable
public class FlowerArm implements Mechanism {

    public static final String LEFT_SERVO = "flowerLeft";
    public static final String RIGHT_SERVO = "flowerRight";
    public static final double PWM_MIN_US = 500;
    public static final double PWM_MAX_US = 2500;

    public static double STOWED_POSITION = 1.0;
    public static double DEPLOYED_POSITION = 0.0;
    public static double SPEED_POS_PER_SEC = 0.7;

    private NextServo left;
    private NextServo right;
    private boolean deployed = false;
    private final Slew position = new Slew(STOWED_POSITION);
    /** False until the first periodic() of this OpMode: the first command jumps straight to the target. */
    private boolean started = false;

    public FlowerArm() {
        bindServos();
    }

    /** Fresh servo objects each OpMode (the SDK resets PWM ranges between OpModes). */
    public void bindServos() {
        left = new NextServo(LEFT_SERVO);
        right = new NextServo(RIGHT_SERVO, 0.01, NextMotor.Direction.REVERSE);
        left.setPwmRange(PWM_MIN_US, PWM_MAX_US);
        right.setPwmRange(PWM_MIN_US, PWM_MAX_US);
        started = false;
    }

    public void reset() {
        deployed = false;
    }

    @Override
    public void periodic() {
        double target = deployed ? DEPLOYED_POSITION : STOWED_POSITION;
        double commanded;
        if (!started || SPEED_POS_PER_SEC <= 0) {
            position.reset(target);
            started = true;
            commanded = target;
        } else {
            commanded = position.step(target, SPEED_POS_PER_SEC, System.nanoTime());
        }
        left.setPosition(commanded);
        right.setPosition(commanded);
    }

    public void deploy() {
        deployed = true;
    }

    public void stow() {
        deployed = false;
    }

    public void toggle() {
        deployed = !deployed;
    }

    public boolean isDeployed() {
        return deployed;
    }
}
