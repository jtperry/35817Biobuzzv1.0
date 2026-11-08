package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.intake.FlowerArm;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import java.util.Locale;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Finds the flower and hood positions (spec 11). Never drives. Both servos hold their jog positions.
 * Gamepad 1:
 *   Y = next servo (flower / hood)
 *   dpad left/right = jog the selected servo (hold LB = coarse)
 *   RB = flower servos (left only / right only / both)
 *   flower  A = mark deployed  B = mark stowed
 *   hood    A = mark min       B = mark max       X = mark stow
 * Then copy the printed lines into FlowerArm.java and Hood.java.
 */
@NextTeleop(name = "Servo Tuner", group = "test")
public class ServoTuner extends NextOpMode {

    private static final double FINE_STEP = 0.002;
    private static final double COARSE_STEP = 0.02;
    private static final int FLOWER = 0;
    private static final int HOOD = 1;
    private static final String[] SERVOS = {"flower", "hood"};
    private static final String[] MARKS = {"A = deployed, B = stowed", "A = min, B = max, X = stow"};
    private static final String[] FLOWER_CHOICES = {"left only", "right only", "both"};

    private final Robot robot;
    private final RobotTelemetry tele;
    private final CommandGamepad pad;

    private int selected = FLOWER;
    /** Index into FLOWER_CHOICES. Starts with the left servo alone so the two cannot fight. */
    private int flowerChoice = 0;
    private double flowerPosition = FlowerArm.stowedPosition();
    private double hoodPosition = Hood.stowPosition();

    private double flowerDeployed = Double.NaN;
    private double flowerStowed = Double.NaN;
    private double hoodMin = Double.NaN;
    private double hoodMax = Double.NaN;
    private double hoodStow = Double.NaN;

    public ServoTuner(Robot robot) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);
        robot.bindHardware(hardwareMap);
        robot.initFollower(hardwareMap, Field.startPose(robot.alliance()));

        pad = new CommandGamepad(gamepad1);
        pad.y().onTrue(Commands.instant(() -> selected = (selected + 1) % SERVOS.length));
        pad.dpadLeft().onTrue(Commands.instant(() -> jog(-step())));
        pad.dpadRight().onTrue(Commands.instant(() -> jog(step())));
        pad.rightBumper().onTrue(Commands.instant(() -> {
            flowerChoice = (flowerChoice + 1) % FLOWER_CHOICES.length;
            applyFlowerChoice();
        }));
        pad.a().onTrue(Commands.instant(this::markA));
        pad.b().onTrue(Commands.instant(this::markB));
        pad.x().onTrue(Commands.instant(() -> {
            if (selected == HOOD) {
                hoodStow = hoodPosition;
            }
        }));
    }

    private double step() {
        return gamepad1.left_bumper ? COARSE_STEP : FINE_STEP;
    }

    private void jog(double delta) {
        if (selected == FLOWER) {
            flowerPosition = MathUtil.clamp(flowerPosition + delta, 0, 1);
        } else {
            hoodPosition = MathUtil.clamp(hoodPosition + delta, 0, 1);
        }
    }

    private void markA() {
        if (selected == FLOWER) {
            flowerDeployed = flowerPosition;
        } else {
            hoodMin = hoodPosition;
        }
    }

    private void markB() {
        if (selected == FLOWER) {
            flowerStowed = flowerPosition;
        } else {
            hoodMax = hoodPosition;
        }
    }

    /** Raw flower position on the chosen servos; the other one is disabled. */
    private void applyFlowerChoice() {
        robot.flower.raw(() -> flowerPosition, flowerChoice != 1, flowerChoice != 0);
    }

    @Override
    public void start() {
        robot.flywheel.off();
        robot.turret.hold(0);
        robot.hood.raw(() -> hoodPosition);
        applyFlowerChoice();
    }

    @Override
    public void periodic() {
        if (!tele.beginFrame()) {
            return;
        }
        tele.add("selected", SERVOS[selected] + "   (Y = next)");
        tele.add("marks", MARKS[selected]);
        tele.add("flower", String.format("%.4f   %s   (RB = change)", flowerPosition, FLOWER_CHOICES[flowerChoice]));
        tele.add("hood", String.format("%.4f", hoodPosition));
        paste("FlowerArm.java", "DEPLOYED_POSITION", flowerDeployed);
        paste("FlowerArm.java", "STOWED_POSITION", flowerStowed);
        pasteHood();
        tele.endFrame();
    }

    /**
     * Hood clamps every position to [MIN_POSITION, MAX_POSITION], so once both limits are marked MIN is the smaller
     * mark and MAX the larger. A single mark prints under its own name (min/max with the unmarked NaN would hide it).
     * The marks are compared as printed (4 decimals), so the warnings always agree with the paste lines.
     */
    private void pasteHood() {
        double min = asPrinted(hoodMin);
        double max = asPrinted(hoodMax);
        double stow = asPrinted(hoodStow);
        boolean bothLimits = !Double.isNaN(min) && !Double.isNaN(max);
        double low = bothLimits ? Math.min(min, max) : min;
        double high = bothLimits ? Math.max(min, max) : max;
        if (bothLimits && min > max) {
            tele.add("hood marks", "marked in reverse order: the lines below are swapped");
        } else if (bothLimits && min == max) {
            tele.add("hood marks", "MIN equals MAX: the hood cannot move");
        }
        if (bothLimits && !Double.isNaN(stow) && (stow < low || stow > high)) {
            tele.add("hood marks", "stow is outside the limits: Hood clamps it to the nearer limit");
        }
        paste("Hood.java", "MIN_POSITION", low);
        paste("Hood.java", "MAX_POSITION", high);
        paste("Hood.java", "STOW_POSITION", stow);
    }

    /** One paste-ready line once a value is marked. Locale.US: Java needs a dot as the decimal point. */
    private void paste(String file, String name, double value) {
        if (!Double.isNaN(value)) {
            tele.add(file, String.format(Locale.US, "public static double %s = %.4f;", name, value));
        }
    }

    /**
     * The value paste() prints (rounded by the same format), NaN kept: jogs add up with rounding error, so marks that
     * print the same can differ in the 17th decimal.
     */
    private static double asPrinted(double value) {
        return Double.isNaN(value) ? value : Double.parseDouble(String.format(Locale.US, "%.4f", value));
    }

    @Override
    public void end() {
        robot.flower.reset();
    }
}
