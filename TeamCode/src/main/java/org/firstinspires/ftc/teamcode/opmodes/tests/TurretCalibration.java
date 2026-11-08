package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import java.util.Locale;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Measures the turret mapping (spec 5.2). Never drives. Gamepad 1:
 *   dpad left/right = jog servo position (hold LB = coarse); also leaves a hold or sweep
 *   A = record BACK (shooter points at the robot's back: the middle of travel)
 *   X = record LEFT 90 (points out the robot's left side)
 *   B = record RIGHT 90 (points out the robot's right side, 270 deg)
 *   Y = apply: sets CENTER_POSITION and DEG_PER_POSITION live, turret goes to 180 deg (back)
 *   RB = check sweep 180, 90, 270, 0, 180 using the applied numbers
 *   back = back to jogging
 * Then copy the two printed lines into Turret.java.
 */
@NextTeleop(name = "Turret Calibration", group = "test")
public class TurretCalibration extends NextOpMode {

    private static final double FINE_STEP = 0.002;
    private static final double COARSE_STEP = 0.02;
    private static final double SWEEP_STEP_S = 1.5;
    private static final double[] SWEEP_DEG = {180, 90, 270, 0, 180};

    private final Robot robot;
    private final RobotTelemetry tele;
    private final CommandGamepad pad;

    private double position = Turret.centerPosition();
    private double back = Double.NaN;
    private double left = Double.NaN;
    private double right = Double.NaN;
    private boolean applied = false;
    private boolean sweeping = false;
    private long sweepStartNanos = 0;

    public TurretCalibration(Robot robot) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);
        robot.bindHardware(hardwareMap);
        robot.initFollower(hardwareMap, Field.startPose(robot.alliance()));

        pad = new CommandGamepad(gamepad1);
        pad.dpadLeft().onTrue(Commands.instant(() -> jogBy(-step())));
        pad.dpadRight().onTrue(Commands.instant(() -> jogBy(step())));
        pad.a().onTrue(Commands.instant(() -> back = position));
        pad.x().onTrue(Commands.instant(() -> left = position));
        pad.b().onTrue(Commands.instant(() -> right = position));
        pad.y().onTrue(Commands.instant(this::apply));
        pad.rightBumper().onTrue(Commands.instant(this::startSweep));
        pad.back().onTrue(Commands.instant(this::jog));
    }

    private double step() {
        return gamepad1.left_bumper ? COARSE_STEP : FINE_STEP;
    }

    private boolean allRecorded() {
        return !Double.isNaN(back) &&!Double.isNaN(left) && !Double.isNaN(right) && left != right;
    }

    private double degPerPosition() {
        return 180.0 / (right - left);
    }

    private void apply() {
        if (!allRecorded()) {
            return;
        }
        Turret.setCalibration(back, degPerPosition());
        applied = true;
        sweeping = false;
        robot.turret.hold(Turret.CENTER_DEG);
    }

    private void startSweep() {
        if (!applied) {
            return;
        }
        sweeping = true;
        sweepStartNanos = System.nanoTime();
    }

    private void jog() {
        sweeping = false;
        robot.turret.raw(() -> position);
    }

    /** Dpad: leave a hold or sweep first (back to jogging), so the turret always shows the position being changed. */
    private void jogBy(double delta) {
        if (robot.turret.mode() != Turret.Mode.RAW) {
            jog();
        }
        position = MathUtil.clamp(position + delta, 0, 1);
    }

    @Override
    public void start() {
        robot.reset();
        jog();
    }

    @Override
    public void periodic() {
        if (sweeping) {
            int index = (int) ((System.nanoTime() - sweepStartNanos) / 1e9 / SWEEP_STEP_S);
            if (index >= SWEEP_DEG.length) {
                sweeping = false;
                robot.turret.hold(Turret.CENTER_DEG);
            } else {
                robot.turret.hold(SWEEP_DEG[index]);
            }
        }

        if (!tele.beginFrame()) {
            return;
        }
        tele.add("jog position", String.format("%.4f", position));
        tele.add("back / left / right", String.format("%.4f / %.4f / %.4f", back, left, right));
        if (allRecorded()) {
            double degPerPosition = degPerPosition();
            double midpointErrorDeg = (back - (left + right) / 2) * degPerPosition;
            tele.add("paste", String.format(Locale.US, "public static double CENTER_POSITION = %.4f;", back));
            tele.add("paste", String.format(Locale.US, "public static double DEG_PER_POSITION = %.1f;", degPerPosition));
            tele.add("back mark vs left/right midpoint", String.format("%.1f deg (should be near 0)", midpointErrorDeg));
            tele.add("range", String.format("%.0f to %.0f deg (CAD predicts about 403 total)",
                    Turret.CENTER_DEG + (0 - back) * degPerPosition, Turret.CENTER_DEG + (1 - back) * degPerPosition));
        }
        tele.add("applied", applied ? String.format("yes: turret %.1f deg", robot.turret.commandedDeg()) : "no (Y)");
        tele.add("sweeping", sweeping);
        tele.endFrame();
    }
}
