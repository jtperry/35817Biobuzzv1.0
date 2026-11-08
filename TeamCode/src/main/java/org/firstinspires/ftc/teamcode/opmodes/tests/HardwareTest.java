package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Raw control of every motor and servo for bring-up. Never drives. Gamepad 1:
 *   flywheel  dpad up/down = power +-0.05, Y = which motors (motor 1 only / motor 2 only / both), B = power 0
 *   stages    left stick Y = stage 1, right stick Y = stage 2 (push up = toward the shooter)
 *   flower    X toggle
 *   turret    dpad left/right = position -/+   (hold LB = coarse), RB = which servos (left only / right only / both)
 *   hood      LT / RT = position -/+           (hold LB = coarse)
 */
@NextTeleop(name = "Hardware Test", group = "test")
public class HardwareTest extends NextOpMode {

    private static final double POWER_STEP = 0.05;
    private static final double FINE_STEP = 0.005;
    private static final double COARSE_STEP = 0.05;
    private static final String[] MOTOR_CHOICES = {"motor 1 only", "motor 2 only", "both"};
    private static final String[] TURRET_CHOICES = {"left only", "right only", "both"};

    private final Robot robot;
    private final RobotTelemetry tele;
    private final CommandGamepad pad;

    private double flywheelPower = 0;
    /** Index into MOTOR_CHOICES. Starts with motor 1 alone: safest for the direction check. */
    private int motorChoice = 0;
    /** Index into TURRET_CHOICES. Starts with the left servo alone so the two cannot fight. */
    private int turretChoice = 0;
    private double turretPosition = Turret.centerPosition();
    private double hoodPosition = Hood.stowPosition();

    public HardwareTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);
        robot.bindHardware(hardwareMap);
        robot.initFollower(hardwareMap, Field.startPose(robot.alliance()));

        pad = new CommandGamepad(gamepad1);
        pad.dpadUp().onTrue(Commands.instant(() -> flywheelPower = MathUtil.clamp(flywheelPower + POWER_STEP, -1, 1)));
        pad.dpadDown().onTrue(Commands.instant(() -> flywheelPower = MathUtil.clamp(flywheelPower - POWER_STEP, -1, 1)));
        pad.b().onTrue(Commands.instant(() -> flywheelPower = 0));
        pad.y().onTrue(Commands.instant(() -> motorChoice = (motorChoice + 1) % MOTOR_CHOICES.length));
        pad.x().onTrue(Commands.instant(robot.flower::toggle));
        pad.dpadLeft().onTrue(Commands.instant(() -> turretPosition = MathUtil.clamp(turretPosition - step(), 0, 1)));
        pad.dpadRight().onTrue(Commands.instant(() -> turretPosition = MathUtil.clamp(turretPosition + step(), 0, 1)));
        pad.rightBumper().onTrue(Commands.instant(() -> {
            turretChoice = (turretChoice + 1) % TURRET_CHOICES.length;
            applyTurretChoice();
        }));
        pad.leftTrigger().isOver(0.5).onTrue(Commands.instant(() -> hoodPosition = MathUtil.clamp(hoodPosition - step(), 0, 1)));
        pad.rightTrigger().isOver(0.5).onTrue(Commands.instant(() -> hoodPosition = MathUtil.clamp(hoodPosition + step(), 0, 1)));
    }

    private double step() {
        return gamepad1.left_bumper ? COARSE_STEP : FINE_STEP;
    }

    /** Raw turret position on the chosen servos; the other one is disabled. */
    private void applyTurretChoice() {
        robot.turret.raw(() -> turretPosition, turretChoice != 1, turretChoice != 0);
    }

    @Override
    public void start() {
        robot.reset();
        applyTurretChoice();
        robot.hood.raw(() -> hoodPosition);
        robot.intake.manual(() -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y).schedule();
    }

    @Override
    public void periodic() {
        double power1 = motorChoice == 1 ? 0 : flywheelPower;
        double power2 = motorChoice == 0 ? 0 : flywheelPower;
        robot.flywheel.raw(power1, power2);

        if (!tele.beginFrame()) {
            return;
        }
        tele.add("flywheel", String.format("power %.2f  (%s)  %.0f RPM",
                flywheelPower, MOTOR_CHOICES[motorChoice], robot.flywheel.rpm()));
        tele.add("flywheel amps", String.format("motor 1 %.2f   motor 2 %.2f",
                robot.flywheel.motor1Amps(), robot.flywheel.motor2Amps()));
        tele.add("stages", String.format("1: %.2f (%.2f A)   2: %.2f (%.2f A)",
                -gamepad1.left_stick_y, robot.intake.stage1Amps(),
                -gamepad1.right_stick_y, robot.intake.stage2Amps()));
        tele.add("flower", robot.flower.isDeployed() ? "deployed" : "stowed");
        tele.add("turret", String.format("position %.4f = %.1f deg (current calibration), %s",
                turretPosition, Turret.degreesAt(turretPosition), TURRET_CHOICES[turretChoice]));
        tele.add("hood", String.format("position %.4f", hoodPosition));
        tele.add("OctoQuad bad reads", robot.flywheel.badReads());
        tele.line("Flywheel check: motor 1 alone and motor 2 alone at +0.2 must BOTH show positive RPM.");
        tele.endFrame();
    }

    @Override
    public void end() {
        robot.flywheel.reset();
    }
}
