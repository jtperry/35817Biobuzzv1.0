package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.SharedOctoQuadLocalizer;
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
 * During INIT it also reads the OctoQuad every loop and counts bad reads, so a bad read that starts with the reads
 * (cable, OctoQuad, its power) shows before PLAY, and one caused by something PLAY switches on shows only after.
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

    private long initStartNanos = 0;
    private int initReads = 0;
    private int initBadReads = 0;
    private double initFirstBadS = Double.NaN;
    private double initLastBadS = Double.NaN;
    private long playStartNanos = 0;
    private int playBadReadsSeen = 0;
    private double playFirstBadS = Double.NaN;
    private double playLastBadS = Double.NaN;

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

    /** INIT loop: read the OctoQuad (nothing else does before PLAY) and count reads whose checksum fails. */
    @Override
    public void disabledPeriodic() {
        SharedOctoQuadLocalizer localizer = robot.localizer();
        if (localizer == null) {
            return;
        }
        long now = System.nanoTime();
        if (initStartNanos == 0) {
            initStartNanos = now;
        }
        localizer.update();
        initReads++;
        if (!localizer.encoderData().crcOk) {
            initBadReads++;
            double t = (now - initStartNanos) / 1e9;
            if (Double.isNaN(initFirstBadS)) {
                initFirstBadS = t;
            }
            initLastBadS = t;
        }

        if (!tele.beginFrame()) {
            return;
        }
        tele.add("OctoQuad INIT reads", String.format("%d reads, %d bad", initReads, initBadReads));
        tele.add("INIT bad reads at", badTimes(initFirstBadS, initLastBadS) + " after INIT");
        tele.line("Wait a few seconds, then PLAY and compare with the bad reads after PLAY.");
        tele.endFrame();
    }

    private static String badTimes(double firstS, double lastS) {
        return Double.isNaN(firstS) ? "none" : String.format("first %.2f s, last %.2f s", firstS, lastS);
    }

    @Override
    public void start() {
        playStartNanos = System.nanoTime();
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

        int badReads = robot.flywheel.badReads();
        if (badReads > playBadReadsSeen) {
            double t = (System.nanoTime() - playStartNanos) / 1e9;
            if (Double.isNaN(playFirstBadS)) {
                playFirstBadS = t;
            }
            playLastBadS = t;
            playBadReadsSeen = badReads;
        }

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
        tele.add("OctoQuad bad reads", String.format("after PLAY %d (%s after PLAY); in INIT %d of %d",
                badReads, badTimes(playFirstBadS, playLastBadS), initBadReads, initReads));
        tele.line("Flywheel check: motor 1 alone and motor 2 alone at +0.2 must BOTH show positive RPM.");
        tele.endFrame();
    }

    @Override
    public void end() {
        robot.flywheel.reset();
    }
}
