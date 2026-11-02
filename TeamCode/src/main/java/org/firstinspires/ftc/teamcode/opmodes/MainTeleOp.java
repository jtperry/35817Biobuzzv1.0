package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.input.DriverInput;
import org.firstinspires.ftc.teamcode.shooter.ShotSolution;
import org.firstinspires.ftc.teamcode.util.FieldDrawing;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Main driver OpMode. Buttons by position, so Xbox and PS pads behave the same. */
@NextTeleop(name = "Main TeleOp")
public class MainTeleOp extends NextOpMode {

    private final Robot robot;
    private final RobotTelemetry tele;
    private final CommandGamepad driver;
    private final CommandGamepad operator;

    /** Alliance and pose came from auto. */
    private final boolean fromAuto;

    public MainTeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);

        robot.bindHardware(hardwareMap);
        fromAuto = robot.hasAutoHandoff();
        if (!fromAuto) {
            robot.setCell(HiveCell.A);
        }
        robot.initFollower(hardwareMap, robot.takeHandoffPose());
        // No reset here: init never moves servos. Reset in start().

        driver = new CommandGamepad(gamepad1);
        operator = new CommandGamepad(gamepad2);
        bindControls();
    }

    private void bindControls() {
        // Operator: reset pose to a restart slot. B = right, X = left, dpad = the four flower slots.
        operator.b().onTrue(Commands.instant(() -> robot.resetPoseToSlot(0)));
        operator.dpadRight().onTrue(Commands.instant(() -> robot.resetPoseToSlot(1)));
        operator.dpadLeft().onTrue(Commands.instant(() -> robot.resetPoseToSlot(2)));
        operator.dpadUp().onTrue(Commands.instant(() -> robot.resetPoseToSlot(3)));
        operator.dpadDown().onTrue(Commands.instant(() -> robot.resetPoseToSlot(4)));
        operator.x().onTrue(Commands.instant(() -> robot.resetPoseToSlot(5)));

        // Hold RT = feed, gated per ball. Release = gate closed, stages off.
        driver.rightTrigger().isOver(0.5)
                .whileTrue(ShootCommands.whileHeld(robot))
                .onFalse(robot.intake.stopFeed());
        driver.leftBumper().whileTrue(robot.intake.intake());
        driver.rightBumper().whileTrue(robot.intake.eject());
        driver.a().onTrue(Commands.instant(robot.flower::toggle));
        driver.x().onTrue(Commands.instant(robot.flywheel::toggleArmed));

        // Trims, same directions as v1.
        driver.dpadLeft().onTrue(Commands.instant(() -> robot.turret.trimStep(-1)));
        driver.dpadRight().onTrue(Commands.instant(() -> robot.turret.trimStep(1)));
        driver.dpadUp().onTrue(Commands.instant(() -> robot.hood.trimStep(1)));
        driver.dpadDown().onTrue(Commands.instant(() -> robot.hood.trimStep(-1)));
        driver.back().onTrue(Commands.instant(() -> {
            robot.turret.clearTrim();
            robot.hood.clearTrim();
        }));
    }

    @Override
    public void disabledPeriodic() {
        // Triggers do not poll in init: read the pads directly. Only when there was no auto.
        if (!fromAuto) {
            if (gamepad1.dpad_left) {
                robot.setAlliance(Alliance.BLUE);
            } else if (gamepad1.dpad_right) {
                robot.setAlliance(Alliance.RED);
            }
            if (gamepad2.b) {
                robot.setRestartSlot(0);
            } else if (gamepad2.dpad_right) {
                robot.setRestartSlot(1);
            } else if (gamepad2.dpad_left) {
                robot.setRestartSlot(2);
            } else if (gamepad2.dpad_up) {
                robot.setRestartSlot(3);
            } else if (gamepad2.dpad_down) {
                robot.setRestartSlot(4);
            } else if (gamepad2.x) {
                robot.setRestartSlot(5);
            }
        }
        if (!tele.beginFrame()) {
            return;
        }
        if (fromAuto) {
            tele.add("alliance", robot.alliance() + "   (from auto)");
            tele.add("start pose", "where auto ended");
        } else {
            tele.add("alliance", robot.alliance() + "   (dpad left = BLUE, right = RED)");
            tele.add("restart slot", robot.restartSlot() + " " + Field.RESTART_SLOT_NAMES[robot.restartSlot()]
                    + "   (gamepad 2: B / X / dpad)");
            tele.add("restart pose", formatPose(Field.restartPose(robot.alliance(), robot.restartSlot())));
        }
        tele.endFrame();
    }

    @Override
    public void start() {
        // No auto: alliance + slot picked in init. Start there.
        if (!fromAuto) {
            robot.resetPoseToStart();
        }
        // Servos to a known state now, after play.
        robot.reset();
        // Flywheel at shot speed the whole match.
        robot.flywheel.arm();
    }

    @Override
    public void periodic() {
        drive();
        robot.pickCellBySide();
        pushTelemetry();
    }

    private void drive() {
        // Red driver stands on the other side: flip both axes. Stick y is negative when pushed up.
        double flip = robot.alliance() == Alliance.RED ? -1 : 1;
        double scale = DriverInput.stickScale(gamepad1.left_stick_x, gamepad1.left_stick_y);
        DrivePowers powers = ManualDrive.fieldCentric(
                flip * gamepad1.left_stick_y * scale,
                flip * gamepad1.left_stick_x * scale,
                -DriverInput.turn(gamepad1.right_stick_x),
                robot.pose().heading());
        // Sticks released: hold the spot instead of coasting.
        ManualDrive.driveOrHold(robot.follower(), powers);
    }

    private void pushTelemetry() {
        if (!tele.beginFrame()) {
            return;
        }
        Pose pose = robot.pose();
        ShotSolution shot = robot.shot();
        Velocity velocity = robot.velocity();
        tele.add("alliance / cell", robot.alliance() + " / " + robot.cell());
        tele.add("pose", formatPose(pose));
        tele.add("ready", robot.readyToShoot());
        tele.add("flywheel", String.format("%s  %.0f / %.0f RPM",
                robot.flywheel.mode(), robot.flywheel.rpm(), robot.flywheel.targetRpm()));
        tele.detail("distance (in)", String.format("%.1f", shot.distanceIn));
        tele.detail("turret (deg)", String.format("cmd %.1f  est %.1f  trim %.1f",
                robot.turret.commandedDeg(), robot.turret.estimatedDeg(), robot.turret.trimDeg()));
        tele.detail("hood", String.format("%.3f  trim %.3f", robot.hood.commanded(), robot.hood.trim()));
        tele.detail("velocity x / y (in/s)", String.format("%.1f / %.1f", velocity.vx, velocity.vy));
        tele.detail("release speed (in/s)", String.format("%.1f", shot.releaseSpeed));
        tele.detail("battery (V)", String.format("%.2f", robot.battery.volts()));
        tele.detail("gate / flower", (robot.intake.isGateOpen() ? "open" : "closed") + " / "
                + (robot.flower.isDeployed() ? "deployed" : "stowed"));
        tele.detail("restart slot", robot.restartSlot() + " " + Field.RESTART_SLOT_NAMES[robot.restartSlot()]);
        FieldDrawing.robot(pose);
        tele.endFrame();
    }

    private static String formatPose(Pose pose) {
        return String.format("%.1f, %.1f, %.1f deg",
                pose.x(), pose.y(), MathUtil.wrapDegrees(Math.toDegrees(pose.heading())));
    }
}
