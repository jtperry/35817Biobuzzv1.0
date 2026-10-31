package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Goal;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Main driver OpMode. */
@NextTeleop(name = "Main TeleOp")
public class MainTeleOp extends NextOpMode {

    private final Robot robot;
    private final CommandGamepad driver;
    private final CommandGamepad operator;

    /** Got alliance + pose from auto. */
    private final boolean fromAuto;

    public MainTeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        // Telemetry to Panels + Driver Hub. Safe to add every OpMode.
        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());

        robot.bindHardware();
        // After auto: same alliance, start where auto end.
        // Else default pose, pick alliance in init.
        fromAuto = robot.hasAutoHandoff();
        if (!fromAuto) {
            robot.setGoal(Goal.A);
        }
        robot.initFollower(hardwareMap, robot.takeHandoffPose());
        // No reset here. Init no move servos. Reset in start().

        driver = new CommandGamepad(gamepad1);
        operator = new CommandGamepad(gamepad2);
        bindControls();
    }

    private void bindControls() {
        // Operator: press to reset pose to that spot.
        // B = right, X = left, dpad right = right-flower, dpad left = left-flower,
        // dpad up = top-flower, dpad down = bottom-flower.
        // Red = blue spots mirrored across x = 72 and y = 72 (180 deg turn).
        operator.b().onTrue(Commands.instant(() -> robot.resetPoseToSlot(0)));
        operator.dpadRight().onTrue(Commands.instant(() -> robot.resetPoseToSlot(1)));
        operator.dpadLeft().onTrue(Commands.instant(() -> robot.resetPoseToSlot(2)));
        operator.dpadUp().onTrue(Commands.instant(() -> robot.resetPoseToSlot(3)));
        operator.dpadDown().onTrue(Commands.instant(() -> robot.resetPoseToSlot(4)));
        operator.x().onTrue(Commands.instant(() -> robot.resetPoseToSlot(5)));

        // Hold = feed all balls. Release = gate closed, stages off.
        driver.rightTrigger().isOver(0.5)
                .whileTrue(ShootCommands.shootWhileHeld(robot))
                .onFalse(robot.intake.stopFeed());

        driver.leftBumper().whileTrue(robot.intake.intake());
        driver.rightBumper().whileTrue(robot.intake.eject());

        driver.a().onTrue(robot.intake.toggleFlower());

        // Not toggleOnTrue: spinUp() instant, nothing to cancel.
        driver.x().onTrue(Commands.conditional(
                robot.shooter::isArmed,
                robot.shooter.idleFlywheel(),
                robot.shooter.spinUp()));

        driver.dpadLeft().onTrue(robot.shooter.trimTurret(-Constants.TURRET_TRIM_STEP_DEG));
        driver.dpadRight().onTrue(robot.shooter.trimTurret(Constants.TURRET_TRIM_STEP_DEG));

        driver.dpadUp().onTrue(robot.shooter.trimHood(Constants.HOOD_TRIM_STEP));
        driver.dpadDown().onTrue(robot.shooter.trimHood(-Constants.HOOD_TRIM_STEP));

        // Clear turret override + trims.
        driver.back().onTrue(robot.shooter.resumeAutoAim());
    }

    @Override
    public void disabledPeriodic() {
        // Triggers no poll in init. Read gamepad direct.
        // Only when no auto handoff.
        if (!fromAuto) {
            if (gamepad1.dpad_left) {
                robot.setAlliance(Alliance.BLUE);
            } else if (gamepad1.dpad_right) {
                robot.setAlliance(Alliance.RED);
            }
            Telemetry.log("alliance", robot.alliance() + "   (dpad left = BLUE, right = RED)");
            Telemetry.log("start pose", "default");
        } else {
            Telemetry.log("alliance", robot.alliance() + "   (from auto)");
            Telemetry.log("start pose", "where auto ended");
        }
        // Init restart slot pick on operator pad (direct read, triggers no poll in init).
        // B = 0 right, X = 5 left, dpad right/left/up/down = right/left/top/bottom-flower.
        if (!fromAuto) {
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
            Pose selected = Constants.restartPose(robot.alliance(), robot.restartSlot());
            Telemetry.log("restart slot", robot.restartSlot() + " "
                    + Constants.RESTART_SLOT_NAMES[robot.restartSlot()]
                    + " (opad B/X/dpad)");
            Telemetry.log("restart pose", String.format("%.1f, %.1f, %.0f deg",
                    selected.x(), selected.y(), Math.toDegrees(selected.heading())));
        }
    }

    @Override
    public void start() {
        // No auto: alliance + slot picked in init now. Start spot for that side.
        if (!fromAuto) {
            robot.resetPoseToStart();
        }
        // Servos to known state now, after play.
        robot.reset();
        // Flywheel stay at shoot speed whole match. Shot instant.
        robot.shooter.arm();
    }

    @Override
    public void periodic() {
        drive();
        robot.pickGoalBySide();
        pushTelemetry();
    }

    private void drive() {
        if (robot.follower() == null) {
            return;
        }

        // follower.update() run after this, in Robot.periodic().
        // Red driver stand other side. Flip x + y.
        double flip = robot.alliance() == Alliance.RED ? -1 : 1;
        DrivePowers powers = ManualDrive.fieldCentric(
                flip * gamepad1.left_stick_y,
                flip * gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robot.follower().pose().heading()
        );

        // driveOrHold: sticks released = hold spot, no coast.
        // Use follower.manual(powers) to coast.
        ManualDrive.driveOrHold(robot.follower(), powers);
    }

    private void pushTelemetry() {
        Telemetry.log("alliance", robot.alliance());
        Telemetry.log("pose reset", "opad B=right X=left dpad=flowers = slot " + robot.restartSlot()
                + " " + Constants.RESTART_SLOT_NAMES[robot.restartSlot()]);
        Pose pose = robot.pose();
        Telemetry.log("x", String.format("%.1f", pose.x()));
        Telemetry.log("y", String.format("%.1f", pose.y()));
        Telemetry.log("heading (deg)", String.format("%.1f", Math.toDegrees(pose.heading())));
        Telemetry.log("velocity x / y (in/s)", String.format("%.1f / %.1f", robot.velocity().vx, robot.velocity().vy));
        Telemetry.log("goal", robot.goal() + " at (" + Constants.goalX(robot.alliance(), robot.goal())
                + ", " + Constants.goalY(robot.alliance(), robot.goal()) + ")");
        Telemetry.log("distance to goal", String.format("%.1f", robot.shooter.distanceToGoal()));
        Telemetry.log("bearing to goal (deg)", String.format("%.1f", robot.shooter.bearingToGoal()));
        Telemetry.log("turret (deg)", robot.shooter.commandedTurretDeg());
        Telemetry.log("hood", robot.shooter.commandedHoodPosition());
        Telemetry.log("flywheel target", robot.shooter.targetRpm());
        Telemetry.log("battery (V)", String.format("%.2f", robot.batteryVoltage()));
        Telemetry.log("flywheel current", robot.shooter.currentRpm());
        Telemetry.log("at speed", robot.shooter.atSpeed());
        Telemetry.log("ready to shoot", robot.shooter.readyToShoot());
        Telemetry.log("turret settled", robot.shooter.turretSettled());
        Telemetry.log("gate open", robot.intake.isGateOpen());
        Telemetry.log("flower deployed", robot.intake.isFlowerDeployed());
    }
}
