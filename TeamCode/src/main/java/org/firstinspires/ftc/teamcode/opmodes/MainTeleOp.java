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
        robot.reset();
        // Flywheel stay at shoot speed whole match. Shot instant.
        robot.shooter.arm();

        driver = new CommandGamepad(gamepad1);
        operator = new CommandGamepad(gamepad2);
        bindControls();
    }

    private void bindControls() {
        // Driver 2: robot in top right corner, press to reset pose there.
        operator.back().onTrue(Commands.instant(robot::resetPoseToStart));

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
        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robot.follower().pose().heading()
        );

        // driveOrHold: sticks released = hold spot, no coast.
        // Use follower.manual(powers) to coast.
        ManualDrive.driveOrHold(robot.follower(), powers);
    }

    private void pushTelemetry() {
        Telemetry.log("alliance", robot.alliance());
        Telemetry.log("pose reset", "driver 2 BACK, robot in top right corner");
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
        Telemetry.log("flywheel target / actual", robot.shooter.targetRpm() + " / " + robot.shooter.currentRpm());
        Telemetry.log("at speed", robot.shooter.atSpeed());
        Telemetry.log("ready to shoot", robot.shooter.readyToShoot());
        Telemetry.log("turret settled", robot.shooter.turretSettled());
        Telemetry.log("gate open", robot.intake.isGateOpen());
        Telemetry.log("flower deployed", robot.intake.isFlowerDeployed());
    }
}
