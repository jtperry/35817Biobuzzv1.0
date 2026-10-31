package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.commands.Commands;

import com.bylazar.telemetry.PanelsTelemetry;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Shot tuner. Find RPM + hood for each distance. */
@NextTeleop(name = "Shot Tuner", group = "test")
public class ShotTuner extends NextOpMode {

    private static final double RPM_STEP = 100;
    private static final double HOOD_STEP = 0.01;

    private final Robot robot;
    private final CommandGamepad driver;

    private double rpm = 3000;
    private double hoodPosition = 0.2;

    /** Recorded points: distance, RPM, hood. */
    private final List<double[]> points = new ArrayList<>();

    public ShotTuner(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot.bindHardware();
        robot.initFollower(hardwareMap, Constants.DEFAULT_START_POSE);
        robot.reset();

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        // Hold = feed. Release = gate closed, stages off.
        driver.rightTrigger().isOver(0.5)
                .whileTrue(ShootCommands.shootWhileHeld(robot))
                .onFalse(robot.intake.stopFeed());

        driver.leftBumper().whileTrue(robot.intake.intake());
        driver.rightBumper().whileTrue(robot.intake.eject());

        driver.dpadUp().onTrue(Commands.instant(() -> rpm += RPM_STEP));
        driver.dpadDown().onTrue(Commands.instant(() -> rpm = Math.max(0, rpm - RPM_STEP)));
        driver.dpadRight().onTrue(Commands.instant(() -> adjustHood(HOOD_STEP)));
        driver.dpadLeft().onTrue(Commands.instant(() -> adjustHood(-HOOD_STEP)));

        driver.a().onTrue(Commands.instant(() ->
                points.add(new double[]{robot.shooter.distanceToGoal(), rpm, hoodPosition})));
        driver.b().onTrue(Commands.instant(() -> {
            if (!points.isEmpty()) {
                points.remove(points.size() - 1);
            }
        }));
    }

    private void adjustHood(double delta) {
        hoodPosition = Constants.clamp(hoodPosition + delta,
                Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION);
    }

    @Override
    public void disabledPeriodic() {
        // Triggers no poll in init. Read gamepad direct.
        if (gamepad1.dpad_left) {
            robot.setAlliance(Alliance.BLUE);
        } else if (gamepad1.dpad_right) {
            robot.setAlliance(Alliance.RED);
        }
        Telemetry.log("alliance", robot.alliance() + "   (dpad left = BLUE, right = RED)");
    }

    @Override
    public void start() {
        // Alliance picked in init. Start spot for that side.
        robot.resetPoseToStart();
        // Schedule once. Turret aim, RPM + hood from tuner values.
        robot.shooter.tuneShot(() -> rpm, () -> hoodPosition).schedule();
    }

    @Override
    public void periodic() {
        drive();
        robot.pickGoalBySide();
        pushTelemetry();
    }

    /** Same drive as Main TeleOp. */
    private void drive() {
        if (robot.follower() == null) {
            return;
        }
        // Red driver stand other side. Flip x + y.
        double flip = robot.alliance() == Alliance.RED ? -1 : 1;
        DrivePowers powers = ManualDrive.fieldCentric(
                flip * gamepad1.left_stick_y,
                flip * gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robot.follower().pose().heading()
        );
        ManualDrive.driveOrHold(robot.follower(), powers);
    }

    private void pushTelemetry() {
        Telemetry.log("RPM:    dpad up / down");
        Telemetry.log("hood:   dpad right / left");
        Telemetry.log("shoot:  hold RT   intake: LB   eject: RB");
        Telemetry.log("record: A   undo: B");
        Telemetry.log("");
        Telemetry.log("goal", robot.alliance() + " " + robot.goal());
        Telemetry.log("distance", String.format("%.1f", robot.shooter.distanceToGoal()));
        Telemetry.log("RPM target / actual", String.format("%.0f / %.0f", rpm, robot.shooter.currentRpm()));
        Telemetry.log("at speed", robot.shooter.atSpeed());
        Telemetry.log("hood", String.format("%.2f", hoodPosition));
        Telemetry.log("");
        Telemetry.log("recorded", points.size());
        for (int i = 0; i < points.size(); i++) {
            double[] p = points.get(i);
            Telemetry.log("point " + (i + 1), String.format("%.1f in   %.0f RPM   hood %.2f", p[0], p[1], p[2]));
        }
    }

    @Override
    public void end() {
        // Cut flywheel direct. Scheduler shutting down.
        robot.shooter.coastFlywheel();
    }
}
