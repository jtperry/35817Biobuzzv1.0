package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.ivy.commands.Commands;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Flower test. Tune flower stowed + deployed positions. */
@NextTeleop(name = "Flower Test", group = "test")
public class FlowerTest extends NextOpMode {

    private static final double FINE_STEP = 0.005;
    private static final double COARSE_STEP = 0.02;

    private final Robot robot;
    private final CommandGamepad driver;

    private double position = Constants.FLOWER_STOWED_POSITION;

    public FlowerTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot.bindHardware();
        // Drive on gamepad 2. Start pose default.
        robot.initFollower(hardwareMap, Constants.DEFAULT_START_POSE);

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        // Move flower.
        driver.dpadUp().onTrue(Commands.instant(() -> adjust(FINE_STEP)));
        driver.dpadDown().onTrue(Commands.instant(() -> adjust(-FINE_STEP)));
        driver.rightBumper().onTrue(Commands.instant(() -> adjust(COARSE_STEP)));
        driver.leftBumper().onTrue(Commands.instant(() -> adjust(-COARSE_STEP)));

        // Jump to saved deployed / stowed.
        driver.a().onTrue(Commands.instant(() -> position = Constants.FLOWER_DEPLOYED_POSITION));
        driver.b().onTrue(Commands.instant(() -> position = Constants.FLOWER_STOWED_POSITION));

        // Save current as deployed / stowed. Live only, copy into Constants.java.
        driver.x().onTrue(Commands.instant(() -> Constants.FLOWER_DEPLOYED_POSITION = position));
        driver.y().onTrue(Commands.instant(() -> Constants.FLOWER_STOWED_POSITION = position));
    }

    private void adjust(double delta) {
        position = Constants.clamp(position + delta, 0, 1);
    }

    @Override
    public void start() {
        // Schedule once. Own intake whole OpMode.
        robot.intake.manualControl(null, null, null, () -> position).schedule();
        // Hold shooter. No auto-aim, turret stay still.
        robot.shooter.standby().schedule();
    }

    @Override
    public void periodic() {
        // Gamepad 2 drive. SDK Y negative when up.
        robot.driveRobotCentric(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);
        Telemetry.log("drive: gamepad 2");
        Telemetry.log("move:  dpad U/D fine, bumpers coarse");
        Telemetry.log("jump:  A = deployed, B = stowed");
        Telemetry.log("save:  X = deployed, Y = stowed");
        Telemetry.log("");
        Telemetry.log("flower position", String.format("%.3f", position));
        Telemetry.log("FLOWER_DEPLOYED_POSITION", String.format("%.3f", Constants.FLOWER_DEPLOYED_POSITION));
        Telemetry.log("FLOWER_STOWED_POSITION", String.format("%.3f", Constants.FLOWER_STOWED_POSITION));
    }
}
