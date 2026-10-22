package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.ivy.commands.Commands;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Shooter test. Raw control flywheel, turret, hood. */
@NextTeleop(name = "Shooter Test", group = "test")
public class ShooterTest extends NextOpMode {

    private static final double FINE_STEP = 0.005;
    private static final double COARSE_STEP = 0.02;
    private static final double POWER_STEP = 0.05;

    private final Robot robot;
    private final CommandGamepad driver;

    private double power = 0;

    /** Last loop time. Scale stick by time. */
    private long lastLoopNanos;
    private double turretPosition = Constants.TURRET_CENTER_POSITION;
    private double hoodPosition = Constants.HOOD_STOWED_POSITION;

    private Double centerMark = null;
    private Double referenceMark = null;

    public ShooterTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        // Telemetry to Panels + Driver Hub. Safe to add every OpMode.
        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());

        // Re-apply servo range + direction. SDK reset them each OpMode.
        robot.bindHardware();
        // Drive on gamepad 2. Start pose default.
        robot.initFollower(hardwareMap, Constants.DEFAULT_START_POSE);

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        // No mechanism needed. No interrupt manualControl.

        // Flywheel.
        driver.rightTrigger().isOver(0.5).onTrue(Commands.instant(() -> adjustPower(POWER_STEP)));
        driver.leftTrigger().isOver(0.5).onTrue(Commands.instant(() -> adjustPower(-POWER_STEP)));
        driver.b().onTrue(Commands.instant(() -> power = 0));

        // Turret. Small steps near ends, servo on hardstop burn.
        driver.dpadLeft().onTrue(Commands.instant(() -> adjustTurret(-FINE_STEP)));
        driver.dpadRight().onTrue(Commands.instant(() -> adjustTurret(FINE_STEP)));
        driver.leftBumper().onTrue(Commands.instant(() -> adjustTurret(-COARSE_STEP)));
        driver.rightBumper().onTrue(Commands.instant(() -> adjustTurret(COARSE_STEP)));

        // Hood.
        driver.dpadDown().onTrue(Commands.instant(() -> adjustHood(-FINE_STEP)));
        driver.dpadUp().onTrue(Commands.instant(() -> adjustHood(FINE_STEP)));
        driver.a().onTrue(Commands.instant(() -> adjustHood(-COARSE_STEP)));
        driver.y().onTrue(Commands.instant(() -> adjustHood(COARSE_STEP)));

        // Calibration marks. X = center, BACK = reference.
        // Predicted angle off from protractor = fix TURRET_SERVO_TRAVEL_DEG.
        driver.x().onTrue(Commands.instant(() -> centerMark = turretPosition));
        driver.back().onTrue(Commands.instant(() -> referenceMark = turretPosition));
    }

    /** Stick control. Left X = turret, right Y = hood. */
    private void driveFromSticks() {
        long now = System.nanoTime();
        // Cap. Slow loop no big jump.
        double dt = Math.min((now - lastLoopNanos) / 1e9, 0.1);
        lastLoopNanos = now;

        double turretStick = gamepad1.left_stick_x;
        if (Math.abs(turretStick) >= Constants.STICK_DEADZONE) {
            double degrees = turretStick * Constants.TURRET_STICK_DEG_PER_SEC * dt;
            adjustTurret(degrees * Constants.turretPositionPerDeg());
        }

        double hoodStick = -gamepad1.right_stick_y;
        if (Math.abs(hoodStick) >= Constants.STICK_DEADZONE) {
            adjustHood(hoodStick * Constants.HOOD_STICK_POSITION_PER_SEC * dt);
        }

        // Clamp every loop. Panels range change apply now.
        hoodPosition = clampHood(hoodPosition);
    }

    private void adjustPower(double delta) {
        power = Constants.clamp(power + delta, -1.0, 1.0);
    }

    private void adjustTurret(double delta) {
        turretPosition = Constants.clamp(turretPosition + delta, 0, 1);
    }

    /** All hood input go here. Never leave range. */
    private void adjustHood(double delta) {
        hoodPosition = clampHood(hoodPosition + delta);
    }

    private static double clampHood(double position) {
        return Constants.clamp(position, Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION);
    }

    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
        // Schedule once. Own shooter whole OpMode.
        robot.shooter.manualControl(
                () -> power,
                () -> turretPosition,
                () -> hoodPosition
        ).schedule();
    }

    @Override
    public void periodic() {
        // Gamepad 2 drive. SDK Y negative when up.
        robot.driveRobotCentric(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);
        driveFromSticks();

        Telemetry.log("drive: gamepad 2");
        Telemetry.log("flywheel: triggers step power, B stops");
        Telemetry.log("turret:   left stick X, dpad L/R fine, bumpers coarse");
        Telemetry.log("hood:     right stick Y, dpad U/D fine, Y/A coarse");
        Telemetry.log("marks:    X center, BACK reference");
        Telemetry.log("");

        double rpm = robot.shooter.currentRpm();
        Telemetry.log("-- flywheel --");
        Telemetry.log("commanded power", String.format("%.2f", power));
        Telemetry.log("applied throttle", String.format("%.2f", robot.shooter.flywheelThrottle()));
        Telemetry.log("measured RPM", String.format("%.0f", rpm));
        Telemetry.log("current (A)", String.format("%.2f", robot.shooter.flywheelCurrentAmps()));
        // Rough kV, kS taken as 0. Flywheel Tuner more exact.
        Telemetry.log("battery (V)", String.format("%.2f", robot.batteryVoltage()));
        if (Math.abs(rpm) > 1) {
            Telemetry.log("implied kV (V/RPM)",
                    String.format("%.6f", power * robot.batteryVoltage() / rpm));
        } else {
            Telemetry.log("implied kV (V/RPM)", "waiting for RPM");
        }
        Telemetry.log("rpm", rpm);
        Telemetry.log("ticks per rev in use", Constants.FLYWHEEL_TICKS_PER_REV);
        Telemetry.log("");

        Telemetry.log("-- turret --");
        Telemetry.log("turret position", String.format("%.4f", turretPosition));
        Telemetry.log("center mark", centerMark == null ? "unset" : format(centerMark));
        Telemetry.log("reference mark", referenceMark == null ? "unset" : format(referenceMark));
        if (centerMark != null && referenceMark != null) {
            double delta = referenceMark - centerMark;
            Telemetry.log("position delta", String.format("%.4f", delta));
            Telemetry.log("predicted angle between marks (deg)",
                    String.format("%.1f", delta / Constants.turretPositionPerDeg()));
            Telemetry.log("compare with protractor; if off, tune TURRET_SERVO_TRAVEL_DEG");
        }
        Telemetry.log("angle per current constants (deg)", String.format("%.1f", Shooter.degreesForTurretPosition(turretPosition)));
        Telemetry.log("center position in use", String.format("%.4f", Constants.TURRET_CENTER_POSITION));
        Telemetry.log("position per deg in use", String.format("%.6f", Constants.turretPositionPerDeg()));
        Telemetry.log("");

        Telemetry.log("-- hood --");
        Telemetry.log("hood position", String.format("%.4f", hoodPosition));
        Telemetry.log("range in use", String.format("%.2f .. %.2f", Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION));
    }

    @Override
    public void end() {
        // Cut power direct. Command already cancelled here.
        power = 0;
        robot.shooter.coastFlywheel();
    }

    private static String format(double value) {
        return String.format("%.4f", value);
    }
}
