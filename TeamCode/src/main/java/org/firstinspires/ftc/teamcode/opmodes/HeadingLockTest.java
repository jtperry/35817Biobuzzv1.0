package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Goal;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Main TeleOp but with heading lock to test heading lock. */
@NextTeleop(name = "Heading Lock test")
public class HeadingLockTest extends NextOpMode {

    private final Robot robot;
    private final CommandGamepad driver;
    private final CommandGamepad operator;

    /** Got alliance + pose from auto. */
    private final boolean fromAuto;
    /** Heading lock */
    private static final double HL_TOL_RAD = Math.toRadians(1.0);     // inside this: stop turning
    private static final double HL_RESUME_RAD = Math.toRadians(3.0);  // beyond this: turn again
    private boolean hlSettled = false;
    private static final int LOCK_OFF = 0, LOCK_GOAL = 1, LOCK_PASS = 2;
    private static final double HL_STICK_CANCEL = 0.15;   // turn stick past this = driver takes over
    private int lockMode = LOCK_OFF;
    private boolean yWasDown = false, bWasDown = false;
    private static final double HL_KP = 1.6, HL_KD = 0.12, HL_MAX_TURN = 0.8;
    private static final double HL_MIN_DIST_IN = 4.0;      // closer than this, bearing is noise
    private static final double PASS_X_BLUE = 141.5, PASS_X_RED = 0;  // B target, change to your corner
    private static final double FIELD_TOP_Y = 141.5, FIELD_BOTTOM_Y = 0;
    private boolean startTop = true;
    private double hlLastError = 0;
    private long hlLastNanos = 0;
    private boolean hlWasActive = false;

    public HeadingLockTest(Robot robot) {
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
        if (gamepad1.dpad_up) {
            startTop = true;
        } else if (gamepad1.dpad_down){
            startTop = false;
        }
        Telemetry.log("start corner", (startTop ? "TOP" : "BOTTOM")
                + " (dpad up = TOP, down = BOTTOM");
        Telemetry.log("B faces", String.format("(%.0f, %.0f)", passX(), passY()));
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

        // Toggle on the press only (not while held): Y = goal, B = pass corner.
        // Pressing the same button again turns the lock off.
        if (gamepad1.y && !yWasDown) {
            lockMode = (lockMode == LOCK_GOAL) ? LOCK_OFF : LOCK_GOAL;
        }
        if (gamepad1.b && !bWasDown) {
            lockMode = (lockMode == LOCK_PASS) ? LOCK_OFF : LOCK_PASS;
        }
        yWasDown = gamepad1.y;
        bWasDown = gamepad1.b;

        // Driver moves the turn stick = take manual control back.
        if (Math.abs(gamepad1.right_stick_x) > HL_STICK_CANCEL) {
            lockMode = LOCK_OFF;
        }

        double turn = -gamepad1.right_stick_x;
        boolean locking = lockMode != LOCK_OFF;

        if (lockMode == LOCK_GOAL) {
            turn = headingLockTurn(Math.toRadians(robot.shooter.bearingToGoal()));
        } else if (lockMode == LOCK_PASS) {
            turn = headingLockTurn(bearingToPointRad(passX(), passY()));
        } else {
        hlWasActive = false;
        hlSettled = false;
    }

        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                turn,
                robot.follower().pose().heading()
        );

        if (locking) {
            robot.follower().manual(powers);
        } else {
            ManualDrive.driveOrHold(robot.follower(), powers);
        }
    }

    /** Angle from robot nose to a field point, radians, CCW positive, in -pi..pi. */
    private double bearingToPointRad(double tx, double ty) {
        Pose pose = robot.pose();
        double dx = tx - pose.x(), dy = ty - pose.y();
        if (Math.hypot(dx, dy) < HL_MIN_DIST_IN) {
            return 0;
        }
        double e = Math.atan2(dy, dx) - pose.heading();
        return Math.atan2(Math.sin(e), Math.cos(e));
    }
    /** Pass target x: alliance side. */
    private double passX() {
        return robot.alliance() == Alliance.RED ? PASS_X_RED : PASS_X_BLUE;
    }

    /** Pass target y: the end of our side we did NOT start on. */
    private double passY() {
        return startTop ? FIELD_BOTTOM_Y : FIELD_TOP_Y;
    }
    /** PD on heading error (radians) -> turn power. */
    private double headingLockTurn(double error) {
        long now = System.nanoTime();
        double derivative = 0;
        if (hlWasActive) {
            double dt = (now - hlLastNanos) / 1e9;
            if (dt > 0) derivative = (error - hlLastError) / dt;
        }
        hlLastError = error;
        hlLastNanos = now;
        hlWasActive = true;

        // Deadband with hysteresis: close enough = no turn, lock stays on.
        double abs = Math.abs(error);
        if (hlSettled) {
            if (abs > HL_RESUME_RAD) hlSettled = false;
        } else if (abs < HL_TOL_RAD) {
            hlSettled = true;
        }
        if (hlSettled) {
            return 0;
        }

        double out = HL_KP * error + HL_KD * derivative;
        return Math.max(-HL_MAX_TURN, Math.min(HL_MAX_TURN, out));
    }
    private void pushTelemetry() {
        Telemetry.log("alliance", robot.alliance());
        Telemetry.log("B faces", String.format("(%.0f, %.0f)", passX(), passY()));
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
        Telemetry.log("gate open", robot.intake.isGateOpen());
        Telemetry.log("flower deployed", robot.intake.isFlowerDeployed());
    }
}
