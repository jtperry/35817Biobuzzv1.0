package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.input.DriverInput;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import java.util.Locale;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Robot-centric driving with the pose and velocity on screen (spec 7.2). This one DRIVES the robot. Gamepad 1:
 *   left stick = forward / strafe, right stick X = turn (all times DRIVE_SCALE)
 *   Y = reset the pose to the alliance start pose
 * Checks (spec 5.5, 4.6):
 *   velocity frame: drive toward the far wall (+y): the field vy must be positive at any heading
 *   pod spin: press Y, spin in place 5 times: x and y stay within about 0.25 in
 */
@Configurable
@NextTeleop(name = "Drive Test", group = "test")
public class DriveTest extends NextOpMode {

    /** Stick output is multiplied by this (0 to 1), so a bring-up test starts slow. */
    public static double DRIVE_SCALE = 0.6;

    private final Robot robot;
    private final RobotTelemetry tele;
    private final CommandGamepad pad;

    public DriveTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);
        robot.bindHardware(hardwareMap);
        robot.initFollower(hardwareMap, Field.startPose(robot.alliance()));

        pad = new CommandGamepad(gamepad1);
        // Slot 0 is the start pose. The next loop's drive call takes the follower out of the hold this leaves it in.
        pad.y().onTrue(Commands.instant(() -> robot.resetPoseToSlot(0)));
    }

    @Override
    public void disabledPeriodic() {
        if (!tele.beginFrame()) {
            return;
        }
        tele.line("Drive Test DRIVES the robot after PLAY. Clear the area.");
        tele.add("alliance", robot.alliance());
        tele.endFrame();
    }

    @Override
    public void start() {
        // No motion here: the drive command is first sent from periodic(), from the sticks.
        robot.reset();
    }

    @Override
    public void periodic() {
        drive();
        pushTelemetry();
    }

    private void drive() {
        double scale = MathUtil.clamp(DRIVE_SCALE, 0, 1);
        double stick = DriverInput.stickScale(gamepad1.left_stick_x, gamepad1.left_stick_y) * scale;
        // Stick y is negative when pushed up. Strafe is positive to the left, turn is counterclockwise positive.
        robot.driveRobotCentric(
                MathUtil.safePower(-gamepad1.left_stick_y * stick),
                MathUtil.safePower(-gamepad1.left_stick_x * stick),
                MathUtil.safePower(-DriverInput.turn(gamepad1.right_stick_x) * scale));
    }

    private void pushTelemetry() {
        if (!tele.beginFrame()) {
            return;
        }
        Pose pose = robot.pose();
        Velocity velocity = robot.velocity();
        double heading = pose.heading();
        // Field velocity turned into the robot frame: rotate by -heading.
        double forward = velocity.vx * Math.cos(heading) + velocity.vy * Math.sin(heading);
        double left = -velocity.vx * Math.sin(heading) + velocity.vy * Math.cos(heading);
        tele.add("pose", String.format(Locale.US, "x %.2f  y %.2f  heading %.1f deg",
                pose.x(), pose.y(), MathUtil.wrapDegrees(Math.toDegrees(heading))));
        tele.add("velocity, field (in/s)", String.format(Locale.US, "vx %.1f  vy %.1f", velocity.vx, velocity.vy));
        tele.add("velocity, robot (in/s)", String.format(Locale.US, "forward %.1f  left %.1f", forward, left));
        tele.add("drive scale", String.format(Locale.US, "%.2f", MathUtil.clamp(DRIVE_SCALE, 0, 1)));
        tele.line("Velocity check: drive toward the far wall (+y): vy must be > 0 at any heading.");
        tele.line("Pod spin test: press Y, spin in place 5 times: x and y stay within about 0.25 in.");
        tele.endFrame();
    }
}
