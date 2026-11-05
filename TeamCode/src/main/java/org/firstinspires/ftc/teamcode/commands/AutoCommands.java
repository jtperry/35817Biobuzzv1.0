package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.pedro.LimitedMecanum;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Auto drive steps. Shooter mechanisms are never used here: they are mode-based and track on their own,
 * and a group that required them would hold them for the whole routine.
 */
@Configurable
public final class AutoCommands {

    /** Slower than this on an intake path = maybe stuck on a ball. */
    public static double STUCK_SPEED_IN_PER_S = 2.0;

    /** Stuck this long = give up the path, hold where it is, go to the next step. */
    public static double STUCK_MS = 3000;

    /** An in-place turn is done once the heading is this close to its target (degrees). */
    public static double TURN_DONE_DEG = 3;

    /** Longest in-place turn (ms) before the auto moves on anyway. */
    public static double TURN_CAP_MS = 1500;

    private AutoCommands() {
    }

    /**
     * Turn in place to target's heading while holding its position. Done once the heading is within TURN_DONE_DEG, or
     * after TURN_CAP_MS. For a turn the next path would otherwise make while driving toward a wall.
     */
    public static Command turnTo(Robot robot, Pose target) {
        // A plain instant hold, so the cap bounds the whole turn whatever Ivy's hold command waits for.
        return Commands.instant(() -> robot.follower().hold(target))
                .then(Commands.waitUntil(() -> headingWithin(robot.pose().heading(), target.heading(), TURN_DONE_DEG))
                        .raceWith(Commands.waitMs(TURN_CAP_MS)));
    }

    /** True if the two headings (radians) are within toleranceDeg of each other, the short way round. NaN = false. */
    static boolean headingWithin(double headingRad, double targetRad, double toleranceDeg) {
        return Math.abs(MathUtil.wrapRadians(headingRad - targetRad)) <= Math.toRadians(toleranceDeg);
    }

    /** Aim at this cell from now on (after a tip). */
    public static Command switchCell(Robot robot, HiveCell cell) {
        return Commands.instant(() -> robot.setCell(cell));
    }

    /** Drive path at percent of full power (0-100). Full power again after. Hold end pose. */
    public static Command drive(Robot robot, Path path, double powerPercent) {
        return limitPower(powerPercent)
                .then(drive(robot, path))
                .then(limitPower(100));
    }

    /** Drive + intake at percent of full power (0-100). Full power again after. Hold end pose. */
    public static Command driveIntaking(Robot robot, Path path, double powerPercent) {
        return limitPower(powerPercent)
                .then(driveIntaking(robot, path))
                .then(limitPower(100));
    }

    /**
     * Straight path, fast then slow, split at splitAt (0-1). Pedro brakes only at a path end, so the fast part
     * brakes into the split point, then the slow part runs. Intake the whole time; stuck = move on.
     */
    public static Command driveIntakingFastThenSlow(Robot robot, Path line, double splitAt,
                                                    double fastPercent, double slowPercent) {
        Pose start = line.get(0);
        Pose split = line.get(MathUtil.clamp(splitAt, 0, 1));
        Pose end = line.get(1);
        Path fast = Paths.line(start, split).linear(start, split);
        Path slow = Paths.line(split, end).linear(split, end);
        return limitPower(fastPercent)
                .then(deadline(follow(robot.follower(), fast), robot.intake.intake()))
                .then(limitPower(slowPercent))
                .then(driveIntakingUnjam(robot, slow))
                .then(limitPower(100));
    }

    /**
     * Piecewise path speed, any path. Pairs: path completion (0-1) where a piece starts, percent power.
     * Example: 0, 100, 0.5, 30 = first half full power, rest 30%. Full power again after.
     */
    public static Command withSpeeds(Robot robot, Command drive, double... donePercentPairs) {
        return deadline(drive, Commands.infinite(() -> {
            double done = robot.follower().completion();
            double percent = 100;
            for (int i = 0; i + 1 < donePercentPairs.length; i += 2) {
                if (done >= donePercentPairs[i]) {
                    percent = donePercentPairs[i + 1];
                }
            }
            LimitedMecanum.maxPower = MathUtil.clamp(percent / 100.0, 0, 1);
        })).then(limitPower(100));
    }

    /** Drive + intake at percent power. Stuck on a ball for STUCK_MS = stop the path, hold, go on. */
    public static Command driveIntakingUnjam(Robot robot, Path path, double powerPercent) {
        return limitPower(powerPercent)
                .then(driveIntakingUnjam(robot, path))
                .then(limitPower(100));
    }

    /** Drive + intake. Stuck on a ball for STUCK_MS = stop the path, hold where it is, go on. */
    public static Command driveIntakingUnjam(Robot robot, Path path) {
        StuckTimer stuck = new StuckTimer(robot);
        return deadline(
                Commands.instant(stuck::reset).then(
                        follow(robot.follower(), path).raceWith(Commands.waitUntil(stuck::isStuck))),
                robot.intake.intake())
                .then(robot.intake.stopIntake())
                // Stuck: hold where it is, do not push into the jam. Else hold the path end.
                .then(Commands.instant(() -> robot.follower().hold(
                        stuck.gaveUp() ? robot.pose() : path.endPose())));
    }

    /** Run the intake this long, then stop the rollers. */
    public static Command intakeFor(Robot robot, double ms) {
        return robot.intake.intake().raceWith(Commands.waitMs(ms))
                .then(robot.intake.stopIntake());
    }

    /** Drive path, then hold its end pose. */
    public static Command drive(Robot robot, Path path) {
        return follow(robot.follower(), path)
                // endPose, not hold(follower): that one grabs the start pose.
                .then(hold(robot.follower(), path.endPose()));
    }

    /** Drive + intake, stop the rollers, hold the end pose. */
    public static Command driveIntaking(Robot robot, Path path) {
        return deadline(
                follow(robot.follower(), path),
                robot.intake.intake())
                .then(robot.intake.stopIntake())
                .then(hold(robot.follower(), path.endPose()));
    }

    private static Command limitPower(double percent) {
        return Commands.instant(() -> LimitedMecanum.maxPower = MathUtil.clamp(percent / 100.0, 0, 1));
    }

    /** True when the robot barely moves for STUCK_MS. */
    private static final class StuckTimer {

        private final Robot robot;
        private long lastMoveMs;
        private boolean gaveUp;

        StuckTimer(Robot robot) {
            this.robot = robot;
        }

        void reset() {
            lastMoveMs = System.currentTimeMillis();
            gaveUp = false;
        }

        boolean isStuck() {
            Velocity v = robot.velocity();
            long now = System.currentTimeMillis();
            if (Math.hypot(v.vx, v.vy) > STUCK_SPEED_IN_PER_S) {
                lastMoveMs = now;
            }
            gaveUp = now - lastMoveMs >= STUCK_MS;
            return gaveUp;
        }

        boolean gaveUp() {
            return gaveUp;
        }
    }
}
