package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Goal;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.pedro.LimitedMecanum;

/** Auto drive steps. */
public final class AutoCommands {

    private AutoCommands() {
    }

    /** Aim at this goal from now on. Use after HIVE tip. */
    public static Command switchGoal(Robot robot, Goal goal) {
        return Commands.instant(() -> robot.setGoal(goal));
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
     * Straight path, fast then slow. Split in two paths at splitAt (0-1).
     * Pedro brake only at path end, so first part brake into split point, then slow part.
     * Intake whole time. Slow part stuck on ball AUTO_STUCK_MS = move on.
     */
    public static Command driveIntakingFastThenSlow(Robot robot, Path line, double splitAt,
                                                    double fastPercent, double slowPercent) {
        Pose start = line.get(0);
        Pose split = line.get(Constants.clamp(splitAt, 0, 1));
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
     * Piecewise path speed. Any path. Too fast for piece = brake down to it. Pairs: path done (0-1) where piece start, percent power.
     * Ex: 0, 100, 0.5, 30 = first half full power, rest 30%. Full power again after.
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
            LimitedMecanum.maxPower = Constants.clamp(percent / 100.0, 0, 1);
        })).then(limitPower(100));
    }

    /** Drive + intake at percent power. Stuck on ball AUTO_STUCK_MS = stop path, hold spot, go next. */
    public static Command driveIntakingUnjam(Robot robot, Path path, double powerPercent) {
        return limitPower(powerPercent)
                .then(driveIntakingUnjam(robot, path))
                .then(limitPower(100));
    }

    /** Drive + intake. Stuck on ball AUTO_STUCK_MS = stop path, hold spot, go next. */
    public static Command driveIntakingUnjam(Robot robot, Path path) {
        StuckTimer stuck = new StuckTimer(robot);
        return deadline(
                Commands.instant(stuck::reset).then(
                        follow(robot.follower(), path).raceWith(Commands.waitUntil(stuck::isStuck))),
                robot.intake.intake())
                .then(robot.intake.stopIntake())
                // Stuck: hold where it is, no push into jam. Else hold path end.
                .then(Commands.instant(() -> robot.follower().hold(
                        stuck.gaveUp() ? robot.pose() : path.endPose())));
    }

    /** True when robot barely move for AUTO_STUCK_MS. */
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
            if (Math.hypot(v.vx, v.vy) > Constants.AUTO_STUCK_SPEED_IN_PER_S) {
                lastMoveMs = now;
            }
            gaveUp = now - lastMoveMs >= Constants.AUTO_STUCK_MS;
            return gaveUp;
        }

        boolean gaveUp() {
            return gaveUp;
        }
    }

    /** Run intake this long, then stop rollers. */
    public static Command intakeFor(Robot robot, double ms) {
        return robot.intake.intake().raceWith(Commands.waitMs(ms))
                .then(robot.intake.stopIntake());
    }

    private static Command limitPower(double percent) {
        return Commands.instant(() -> LimitedMecanum.maxPower = Constants.clamp(percent / 100.0, 0, 1));
    }

    /** Drive path. Hold end pose. */
    public static Command drive(Robot robot, Path path) {
        return follow(robot.follower(), path)
                // endPose, not hold(follower): that one grab start pose.
                .then(hold(robot.follower(), path.endPose()));
    }

    // Never use shooter here. Sequence hold its mechanisms whole time,
    // shooter default (aim + flywheel) would stop for all of auto.

    /** Drive + intake. Stop rollers, hold end pose. */
    public static Command driveIntaking(Robot robot, Path path) {
        return deadline(
                follow(robot.follower(), path),
                robot.intake.intake())
                .then(robot.intake.stopIntake())
                // endPose, not hold(follower): that one grab start pose.
                .then(hold(robot.follower(), path.endPose()));
    }
}
