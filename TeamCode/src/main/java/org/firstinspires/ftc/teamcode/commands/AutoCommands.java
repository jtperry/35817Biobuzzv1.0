package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.globals.Goal;
import org.firstinspires.ftc.teamcode.globals.Robot;

/** Auto drive steps. */
public final class AutoCommands {

    private AutoCommands() {
    }

    /** Aim at this goal from now on. Use after HIVE tip. */
    public static Command switchGoal(Robot robot, Goal goal) {
        return Commands.instant(() -> robot.setGoal(goal));
    }

    /** Path capped at percent of top speed (0-100). Normal speed again after this path. */
    public static Path slow(Path path, double speedPercent) {
        ForesightConfig config = org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig;
        double topSpeed = config.maxAchievableForwardVelocity.get();
        return path.with(config.maxPathSpeed.at(topSpeed * speedPercent / 100.0));
    }

    /** Drive path at percent of top speed (0-100). Hold end pose. */
    public static Command drive(Robot robot, Path path, double speedPercent) {
        return drive(robot, slow(path, speedPercent));
    }

    /** Drive + intake at percent of top speed (0-100). Hold end pose. */
    public static Command driveIntaking(Robot robot, Path path, double speedPercent) {
        return driveIntaking(robot, slow(path, speedPercent));
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
