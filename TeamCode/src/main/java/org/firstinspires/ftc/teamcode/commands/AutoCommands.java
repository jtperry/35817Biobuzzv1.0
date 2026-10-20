package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
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
