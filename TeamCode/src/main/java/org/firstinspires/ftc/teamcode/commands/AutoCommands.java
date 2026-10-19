package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

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
