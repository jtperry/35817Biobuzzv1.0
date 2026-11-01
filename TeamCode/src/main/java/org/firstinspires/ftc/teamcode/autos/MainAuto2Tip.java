package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

/** Main auto. Routine for both alliances. */
public abstract class MainAuto2Tip extends MainAuto {

    private final AutoRoute route;

    protected MainAuto2Tip(Robot robot, Alliance alliance, AutoRoute route) {
        super(robot, alliance, StartPose.of(route));
        this.route = route;
    }

    @Override
    protected Command routine() {
        return sequential(
                ShootCommands.shootAll(robot),

                // First half full power, rest 30%. Stuck on ball 3 s = move on.
                AutoCommands.withSpeeds(robot, AutoCommands.driveIntakingUnjam(robot, route.garden()),
                        0, 100,
                        0.5, 30),
                AutoCommands.intakeFor(robot, 500),
                AutoCommands.drive(robot, route.gardenShoot()),
                ShootCommands.shootAll(robot),

                robot.intake.deployFlower(),
                Commands.waitMs(1000),
                AutoCommands.driveIntakingUnjam(robot, route.flower1(), 75),
                AutoCommands.intakeFor(robot, 1000),

                AutoCommands.drive(robot, route.flower1Shoot()),
                ShootCommands.shootAll(robot),

                AutoCommands.drive(robot, route.flower2Prep()),
                AutoCommands.driveIntakingUnjam(robot, route.flower2(), 75),
                AutoCommands.intakeFor(robot, 1000),
                robot.intake.stowFlower(),

                AutoCommands.drive(robot, route.pushPrep()),
                AutoCommands.drive(robot, route.flower2Shoot()),
                ShootCommands.shootAll(robot),

                AutoCommands.drive(robot, route.parkPrep()),
                AutoCommands.drive(robot, route.park(), 50));
    }
}
