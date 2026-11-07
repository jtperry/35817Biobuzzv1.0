package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.field.Alliance;

/** 2 Tip auto: preloads at A, then three volleys at B. Routine for both alliances. */
public abstract class MainAuto2Tip extends MainAuto {

    /** Longest wait for the flower arm to finish deploying (about 1.4 s at the default speed), so it never hangs. */
    private static final double FLOWER_WAIT_CAP_MS = 2500;

    private final AutoRoute route;

    protected MainAuto2Tip(Robot robot, Alliance alliance, AutoRoute route) {
        super(robot, alliance, StartPose.of(route));
        this.route = route;
    }

    @Override
    protected Command routine() {
        return sequential(
                step("preload-volley", ShootCommands.volley(robot)),

                step("intakeflower", AutoCommands.withSpeeds(robot,
                        AutoCommands.driveIntakingUnjam(robot, route.intakeflower()),
                        0, 100,
                        0.5, 30)),
                step("garden-intake", AutoCommands.intakeFor(robot, 500)),
                step("gardenhiveBshoot", AutoCommands.drive(robot, route.gardenhiveBshoot())),
                step("volley-B1+turn", parallel(ShootCommands.volley(robot),
                        AutoCommands.turnTo(robot, route.hiveBshootflower1().get(0)))),

                step("flower-deploy", sequential(
                        Commands.instant(robot.flower::deploy),
                        Commands.waitUntil(robot.flower::atTarget).raceWith(Commands.waitMs(FLOWER_WAIT_CAP_MS)))),
                step("hiveBshootflower1", AutoCommands.driveIntakingUnjam(robot, route.hiveBshootflower1(), 75)),
                step("flower1-intake", AutoCommands.intakeFor(robot, 1000)),

                step("flower1hiveBshootcheck", AutoCommands.drive(robot, route.flower1hiveBshootcheck())),
                step("volley-B2", ShootCommands.volley(robot)),

                step("hiveBshootcheckflower2prep", AutoCommands.drive(robot, route.hiveBshootcheckflower2prep())),
                step("flower2prepflower2", AutoCommands.driveIntakingUnjam(robot, route.flower2prepflower2(), 75)),
                step("flower2-intake", AutoCommands.intakeFor(robot, 1000)),
                step("flower-stow", Commands.instant(robot.flower::stow)),

                step("flower2hiveAshoot", AutoCommands.drive(robot, route.flower2hiveAshoot())),
                step("volley-B3", ShootCommands.volley(robot)),

                step("hiveAshootparkprep", AutoCommands.drive(robot, route.hiveAshootparkprep())),
                step("parkpreppark", AutoCommands.drive(robot, route.parkpreppark(), 50)),
                StepMarks.routineEnd());
    }
}
