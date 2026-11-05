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
                ShootCommands.volley(robot),

                // First half full power, rest 30%. Stuck on a ball 3 s = move on.
                AutoCommands.withSpeeds(robot, AutoCommands.driveIntakingUnjam(robot, route.intakeflower()),
                        0, 100,
                        0.5, 30),
                AutoCommands.intakeFor(robot, 500),
                AutoCommands.drive(robot, route.gardenhiveBshoot()),
                // Turn to flower 1's heading here, during the volley (the turret keeps aiming), so the short drive
                // in toward the wall is straight instead of finishing the turn on the way.
                parallel(ShootCommands.volley(robot),
                        AutoCommands.turnTo(robot, route.hiveBshootflower1().get(0))),

                // Flower 1 is close: wait until the arm is down before driving in.
                Commands.instant(robot.flower::deploy),
                Commands.waitUntil(robot.flower::atTarget).raceWith(Commands.waitMs(FLOWER_WAIT_CAP_MS)),
                AutoCommands.driveIntakingUnjam(robot, route.hiveBshootflower1(), 75),
                AutoCommands.intakeFor(robot, 1000),

                AutoCommands.drive(robot, route.flower1hiveBshootcheck()),
                ShootCommands.volley(robot),

                AutoCommands.drive(robot, route.hiveBshootcheckflower2prep()),
                AutoCommands.driveIntakingUnjam(robot, route.flower2prepflower2(), 75),
                AutoCommands.intakeFor(robot, 1000),
                Commands.instant(robot.flower::stow),

                // The export calls this spot hiveAshoot, but it is on B's side: the third volley at B.
                AutoCommands.drive(robot, route.flower2hiveAshoot()),
                ShootCommands.volley(robot),

                AutoCommands.drive(robot, route.hiveAshootparkprep()),
                AutoCommands.drive(robot, route.parkpreppark(), 50));
    }
}
