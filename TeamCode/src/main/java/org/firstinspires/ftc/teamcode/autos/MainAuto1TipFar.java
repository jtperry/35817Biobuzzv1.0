package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

/** Far 1 tip auto: preload, one long curve to the far spot, shoot there. Both alliances. */
public abstract class MainAuto1TipFar extends MainAuto {

    private final AutoRoute1TipFar route;

    protected MainAuto1TipFar(Robot robot, Alliance alliance, AutoRoute1TipFar route) {
        super(robot, alliance, StartPose.of(route));
        this.route = route;
    }

    @Override
    protected Command routine() {
        return sequential(
                // Preload. Turret aimed + flywheel at speed from init.
                ShootCommands.volley(robot),

                AutoCommands.drive(robot, route.path1()),
                // The tip: one volley from the far spot.
                ShootCommands.volley(robot));
    }
}
