package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

/** 1 tip auto: one shoot spot, then park. Routine for both alliances. */
public abstract class MainAuto1Tip extends MainAuto {

    private final AutoRoute1Tip route;

    protected MainAuto1Tip(Robot robot, Alliance alliance, AutoRoute1Tip route) {
        super(robot, alliance, StartPose.of(route));
        this.route = route;
    }

    @Override
    protected Command routine() {
        return sequential(
                // Preload. Turret aimed + flywheel at speed from init.
                ShootCommands.shootAll(robot),

                AutoCommands.drive(robot, route.path1()),

                // Half power into park, like 2 Tip.
                AutoCommands.drive(robot, route.path2(), 85));
    }
}
