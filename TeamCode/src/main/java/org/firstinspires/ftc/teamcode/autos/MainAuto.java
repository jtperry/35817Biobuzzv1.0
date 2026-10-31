package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;

/** Main auto. Routine for both alliances. */
public abstract class MainAuto extends NextOpMode {

    private final Robot robot;
    private final Alliance alliance;
    private final AutoRoute route;

    /** False = no follower. Nothing scheduled. */
    private final boolean ready;

    protected MainAuto(Robot robot, Alliance alliance, AutoRoute route) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        this.alliance = alliance;
        this.route = route;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());

        robot.bindHardware();
        // Start pose = export "start" pose.
        ready = robot.beginAuto(hardwareMap, alliance, StartPose.of(route));
        robot.reset();
        // Like teleop: flywheel at table speed whole auto. Turret aim by default command.
        robot.shooter.arm();
    }

    /** Auto routine. Add actions between drives. */
    private Command routine() {
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

    @Override
    public void start() {
        if (ready) {
            routine().schedule();
        }
    }

    @Override
    public void disabledPeriodic() {
        // Goal on robot's side, like teleop. Turret face it before start.
        robot.pickGoalBySide();
        robot.shooter.aimTurretAtGoal();
        pushTelemetry();
    }

    @Override
    public void periodic() {
        robot.pickGoalBySide();
        pushTelemetry();
    }

    private void pushTelemetry() {
        Telemetry.log("alliance", alliance);
        Telemetry.log("goal", robot.goal());
        if (!ready) {
            Telemetry.log("NOT READY", "follower unavailable — implement pedro/Constants.create()");
            return;
        }
        Pose pose = robot.pose();
        Telemetry.log("x", String.format("%.1f", pose.x()));
        Telemetry.log("y", String.format("%.1f", pose.y()));
        Telemetry.log("heading (deg)", String.format("%.1f", Math.toDegrees(pose.heading())));
    }

    @Override
    public void end() {
        // Cut flywheel direct. Scheduler shutting down.
        robot.shooter.coastFlywheel();
    }
}
