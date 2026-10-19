package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
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
                AutoCommands.driveIntaking(robot, route.path1()),
                AutoCommands.drive(robot, route.path2()),
                AutoCommands.driveIntaking(robot, route.path3()),
                AutoCommands.drive(robot, route.path4()),
                ShootCommands.shootAll(robot),
                AutoCommands.drive(robot, route.path5()),
                AutoCommands.driveIntaking(robot, route.path6()),
                AutoCommands.drive(robot, route.path7()),
                AutoCommands.drive(robot, route.path8()));
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
