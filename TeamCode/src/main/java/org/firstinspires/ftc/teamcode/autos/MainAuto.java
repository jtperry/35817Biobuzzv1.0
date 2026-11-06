package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldDrawing;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;

/** Setup every auto shares. Subclass = path sequence. */
public abstract class MainAuto extends NextOpMode {

    protected final Robot robot;
    protected final Alliance alliance;
    private final RobotTelemetry tele;

    protected MainAuto(Robot robot, Alliance alliance, Pose startPose) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        this.alliance = alliance;
        tele = new RobotTelemetry(telemetry);

        robot.bindHardware(hardwareMap);
        robot.beginAuto(hardwareMap, alliance, startPose);
        // Not reset(): an error here must wait for robot.initCheck() to reach the Driver Station.
        robot.resetInInit();
        // Flywheel tracks table speed all auto (spins from start); turret and hood track on their own.
        robot.flywheel.arm();
    }

    /** Auto routine. */
    protected abstract Command routine();

    /** Routine step named id: the sim sees where body starts and ends (StepMarks). The robot just runs body. */
    protected Command step(String id, Command body) {
        return StepMarks.wrap(id, body);
    }

    @Override
    public void start() {
        routine().schedule();
    }

    @Override
    public void disabledPeriodic() {
        // Cell on the robot's side, like teleop. Turret faces it before start.
        robot.pickCellBySide();
        robot.aimInInit();
        pushTelemetry();
    }

    @Override
    public void periodic() {
        robot.pickCellBySide();
        pushTelemetry();
    }

    private void pushTelemetry() {
        if (!tele.beginFrame()) {
            return;
        }
        Pose pose = robot.pose();
        tele.add("alliance / cell", alliance + " / " + robot.cell());
        tele.add("pose", String.format("%.1f, %.1f, %.1f deg",
                pose.x(), pose.y(), MathUtil.wrapDegrees(Math.toDegrees(pose.heading()))));
        tele.detail("ready to shoot", robot.readyToShoot());
        tele.detail("flywheel", String.format("%.0f / %.0f RPM", robot.flywheel.rpm(), robot.flywheel.targetRpm()));
        FieldDrawing.robot(pose);
        tele.endFrame();
    }

    @Override
    public void end() {
        robot.flywheel.reset();
    }
}
