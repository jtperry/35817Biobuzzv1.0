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
        robot.resetInInit();
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
        org.firstinspires.ftc.teamcode.pedro.SharedOctoQuadLocalizer localizer = robot.localizer();
        if (localizer != null) {
            tele.add("odo status", String.format(java.util.Locale.US, "%s | crc:%s | bad:%d",
                    localizer.lastStatus(),
                    localizer.localizerData().isDataValid() ? "OK" : "ERR",
                    localizer.badLocalizerReads()));
        }
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
