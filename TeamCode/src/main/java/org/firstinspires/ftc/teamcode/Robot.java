package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.intake.FlowerArm;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.LimitedMecanum;
import org.firstinspires.ftc.teamcode.pedro.SharedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.ShotSolution;
import org.firstinspires.ftc.teamcode.shooter.ShotSolver;
import org.firstinspires.ftc.teamcode.shooter.ShotTable;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.util.Battery;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/**
 * The whole robot. NextFTC creates one instance for the app session; each OpMode calls bindHardware()
 * and initFollower() (or beginAuto()) in its constructor.
 *
 * Loop order (NextFTC): OpMode periodic (gamepads, drive powers) -> Robot.periodic (one OctoQuad read,
 * battery, one ShotSolution) -> mechanism periodic (motor/servo writes) -> command scheduler -> telemetry.
 */
public class Robot implements NextRobot {

    public final Battery battery = new Battery();
    public final Flywheel flywheel = new Flywheel(battery);
    public final Turret turret = new Turret();
    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final FlowerArm flower = new FlowerArm();

    private final ShotSolver solver = new ShotSolver(ShotTable::flightTimeS);
    private final Set<Mechanism> mechanisms = Collections.unmodifiableSet(
            new HashSet<Mechanism>(Arrays.asList(flywheel, turret, hood, intake, flower)));

    private Follower follower;
    private Alliance alliance = Alliance.BLUE;
    private HiveCell cell = HiveCell.A;
    private int restartSlot = 0;
    private final AutoHandoff handoff = new AutoHandoff();
    private long lastSolveNanos = 0;

    // ---------------------------------------------------------------- OpMode setup

    /**
     * Fresh motor and servo objects and the battery sensor. Call first in every OpMode constructor. This Robot lives
     * for the app session, but a Driver Station "Restart Robot" or a config activation builds a new hardware map, and
     * hardware objects kept from an earlier OpMode would go on writing to its closed devices.
     */
    public void bindHardware(HardwareMap hardwareMap) {
        // NextFTC's MotorHook updates every NextMotor in this static loop each cycle: drop the old OpMode's motors
        // (they would pile up and keep writing to stale devices), then make fresh ones.
        NextMotor.Companion.getMotorEventLoop().clear();
        flywheel.bindMotors();
        intake.bindMotors();
        turret.bindServos();
        hood.bindServos();
        intake.bindServos();
        flower.bindServos();
        battery.bind(hardwareMap);
    }

    /** Build the follower at a start pose. The follower is required: a missing OctoQuad throws here. */
    public void initFollower(HardwareMap hardwareMap, Pose startPose) {
        // Only an auto records the pose teleop continues from (beginAuto turns recording back on after this).
        handoff.stopRecording();
        LimitedMecanum.maxPower = 1.0;
        SharedOctoQuadLocalizer localizer = new SharedOctoQuadLocalizer(hardwareMap, Constants.localizerConfig);
        follower = Constants.create(hardwareMap, localizer);
        flywheel.useOctoQuad(localizer);
        follower.setPose(startPose);
        LimitedMecanum.velocity = follower::velocity;
        LimitedMecanum.heading = () -> follower.pose().heading();
        solver.reset();
        lastSolveNanos = 0;
    }

    /** Auto start: alliance, cell A (starts up), follower at the start pose; teleop will continue from here. */
    public void beginAuto(HardwareMap hardwareMap, Alliance autoAlliance, Pose startPose) {
        alliance = autoAlliance;
        cell = HiveCell.A;
        initFollower(hardwareMap, startPose);
        handoff.beginAuto(startPose);
    }

    /** An auto ran and no teleop loop has used its end pose yet. */
    public boolean hasAutoHandoff() {
        return handoff.pending();
    }

    /** Teleop start pose: where auto ended, else slot 0 for the alliance. Read at INIT; does not use it up. */
    public Pose handoffPose() {
        return handoff.poseOr(Field.startPose(alliance));
    }

    /**
     * Teleop's first loop: the auto pose is used up. Not in start(): NextFTC also calls start() after a STOP during
     * INIT, and that teleop never ran.
     */
    public void clearAutoHandoff() {
        handoff.clear();
    }

    /** Every mechanism to a known state. */
    public void reset() {
        flywheel.reset();
        turret.reset();
        hood.reset();
        intake.reset();
        flower.reset();
    }

    // ---------------------------------------------------------------- loop

    @Override
    public void periodic() {
        follower.update();
        handoff.record(follower.pose());
        battery.update();
        solveShot();
    }

    /** Init loop: mechanism periodic does not run before start, so point the turret here. */
    public void aimInInit() {
        solveShot();
        turret.applyTrackingNow();
    }

    private void solveShot() {
        long now = System.nanoTime();
        double dt = lastSolveNanos == 0 ? 0 : Math.min((now - lastSolveNanos) / 1e9, 0.1);
        lastSolveNanos = now;
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        double vx = velocity == null ? 0 : velocity.vx;
        double vy = velocity == null ? 0 : velocity.vy;
        double omega = velocity == null ? 0 : velocity.omega;
        ShotSolution shot = solver.update(pose.x(), pose.y(), pose.heading(), vx, vy, omega, dt,
                Field.cellX(alliance, cell), Field.cellY(alliance, cell));
        turret.setTracking(shot.turretDeg, shot.turretRateDegPerS);
        hood.setTracking(ShotTable.hood(shot.distanceIn));
        flywheel.setTrackingRpm(ShotTable.rpm(shot.distanceIn));
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    /** The shot computed this loop. */
    public ShotSolution shot() {
        return solver.solution();
    }

    public boolean readyToShoot() {
        return flywheel.atSpeed() && aimSettled();
    }

    /** Turret and hood where the shot wants them. */
    public boolean aimSettled() {
        return turret.settled() && hood.settled();
    }

    // ---------------------------------------------------------------- driving

    public Follower follower() {
        return follower;
    }

    public Pose pose() {
        return follower.pose();
    }

    public Velocity velocity() {
        Velocity velocity = follower.velocity();
        return velocity == null ? Velocity.zero() : velocity;
    }

    public void driveRobotCentric(double forward, double strafe, double turn) {
        follower.manual(new DrivePowers(forward, strafe, turn));
    }

    // ---------------------------------------------------------------- alliance, cell, restart slots

    public Alliance alliance() {
        return alliance;
    }

    public void setAlliance(Alliance newAlliance) {
        alliance = newAlliance;
    }

    public HiveCell cell() {
        return cell;
    }

    public void setCell(HiveCell newCell) {
        cell = newCell;
    }

    /** Aim at the cell on the robot's side of the hive (teleop, and auto until spec 2). */
    public void pickCellBySide() {
        cell = Field.cellOnSide(alliance, follower.pose().y(), cell);
    }

    public int restartSlot() {
        return restartSlot;
    }

    public void setRestartSlot(int slot) {
        restartSlot = Math.floorMod(slot, Field.RESTART_SLOTS);
    }

    /** Pose = that restart slot for this alliance, and hold there. */
    public void resetPoseToSlot(int slot) {
        setRestartSlot(slot);
        Pose pose = Field.restartPose(alliance, restartSlot);
        follower.setPose(pose);
        follower.hold(pose);
        solver.reset();
    }

    public void resetPoseToStart() {
        resetPoseToSlot(restartSlot);
    }
}
