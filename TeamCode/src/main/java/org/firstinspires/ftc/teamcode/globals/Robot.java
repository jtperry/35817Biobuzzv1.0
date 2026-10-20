package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;


import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/** Robot class. Holds whole robot. */
public class Robot implements NextRobot {

    public final IntakeTransfer intake = new IntakeTransfer();
    public final Shooter shooter = new Shooter(this::pose, this::alliance, this::goal, this::velocity);

    private Follower follower;

    private Alliance alliance = Alliance.BLUE;

    /** HIVE goal shooter aim at now. */
    private Goal goal = Goal.A;

    /** Latest pose. */
    private Pose lastPose = Constants.DEFAULT_START_POSE;

    /** Auto handoff waiting for teleop. */
    private boolean autoHandoffPending = false;

    public Robot() {
    }

    // ---------------------------------------------------------------- lifecycle

    /** Build Pedro follower, set start pose. Take OpMode hardwareMap. */
    public void initFollower(HardwareMap hardwareMap, Pose startPose) {
        // Full power. Old limit from stopped auto no carry over.
        org.firstinspires.ftc.teamcode.pedro.LimitedMecanum.maxPower = 1.0;
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(startPose);
        }
    }

    /** Start auto: set alliance, start follower, mark handoff. False = no follower. */
    public boolean beginAuto(HardwareMap hardwareMap, Alliance alliance, Pose startPose) {
        this.alliance = alliance;
        this.goal = Goal.A;
        initFollower(hardwareMap, startPose);
        lastPose = startPose;
        autoHandoffPending = follower != null;
        return follower != null;
    }

    /** Auto ran, handoff not used yet. */
    public boolean hasAutoHandoff() {
        return autoHandoffPending;
    }

    /** Teleop start pose: auto end pose, else default. Use once. */
    public Pose takeHandoffPose() {
        Pose pose = autoHandoffPending ? lastPose : Constants.DEFAULT_START_POSE;
        autoHandoffPending = false;
        return pose;
    }

    /** Pose = default start spot. Hold there, not old hold spot. */
    public void resetPoseToStart() {
        if (follower != null) {
            follower.setPose(Constants.DEFAULT_START_POSE);
            follower.hold(Constants.DEFAULT_START_POSE);
        }
        lastPose = Constants.DEFAULT_START_POSE;
    }

    /** Robot-centric drive. No follower = nothing. */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        if (follower != null) {
            follower.manual(new DrivePowers(forward, strafe, turn));
        }
    }

    /** Fresh servos. Call first every OpMode. */
    public void bindHardware() {
        shooter.bindServos();
        intake.bindServos();
    }

    /** Both mechanisms to known state. */
    public void reset() {
        shooter.reset();
        intake.reset();
    }

    @Override
    public void periodic() {
        if (follower != null) {
            follower.update();
            lastPose = follower.pose();
            Drawing.drawRobot(lastPose);
        }
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return new HashSet<Mechanism>(Arrays.asList(shooter, intake));
    }

    // ---------------------------------------------------------------- accessors

    public Follower follower() {
        return follower;
    }

    public Alliance alliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Goal goal() {
        return goal;
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    /** Switch to other goal. */
    public void toggleGoal() {
        goal = goal.other();
    }

    /** Pick goal on same side as robot. Line = halfway between goals in y. */
    public void pickGoalBySide() {
        double y = pose().y();
        double half = (Constants.GOAL_A_Y + Constants.GOAL_B_Y) / 2;
        boolean aAbove = Constants.GOAL_A_Y > Constants.GOAL_B_Y;
        if (y > half + Constants.GOAL_SWITCH_DEADBAND_IN) {
            goal = aAbove ? Goal.A : Goal.B;
        } else if (y < half - Constants.GOAL_SWITCH_DEADBAND_IN) {
            goal = aAbove ? Goal.B : Goal.A;
        }
    }

    /** Robot pose now. Default if no follower. */
    public Pose pose() {
        return follower == null ? Constants.DEFAULT_START_POSE : follower.pose();
    }

    /** Robot velocity, field frame. Zero if no follower. */
    public Velocity velocity() {
        return follower == null ? Velocity.zero() : follower.velocity();
    }
}
