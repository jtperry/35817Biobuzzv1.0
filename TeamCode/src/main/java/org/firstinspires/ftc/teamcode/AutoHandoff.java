package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Robot's auto-to-teleop pose handoff (spec 8). Only an auto records the pose. Teleop reads it at INIT without using
 * it up and clears it on its first loop, so a teleop stopped during INIT and re-INITed still starts from it, and a
 * test OpMode run in between neither uses it up nor overwrites it.
 */
final class AutoHandoff {

    private boolean recording = false;
    private boolean pending = false;
    private Pose pose;

    /** Auto INIT: pending from the start pose; record() then follows the robot until another OpMode inits. */
    void beginAuto(Pose startPose) {
        pose = startPose;
        pending = true;
        recording = true;
    }

    /** Every OpMode's INIT (Robot.initFollower): stop recording. A pending pose stays for teleop. */
    void stopRecording() {
        recording = false;
    }

    /** Every loop: keeps the robot's pose only while an auto runs. */
    void record(Pose current) {
        if (recording) {
            pose = current;
        }
    }

    /** An auto ran and no teleop loop has used its pose yet. */
    boolean pending() {
        return pending;
    }

    /** Where the auto ended while pending, else the fallback. */
    Pose poseOr(Pose fallback) {
        return pending ? pose : fallback;
    }

    /** Teleop is running: the auto pose is used up. */
    void clear() {
        pending = false;
    }
}
