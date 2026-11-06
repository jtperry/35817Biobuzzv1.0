package org.biobuzz.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autos.AutoRoute;
import org.firstinspires.ftc.teamcode.sim.BlueAuto2TipSim;
import org.firstinspires.ftc.teamcode.sim.RedAuto2TipSim;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.Test;

import java.util.List;

/**
 * The real 2 Tip auto (BlueAuto2Tip / RedAuto2Tip: MainAuto2Tip, MainAuto, Robot, Pedro, Ivy, NextFTC) runs in SimLoop
 * from INIT to STOP without an exception (an OpModeException fails the test), drives the plant well off its start,
 * and a second identical run ends at the same pose.
 *
 * <p>Only the end pose repeats exactly, not the timeline: Ivy 1.1.1's Race, Parallel and Deadline keep their children
 * in a HashMap keyed by identity hash, so the order they run in within a loop changes from run to run (on the robot
 * too), and each race, parallel or deadline can end one tick earlier or later. Every one of them ends while the robot
 * holds still, so the path, and the pose it ends at, are the same.
 */
public class AutoSimSmokeTest {

    private static final double MOVED_IN = 20;
    private static final double SAME_POSE = 1e-9;

    /** What one run did. */
    private static final class Run {
        double x;
        double y;
        double heading;
        double farthestIn;
    }

    @Test
    public void blue2TipRunsTheRouteAndEndsAtTheSamePoseTwice() {
        check("Blue 2 Tip", BlueAuto2TipSim.class, new AutoRoute.Blue());
    }

    @Test
    public void red2TipRunsTheRouteAndEndsAtTheSamePoseTwice() {
        check("Red 2 Tip", RedAuto2TipSim.class, new AutoRoute.Red());
    }

    private static void check(String name, Class<? extends OpMode> adapter, Object export) {
        Run first = run(name + ", run 1", adapter, export);
        assertTrue(name + " got only " + first.farthestIn + " in from its start", first.farthestIn > MOVED_IN);
        Run second = run(name + ", run 2", adapter, export);
        assertEquals(name + ": x differs between two identical runs", first.x, second.x, SAME_POSE);
        assertEquals(name + ": y differs between two identical runs", first.y, second.y, SAME_POSE);
        assertEquals(name + ": heading differs between two identical runs", first.heading, second.heading, SAME_POSE);
    }

    /** A fresh world and runner every time: a runner runs once, and the plant must start from the placement. */
    private static Run run(String name, Class<? extends OpMode> adapter, Object export) {
        final BiobuzzSim sim = new BiobuzzSim();
        final Pose start = BiobuzzSim.exportStart(export);
        sim.placeAtStartOf(export);
        final Run run = new Run();
        long startNanos = System.nanoTime();
        OpModeRunResult result = sim.runner(adapter, dt -> run.farthestIn = Math.max(run.farthestIn,
                Math.hypot(sim.pedroX() - start.x(), sim.pedroY() - start.y()))).run();
        double wallSeconds = (System.nanoTime() - startNanos) / 1e9;
        run.x = sim.pedroX();
        run.y = sim.pedroY();
        run.heading = sim.heading();

        List<List<String>> frames = result.telemetry().transmissions();
        int ready = 0;
        for (List<String> frame : frames) {
            if (frame.toString().contains("ready to shoot: true")) {
                ready++;
            }
        }
        System.out.printf("%s: %.1f s wall clock for %d ticks (%.2f s sim); start (%.2f, %.2f, %.1f deg), end"
                        + " (%.2f, %.2f, %.1f deg), farthest %.1f in from the start; flywheel %.0f RPM; %d of %d"
                        + " telemetry frames ready to shoot%n",
                name, wallSeconds, result.ticks(), result.finalTimeSeconds(), start.x(), start.y(),
                Math.toDegrees(start.heading()), run.x, run.y, Math.toDegrees(run.heading), run.farthestIn,
                sim.flywheelRpm(), ready, frames.size());
        System.out.println(name + ": last telemetry " + result.telemetry().lastTransmission());
        System.out.println(name + ": warnings " + result.warnings());
        return run;
    }
}
