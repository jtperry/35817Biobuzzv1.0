package org.biobuzz.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autos.AutoRoute;
import org.firstinspires.ftc.teamcode.autos.StepMarks;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.sim.BlueAuto2TipSim;
import org.firstinspires.ftc.teamcode.sim.NextOpModeSim;
import org.firstinspires.ftc.teamcode.sim.RedAuto2TipSim;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.BiConsumer;

/**
 * The real 2 Tip auto (BlueAuto2Tip / RedAuto2Tip: MainAuto2Tip, MainAuto, Robot, Pedro, Ivy, NextFTC) in SimLoop,
 * checked on the plant's true pose. Each alliance runs twice; each run must get from INIT to STOP without an exception
 * (an OpModeException fails the test), mark every routine step in order (StepMarks) and reach routine-end before 30 s
 * of active time, keep every footprint corner inside the walls (0.5 in tolerance) and off the hive frame on every tick,
 * and end within 2 in and 2 deg of the route's park pose. A report (step times, volleys and the cell each aims at, the
 * tip odds for those volleys at hit rates 1.0, 0.9, 0.8 and 0.7 (HiveTipModel), park arrival, closest approach to a
 * wall and to the frame) is printed before anything is asserted.
 *
 * <p>The first run of each alliance is logged for AdvantageScope (SimLog: TeamCode/build/sim/blue-2-tip.rlog and
 * red-2-tip.rlog) and the second is not; read back through SimLoop's decoder, the log must hold every key and the
 * run's start and end poses, and the logged run must end at the same pose as the unlogged one (logging changes
 * nothing).
 *
 * <p>Only the end pose repeats exactly, not the timeline: Ivy 1.1.1's Race, Parallel and Deadline keep their children
 * in a HashMap keyed by identity hash, so the order they run in within a loop changes from run to run (on the robot
 * too), and each one can end a tick earlier or later. Every one of them ends while the robot holds still, so the path,
 * and the pose it ends at, are the same. So times are printed and held to limits, never compared exactly.
 */
public class AutoSimTest {

    /** MainAuto2Tip's steps in routine order: the ids its StepMarks send. */
    static final List<String> STEP_IDS = Arrays.asList(
            "preload-volley", "intakeflower", "garden-intake", "gardenhiveBshoot", "volley-B1+turn",
            "flower-deploy", "hiveBshootflower1", "flower1-intake", "flower1hiveBshootcheck", "volley-B2",
            "hiveBshootcheckflower2prep", "flower2prepflower2", "flower2-intake", "flower-stow", "flower2hiveAshoot",
            "volley-B3", "hiveAshootparkprep", "parkpreppark");
    /** The steps that fire a volley. */
    static final List<String> VOLLEY_IDS = Arrays.asList("preload-volley", "volley-B1+turn", "volley-B2", "volley-B3");
    static final String PARK_STEP = "parkpreppark";

    /** The auto period. A run's last ACTIVE tick is at 29.98 s of active time: any routine-end is before it. */
    static final double AUTO_S = 30.0;
    /** A corner may be this far past a wall: plant and placement rounding (spec). */
    static final double WALL_TOLERANCE_IN = 0.5;
    static final double FRAME_MARGIN_IN = 0;
    /** Where the plant must end, against the route's park pose: catches a wrong drive slot, sign or frame offset. */
    static final double PARK_IN = 2;
    static final double PARK_DEG = 2;
    private static final double SAME_POSE = 1e-9;
    static final String CAVEAT = "Caveat: no contact model, no flower bases, pickups look cleaner than reality.";
    /** Hit rates per ball the report's tip odds are worked out at (spec). */
    static final double[] HIT_RATES = {1.0, 0.9, 0.8, 0.7};
    /** The step a run is in before PLAY, and after the routine is done. */
    static final String BEFORE_START = "before start";
    static final String AFTER_ROUTINE_END = "after routine-end";
    /**
     * How far the code's believed pose may sit from the true one at the end of a run (parked, so the one-tick lag
     * does not show; the OctoQuad reads to a millimetre): far less than any frame or unit mistake.
     */
    static final double BELIEF_IN = 1;
    static final double BELIEF_DEG = 1;
    private static final double M_PER_IN = 0.0254;

    private double[] savedFrame;

    @Before
    public void setUp() {
        savedFrame = new double[] {Field.FRAME_MIN_X, Field.FRAME_MAX_X, Field.FRAME_MIN_Y, Field.FRAME_MAX_Y};
        Field.FRAME_MIN_X = 52.5;
        Field.FRAME_MAX_X = 91.5;
        Field.FRAME_MIN_Y = 47.3;
        Field.FRAME_MAX_Y = 96.7;
    }

    @After
    public void tearDown() {
        Field.FRAME_MIN_X = savedFrame[0];
        Field.FRAME_MAX_X = savedFrame[1];
        Field.FRAME_MIN_Y = savedFrame[2];
        Field.FRAME_MAX_Y = savedFrame[3];
    }

    @Test
    public void blue2TipFinishesInTimeOffTheWallsAndTheHiveFrame() throws IOException {
        check("Blue 2 Tip", Alliance.BLUE, BlueAuto2TipSim.class, new AutoRoute.Blue(), "blue-2-tip.rlog");
    }

    @Test
    public void red2TipFinishesInTimeOffTheWallsAndTheHiveFrame() throws IOException {
        check("Red 2 Tip", Alliance.RED, RedAuto2TipSim.class, new AutoRoute.Red(), "red-2-tip.rlog");
    }

    private static void check(String name, Alliance alliance, Class<? extends OpMode> adapter, AutoRoute route,
                              String logName) throws IOException {
        Pose park = route.parkpreppark().endPose();
        Path rlog = Paths.get("build", "sim", logName);
        Run logged = Run.of(name + ", run 1 (logged)", alliance, adapter, route, rlog);
        logged.printReport(park);
        Run plain = Run.of(name + ", run 2 (not logged)", alliance, adapter, route, null);
        plain.printSummary();

        logged.assertChecks(park);
        plain.assertChecks(park);
        logged.assertLog(BiobuzzSim.exportStart(route));
        assertEquals(name + ": x differs between the logged and the unlogged run", logged.x, plain.x, SAME_POSE);
        assertEquals(name + ": y differs between the logged and the unlogged run", logged.y, plain.y, SAME_POSE);
        assertEquals(name + ": heading differs between the logged and the unlogged run", logged.heading, plain.heading,
                SAME_POSE);
    }

    /** Stands for a blackboard key that had no entry before a run. */
    private static final Object ABSENT = new Object();

    /** Puts value under key in OpMode.blackboard (null removes the key) and returns what was there, for putBack. */
    static Object swapIn(String key, Object value) {
        Object previous = OpMode.blackboard.containsKey(key) ? OpMode.blackboard.get(key) : ABSENT;
        if (value == null) {
            OpMode.blackboard.remove(key);
        } else {
            OpMode.blackboard.put(key, value);
        }
        return previous;
    }

    static void putBack(String key, Object previous) {
        if (previous == ABSENT) {
            OpMode.blackboard.remove(key);
        } else {
            OpMode.blackboard.put(key, previous);
        }
    }

    /** The cell a volley aims at by the side rule: blue's A is above y = 72, red's (rotated 180) below. */
    static HiveCell cellOnSide(Alliance alliance, double plantY) {
        boolean upper = plantY > Field.SIZE_IN / 2;
        return upper == (alliance == Alliance.BLUE) ? HiveCell.A : HiveCell.B;
    }

    /** A step mark as the sim saw it: active time, step id, event, and the plant's true pose (Pedro frame). */
    static final class Mark {
        final double t;
        final String id;
        final String event;
        final double x;
        final double y;
        final double heading;

        Mark(double t, String id, String event, double x, double y, double heading) {
            this.t = t;
            this.id = id;
            this.event = event;
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        @Override
        public String toString() {
            return id + " " + event + " at " + time(t) + " s";
        }
    }

    /** One volley step: when it ran and the cell it aimed at by the side rule (plant y at its start). */
    static final class Volley {
        final String id;
        final double start;
        final double end;
        final double y;
        final HiveCell cell;

        Volley(String id, double start, double end, double y, HiveCell cell) {
            this.id = id;
            this.start = start;
            this.end = end;
            this.y = y;
            this.cell = cell;
        }
    }

    /** One tick's closeness to a wall or the frame, inches (negative = past it), and where the plant was. */
    static final class Where {
        final String when;
        final String step;
        final double x;
        final double y;
        final double heading;
        final double clearance;

        Where(String when, String step, double x, double y, double heading, double clearance) {
            this.when = when;
            this.step = step;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.clearance = clearance;
        }

        @Override
        public String toString() {
            return String.format("%.2f in at %s, step %s, plant (%.2f, %.2f, %.1f deg)",
                    clearance, when, step, x, y, degrees(heading));
        }
    }

    /**
     * One run as the test saw it. It is the sink StepMarks sends to (through OpMode.blackboard), and it looks at the
     * plant's true pose after every tick. Marks come on the OpMode's thread and ticks on the runner's, one at a time
     * (SimLoop runs them in lockstep); synchronized all the same.
     */
    static final class Run implements BiConsumer<String, String> {

        final String name;
        final Alliance alliance;
        /** Where this run is logged for AdvantageScope; null for a run that is not logged. */
        final Path rlog;
        final BiobuzzSim sim = new BiobuzzSim();
        private final List<Mark> marks = new ArrayList<>();
        private final double[] corners = new double[8];
        /**
         * Sim time at the first ACTIVE tick (PLAY: start() schedules the routine); 0.54 s after 1 INIT and 25 INIT_LOOP
         * ticks, since the runner advances the timer before each tick's code. Active time counts from here.
         */
        private double activeStart = Double.NaN;
        private OpModePhase phase = OpModePhase.INIT;
        /** The step started last (one ends and the next starts in the same loop). */
        private String step = BEFORE_START;
        /** What the code believed after its latest loop (NextOpModeSim's belief sink; logged runs only), or null. */
        private double[] believedPose;
        private String believedCell;
        private Where closestToWall;
        private Where closestToFrame;
        private int ticksPastWall;
        private int ticksOnFrame;
        OpModeRunResult result;
        double wallClockS;
        /** True pose at the end of the run, Pedro frame; heading not wrapped. */
        double x;
        double y;
        double heading;
        /** The flywheel model's speed at the end of the run, RPM. */
        double flywheelRpm;

        private Run(String name, Alliance alliance, Path rlog) {
            this.name = name;
            this.alliance = alliance;
            this.rlog = rlog;
        }

        /**
         * Runs the auto once on a fresh world placed at the route's start, with this run as the blackboard's step
         * sink (the runner keeps the blackboard; the previous values are put back after). With an rlog path, the run
         * is logged there for AdvantageScope (OpModeRunner.runLogged), with this run as the belief sink too; without
         * one it runs unlogged, with no belief sink.
         */
        static Run of(String name, Alliance alliance, Class<? extends OpMode> adapter, Object export, Path rlog) {
            final Run run = new Run(name, alliance, rlog);
            run.sim.placeAtStartOf(export);
            if (rlog != null) {
                try {
                    Files.deleteIfExists(rlog);
                } catch (IOException e) {
                    throw new UncheckedIOException("Cannot delete the old RLOG " + rlog.toAbsolutePath(), e);
                }
            }
            Object savedSteps = swapIn(StepMarks.SINK_KEY, run);
            BiConsumer<double[], String> beliefSink = run::onBelief;
            Object savedBelief = swapIn(NextOpModeSim.BELIEF_SINK_KEY, rlog != null ? beliefSink : null);
            long startNanos = System.nanoTime();
            try {
                OpModeRunner runner = run.sim.runner(adapter, dt -> run.afterStep())
                        .gamepads((phase, tickInPhase, gamepad1, gamepad2) -> run.onTick(phase, tickInPhase));
                run.result = rlog != null ? runner.runLogged(BiobuzzSim.ROBOT_CONFIG, rlog) : runner.run();
            } finally {
                putBack(StepMarks.SINK_KEY, savedSteps);
                putBack(NextOpModeSim.BELIEF_SINK_KEY, savedBelief);
            }
            run.wallClockS = (System.nanoTime() - startNanos) / 1e9;
            run.x = run.sim.pedroX();
            run.y = run.sim.pedroY();
            run.heading = run.sim.heading();
            run.flywheelRpm = run.sim.flywheelRpm();
            return run;
        }

        /** Start of every tick, before the OpMode's code (SimLoop's gamepad script hook; the gamepads stay at rest). */
        private synchronized void onTick(OpModePhase phase, int tickInPhase) {
            this.phase = phase;
            if (phase == OpModePhase.ACTIVE && tickInPhase == 0) {
                activeStart = sim.timer.time();
            }
        }

        /** What the code believes after a loop pass, from inside the run (NextOpModeSim.reportBelief). */
        private synchronized void onBelief(double[] pose, String cell) {
            believedPose = pose.clone();
            believedCell = cell;
        }

        /** A step mark, from inside the run (StepMarks.mark). */
        @Override
        public synchronized void accept(String id, String event) {
            marks.add(new Mark(activeTime(), id, event, sim.pedroX(), sim.pedroY(), sim.heading()));
            if (StepMarks.ROUTINE_END.equals(id)) {
                step = AFTER_ROUTINE_END;
            } else if (StepMarks.START.equals(event)) {
                step = id;
            }
        }

        /**
         * After every tick's world step: the walls and the hive frame, on the true pose; then the tick goes into the
         * AdvantageScope log, if this run is logged (SimLog).
         */
        private synchronized void afterStep() {
            double px = sim.pedroX();
            double py = sim.pedroY();
            double ph = sim.heading();
            Field.footprintCorners(px, py, ph, corners);
            double wall = Double.MAX_VALUE;
            double minX = Double.MAX_VALUE;
            double maxX = -Double.MAX_VALUE;
            double minY = Double.MAX_VALUE;
            double maxY = -Double.MAX_VALUE;
            for (int i = 0; i < 8; i += 2) {
                double cx = corners[i];
                double cy = corners[i + 1];
                wall = Math.min(wall, Math.min(Math.min(cx, Field.SIZE_IN - cx), Math.min(cy, Field.SIZE_IN - cy)));
                minX = Math.min(minX, cx);
                maxX = Math.max(maxX, cx);
                minY = Math.min(minY, cy);
                maxY = Math.max(maxY, cy);
            }
            double frame = Math.max(Math.max(Field.FRAME_MIN_X - maxX, minX - Field.FRAME_MAX_X),
                    Math.max(Field.FRAME_MIN_Y - maxY, minY - Field.FRAME_MAX_Y));
            if (!Field.insideWalls(px, py, ph, -WALL_TOLERANCE_IN)) {
                ticksPastWall++;
            }
            if (!Field.clearOfFrame(px, py, ph, FRAME_MARGIN_IN)) {
                ticksOnFrame++;
            }
            String when = phase == OpModePhase.ACTIVE ? time(activeTime()) + " s" : phase.toString();
            if (closestToWall == null || wall < closestToWall.clearance) {
                closestToWall = new Where(when, step, px, py, ph, wall);
            }
            if (closestToFrame == null || frame < closestToFrame.clearance) {
                closestToFrame = new Where(when, step, px, py, ph, frame);
            }
            SimLog.record(sim, believedPose, believedCell, step);
        }

        private double activeTime() {
            return sim.timer.time() - activeStart;
        }

        synchronized List<Mark> marks() {
            return new ArrayList<>(marks);
        }

        Mark find(String id, String event) {
            for (Mark mark : marks()) {
                if (mark.id.equals(id) && mark.event.equals(event)) {
                    return mark;
                }
            }
            return null;
        }

        Mark routineEnd() {
            return find(StepMarks.ROUTINE_END, StepMarks.END);
        }

        List<Volley> volleys() {
            List<Volley> volleys = new ArrayList<>();
            for (String id : VOLLEY_IDS) {
                Mark start = find(id, StepMarks.START);
                Mark end = find(id, StepMarks.END);
                if (start != null) {
                    volleys.add(new Volley(id, start.t, end != null ? end.t : Double.NaN, start.y,
                            cellOnSide(alliance, start.y)));
                }
            }
            return volleys;
        }

        void printReport(Pose park) {
            Mark end = routineEnd();
            System.out.printf("%s: %.2f s wall clock, %d ticks (%.2f s sim), routine-end at %s s of %.0f;"
                            + " warnings %s%n", name, wallClockS, result.ticks(), result.finalTimeSeconds(),
                    end != null ? time(end.t) : "never", AUTO_S, result.warnings());
            System.out.printf("  %-28s %7s %7s %7s%n", "step", "start", "end", "took");
            for (String id : STEP_IDS) {
                Mark start = find(id, StepMarks.START);
                Mark stop = find(id, StepMarks.END);
                System.out.printf("  %-28s %7s %7s %7s%n", id, start != null ? time(start.t) : "-",
                        stop != null ? time(stop.t) : "-",
                        start != null && stop != null ? time(stop.t - start.t) : "-");
            }
            System.out.printf("  %-28s %7s%n", StepMarks.ROUTINE_END, end != null ? time(end.t) : "-");
            List<Volley> volleys = volleys();
            for (Volley volley : volleys) {
                System.out.printf("  volley %s, %s-%s s, plant y %.1f: cell %s%n", volley.id, time(volley.start),
                        time(volley.end), volley.y, volley.cell);
            }
            printTipOdds(volleys);
            Mark arrived = find(PARK_STEP, StepMarks.END);
            if (arrived != null) {
                System.out.printf("  park: %s ended at %s s, plant %.2f in and %.1f deg from the park pose"
                                + " (%.2f, %.2f, %.1f deg); end pose (%.2f, %.2f, %.1f deg)%n",
                        PARK_STEP, time(arrived.t), Math.hypot(arrived.x - park.x(), arrived.y - park.y()),
                        degreesOff(arrived.heading, park.heading()), park.x(), park.y(), degrees(park.heading()),
                        x, y, degrees(heading));
            } else {
                System.out.printf("  park: %s never ended; end pose (%.2f, %.2f, %.1f deg)%n", PARK_STEP, x, y,
                        degrees(heading));
            }
            System.out.println("  closest to a wall: " + closestToWall);
            System.out.println("  closest to the hive frame (bounding box): " + closestToFrame);
            System.out.println("  last telemetry: " + result.telemetry().lastTransmission());
            if (rlog != null) {
                System.out.println("  AdvantageScope log: " + rlog.toAbsolutePath()
                        + (Files.exists(rlog) ? "" : " (not written)"));
            }
            System.out.println("  " + CAVEAT);
        }

        void printSummary() {
            Mark end = routineEnd();
            System.out.printf("%s: %.2f s wall clock, routine-end at %s s, end pose (%.2f, %.2f, %.1f deg);"
                            + " closest to a wall %.2f in, to the frame %.2f in; warnings %s%n", name, wallClockS,
                    end != null ? time(end.t) : "never", x, y, degrees(heading), closestToWall.clearance,
                    closestToFrame.clearance, result.warnings());
        }

        void assertChecks(Pose park) {
            List<Mark> seen = marks();
            Mark end = routineEnd();
            assertNotNull(name + ": routine-end never came; last mark: "
                    + (seen.isEmpty() ? "none" : seen.get(seen.size() - 1)), end);
            assertTrue(name + ": routine-end at " + time(end.t) + " s, not before " + AUTO_S + " s", end.t < AUTO_S);

            List<String> expected = new ArrayList<>();
            for (String id : STEP_IDS) {
                expected.add(id + " " + StepMarks.START);
                expected.add(id + " " + StepMarks.END);
            }
            expected.add(StepMarks.ROUTINE_END + " " + StepMarks.END);
            List<String> actual = new ArrayList<>();
            for (Mark mark : seen) {
                actual.add(mark.id + " " + mark.event);
            }
            assertEquals(name + ": step marks out of routine order", expected, actual);

            assertEquals(name + ": ticks with a corner more than " + WALL_TOLERANCE_IN + " in past a wall; closest: "
                    + closestToWall, 0, ticksPastWall);
            assertEquals(name + ": ticks on the hive frame; closest: " + closestToFrame, 0, ticksOnFrame);

            double offIn = Math.hypot(x - park.x(), y - park.y());
            double offDeg = degreesOff(heading, park.heading());
            assertTrue(String.format("%s: ends %.2f in from the park pose (%.2f, %.2f), at (%.2f, %.2f)",
                    name, offIn, park.x(), park.y(), x, y), offIn <= PARK_IN);
            assertTrue(String.format("%s: ends %.2f deg off the park heading %.1f deg, at %.1f deg",
                    name, offDeg, degrees(park.heading()), degrees(heading)), offDeg <= PARK_DEG);
        }

        /**
         * The run's AdvantageScope log, read back through SimLoop's decoder (RlogDecodedCompare): there and not empty;
         * every SimLog key in it; the first frame's true pose at the start (INIT: the robot has not moved) and the last
         * frame's at the end pose; the believed pose next to the true one at the end; the code's cell on the side
         * rule's at the start and the end; the flywheel speed and the step the run ended with.
         */
        void assertLog(Pose start) throws IOException {
            String where = rlog.toAbsolutePath().toString();
            assertTrue(name + ": no RLOG at " + where, Files.exists(rlog));
            assertTrue(name + ": empty RLOG at " + where, Files.size(rlog) > 0);
            final Set<String> keys = new TreeSet<>();
            final List<Map<String, String>> ends = new ArrayList<>();
            RlogDecodedCompare.forEachFrame(rlog, frame -> {
                keys.addAll(frame.keySet());
                if (ends.size() == 2) {
                    ends.set(1, frame);
                } else {
                    ends.add(frame);
                }
            });
            for (String key : SimLog.KEYS) {
                assertTrue(name + ": " + where + " never logs " + key + "; its keys: " + keys,
                        keys.contains(SimLog.READ_PREFIX + key));
            }
            Map<String, String> first = ends.get(0);
            Map<String, String> last = ends.get(ends.size() - 1);

            assertLoggedPose(name + ": the first frame's true pose (INIT, at the start)",
                    SimLog.loggedPose(start.x(), start.y(), start.heading()), pose(first, SimLog.TRUE_POSE));
            Pose2d trueEnd = pose(last, SimLog.TRUE_POSE);
            assertLoggedPose(name + ": the last frame's true pose (the end pose)", SimLog.loggedPose(x, y, heading),
                    trueEnd);
            Pose2d believedEnd = pose(last, SimLog.BELIEVED_POSE);
            double offIn = Math.hypot(believedEnd.getX() - trueEnd.getX(), believedEnd.getY() - trueEnd.getY())
                    / M_PER_IN;
            double offDeg = degreesOff(believedEnd.getRotation().getRadians(), trueEnd.getRotation().getRadians());
            assertTrue(String.format("%s: the believed pose ends %.2f in and %.2f deg from the true pose (%s vs %s)",
                    name, offIn, offDeg, believedEnd, trueEnd), offIn <= BELIEF_IN && offDeg <= BELIEF_DEG);

            assertEquals(name + ": the cell the code aims at in the first frame",
                    cellOnSide(alliance, start.y()).name(), value(first, SimLog.TARGET_CELL));
            assertEquals(name + ": the cell the code aims at in the last frame",
                    cellOnSide(alliance, y).name(), value(last, SimLog.TARGET_CELL));
            assertEquals(name + ": the flywheel speed in the last frame", flywheelRpm,
                    Double.parseDouble(value(last, SimLog.FLYWHEEL_RPM)), 0);
            assertEquals(name + ": the step in the last frame", AFTER_ROUTINE_END, value(last, SimLog.STEP));
        }

        private String value(Map<String, String> frame, String key) {
            String value = frame.get(SimLog.READ_PREFIX + key);
            assertNotNull(name + ": no " + key + " in the frame; its keys: " + frame.keySet(), value);
            return value;
        }

        private Pose2d pose(Map<String, String> frame, String key) {
            return SimLog.decodePose(value(frame, key));
        }
    }

    /** A logged pose (metres, radians) against the one expected, to SAME_POSE. */
    static void assertLoggedPose(String what, Pose2d expected, Pose2d actual) {
        String message = what + ": expected " + expected + ", logged " + actual;
        assertEquals(message, expected.getX(), actual.getX(), SAME_POSE);
        assertEquals(message, expected.getY(), actual.getY(), SAME_POSE);
        assertEquals(message, expected.getRotation().getRadians(), actual.getRotation().getRadians(), SAME_POSE);
    }

    /**
     * The tip odds for a run's volleys (HiveTipModel) at each of HIT_RATES: the cells they aimed at by the side rule,
     * the preloads first, then DEFAULT_PICKUP balls for every later volley. A down-cell volley is a note, not a failure.
     */
    static void printTipOdds(List<Volley> volleys) {
        List<HiveCell> aimed = new ArrayList<>();
        for (Volley volley : volleys) {
            aimed.add(volley.cell);
        }
        int[] balls = HiveTipModel.ballsPerVolley(aimed.size(), HiveTipModel.DEFAULT_PICKUP);
        System.out.printf("  tip odds (HiveTipModel, exact): %d preloads, then %d per pickup (a guess); A tips at %d"
                        + " pollen on its staged nectar, every later cell at %d; the rest of a tipping volley is lost%n",
                HiveTipModel.PRELOADS, HiveTipModel.DEFAULT_PICKUP, HiveTipModel.FIRST_TIP_POLLEN,
                HiveTipModel.LATER_TIP_POLLEN);
        System.out.printf("  %5s  %-16s %4s %5s %11s %11s %7s %12s%n", "hit", "volley", "cell", "balls",
                "P(A tipped)", "P(B tipped)", "E[tips]", "P(down cell)");
        for (double p : HIT_RATES) {
            List<HiveTipModel.VolleyOdds> odds = HiveTipModel.odds(aimed, balls, p);
            for (int i = 0; i < odds.size(); i++) {
                HiveTipModel.VolleyOdds volley = odds.get(i);
                System.out.printf("  %5.2f  %-16s %4s %5d %11.4f %11.4f %7.4f %12.4f%s%n", p, volleys.get(i).id,
                        volley.aimed, volley.balls, volley.pTipped(HiveCell.A), volley.pTipped(HiveCell.B),
                        volley.expectedTips, volley.pDownCell, volley.flagged() ? "  G417 note" : "");
            }
        }
        System.out.println("  P(down cell): the chance the volley flies at a down cell, where it scores nothing (a G417"
                + " note, not a failure).");
        System.out.println("  Tip odds assume each ball goes in independently (an upper bound when misses come"
                + " together), a calibrated field and no partner shots.");
    }

    static String time(double seconds) {
        return String.format("%.2f", seconds);
    }

    /** Degrees wrapped to [-180, 180). */
    static double degrees(double radians) {
        return MathUtil.wrapDegrees(Math.toDegrees(radians));
    }

    static double degreesOff(double radians, double targetRadians) {
        return Math.abs(Math.toDegrees(MathUtil.wrapRadians(radians - targetRadians)));
    }
}
