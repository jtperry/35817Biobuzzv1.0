package org.firstinspires.ftc.teamcode.autos;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.field.Field;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;

/** The 2 Tip route: a chain, red = blue turned 180 degrees, starts at the start pose, never hits a wall or the hive frame. */
public class AutoRouteTest {

    /** Path names in MainAuto2Tip's routine order. Keep in step with inRoutineOrder and the routine. */
    private static final List<String> NAMES = Arrays.asList(
            "intakeflower", "gardenhiveBshoot", "hiveBshootflower1", "flower1hiveBshootcheck",
            "hiveBshootcheckflower2prep", "flower2prepflower2", "flower2hiveAshoot", "hiveAshootparkprep",
            "parkpreppark");
    private static final int SAMPLES = 500;
    private static final double EPS = 1e-6;

    private double[] savedFrame;

    @Before
    public void setUp() {
        savedFrame = new double[] {Field.FRAME_MIN_X, Field.FRAME_MAX_X, Field.FRAME_MIN_Y, Field.FRAME_MAX_Y};
        // Field.java's defaults (approximate, spec 11), as FieldTest and ShotTunerLaneTest set them.
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

    private static List<Path> inRoutineOrder(AutoRoute route) {
        return Arrays.asList(route.intakeflower(), route.gardenhiveBshoot(), route.hiveBshootflower1(),
                route.flower1hiveBshootcheck(), route.hiveBshootcheckflower2prep(), route.flower2prepflower2(),
                route.flower2hiveAshoot(), route.hiveAshootparkprep(), route.parkpreppark());
    }

    private static List<AutoRoute> routes() {
        return Arrays.asList(new AutoRoute.Blue(), new AutoRoute.Red());
    }

    private static String name(AutoRoute route) {
        return route instanceof AutoRoute.Red ? "red" : "blue";
    }

    /** Angle wrapped to (-pi, pi]. Pedro keeps headings in [0, 2 pi). */
    private static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    @Test
    public void pedroPathBuildsAndSamplesOnTheJvm() {
        Path path = Paths.line(new Pose(0, 0, 0), new Pose(10, 0, 0)).constant(0);
        Pose middle = path.get(0.5);
        assertEquals(5, middle.x(), EPS);
        assertEquals(0, middle.y(), EPS);
    }

    @Test
    public void eachPathStartsWhereThePreviousEnds() {
        for (AutoRoute route : routes()) {
            List<Path> paths = inRoutineOrder(route);
            for (int i = 1; i < paths.size(); i++) {
                Pose end = paths.get(i - 1).endPose();
                Pose start = paths.get(i).get(0);
                String what = name(route) + " " + NAMES.get(i - 1) + " -> " + NAMES.get(i);
                // Positions only: some headings jump between paths by design.
                assertEquals(what + " x", end.x(), start.x(), EPS);
                assertEquals(what + " y", end.y(), start.y(), EPS);
            }
        }
    }

    @Test
    public void startPoseIsTheFirstPathsStart() {
        for (AutoRoute route : routes()) {
            Pose start = StartPose.of(route);
            Pose first = route.intakeflower().get(0);
            assertEquals(name(route) + " x", start.x(), first.x(), EPS);
            assertEquals(name(route) + " y", start.y(), first.y(), EPS);
            assertEquals(name(route) + " heading", 0, wrap(start.heading() - first.heading()), EPS);
        }
    }

    @Test
    public void redIsBlueTurned180AboutTheCenter() {
        List<Path> blue = inRoutineOrder(new AutoRoute.Blue());
        List<Path> red = inRoutineOrder(new AutoRoute.Red());
        for (int i = 0; i < blue.size(); i++) {
            for (int s = 0; s <= SAMPLES; s++) {
                double t = (double) s / SAMPLES;
                Pose b = blue.get(i).get(t);
                Pose r = red.get(i).get(t);
                String what = NAMES.get(i) + " t=" + t;
                assertEquals(what + " x", Field.SIZE_IN - b.x(), r.x(), EPS);
                assertEquals(what + " y", Field.SIZE_IN - b.y(), r.y(), EPS);
                assertEquals(what + " heading", 0, wrap(r.heading() - b.heading() - Math.PI), EPS);
            }
        }
    }

    /** MainAuto2Tip turns in place at the B spot (during the volley) to flower 1's heading before driving in. */
    @Test
    public void turnAtTheBSpotIsClear() {
        for (AutoRoute route : routes()) {
            Pose from = route.gardenhiveBshoot().endPose();
            Pose to = route.hiveBshootflower1().get(0);
            double sweep = wrap(to.heading() - from.heading());
            for (int s = 0; s <= SAMPLES; s++) {
                double heading = from.heading() + sweep * s / SAMPLES;
                String what = name(route) + " turn at the B spot, heading " + Math.toDegrees(heading) + " deg";
                assertTrue(what + ": a footprint corner is past a wall",
                        Field.insideWalls(to.x(), to.y(), heading, 0));
                assertTrue(what + ": the footprint overlaps the hive frame",
                        Field.clearOfFrame(to.x(), to.y(), heading, 0));
            }
        }
    }

    @Test
    public void robotStaysInsideTheWallsAndOffTheHiveFrame() {
        for (AutoRoute route : routes()) {
            List<Path> paths = inRoutineOrder(route);
            for (int i = 0; i < paths.size(); i++) {
                for (int s = 0; s <= SAMPLES; s++) {
                    double t = (double) s / SAMPLES;
                    Pose p = paths.get(i).get(t);
                    String what = name(route) + " " + NAMES.get(i) + " t=" + t + " at (" + p.x() + ", " + p.y()
                            + ", " + Math.toDegrees(p.heading()) + " deg)";
                    assertTrue(what + ": a footprint corner is past a wall",
                            Field.insideWalls(p.x(), p.y(), p.heading(), 0));
                    assertTrue(what + ": the footprint overlaps the hive frame",
                            Field.clearOfFrame(p.x(), p.y(), p.heading(), 0));
                }
            }
        }
    }
}
