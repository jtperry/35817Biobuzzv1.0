package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

public class ShotTunerLaneTest {

    private static final double EPS = 1e-9;

    // Spec 7.3, blue, West: x, y, heading degrees for 30, 35, ... 80.
    private static final double[][] BLUE_WEST = {
            {84.2, 116, -90}, {84.2, 121, -90}, {64.8, 121, -61}, {55.9, 121, -51}, {48.5, 121, -44},
            {41.8, 121, -40}, {35.5, 121, -36}, {29.4, 121, -33}, {23.6, 121, -30}, {17.9, 121, -28},
            {12.3, 121, -26},
    };

    private double[] saved;

    @Before
    public void setUp() {
        saved = new double[] {
                Field.BLUE_CELL_A_X, Field.BLUE_CELL_A_Y,
                Field.FRAME_MIN_X, Field.FRAME_MAX_X, Field.FRAME_MIN_Y, Field.FRAME_MAX_Y,
                ShotTunerLane.START_IN, ShotTunerLane.END_IN, ShotTunerLane.STEP_IN, ShotTunerLane.BACK_DEG,
                ShotTunerLane.TURN_IN, ShotTunerLane.SIDE_DEG, ShotTunerLane.WALL_MARGIN_IN,
                ShotTunerLane.FRAME_MARGIN_IN,
        };
        Field.BLUE_CELL_A_X = 84.2;
        Field.BLUE_CELL_A_Y = 86.0;
        Field.FRAME_MIN_X = 52.5;
        Field.FRAME_MAX_X = 91.5;
        Field.FRAME_MIN_Y = 47.3;
        Field.FRAME_MAX_Y = 96.7;
        ShotTunerLane.START_IN = 30;
        ShotTunerLane.END_IN = 80;
        ShotTunerLane.STEP_IN = 5;
        ShotTunerLane.BACK_DEG = 90;
        ShotTunerLane.TURN_IN = 35;
        ShotTunerLane.SIDE_DEG = 180;
        ShotTunerLane.WALL_MARGIN_IN = 1;
        ShotTunerLane.FRAME_MARGIN_IN = 2;
    }

    @After
    public void tearDown() {
        Field.BLUE_CELL_A_X = saved[0];
        Field.BLUE_CELL_A_Y = saved[1];
        Field.FRAME_MIN_X = saved[2];
        Field.FRAME_MAX_X = saved[3];
        Field.FRAME_MIN_Y = saved[4];
        Field.FRAME_MAX_Y = saved[5];
        ShotTunerLane.START_IN = saved[6];
        ShotTunerLane.END_IN = saved[7];
        ShotTunerLane.STEP_IN = saved[8];
        ShotTunerLane.BACK_DEG = saved[9];
        ShotTunerLane.TURN_IN = saved[10];
        ShotTunerLane.SIDE_DEG = saved[11];
        ShotTunerLane.WALL_MARGIN_IN = saved[12];
        ShotTunerLane.FRAME_MARGIN_IN = saved[13];
    }

    private static void assertHeading(double expected, double actual, double tolerance) {
        assertEquals(0.0, MathUtil.wrapRadians(expected - actual), tolerance);
    }

    @Test
    public void blueWestMatchesTheSpecList() {
        List<ShotTunerLane.Spot> spots = ShotTunerLane.spots(Alliance.BLUE);
        assertEquals(11, spots.size());
        for (int i = 0; i < 11; i++) {
            ShotTunerLane.Spot spot = spots.get(i);
            assertEquals(30.0 + 5 * i, spot.nominalIn, 0.0);
            assertEquals(BLUE_WEST[i][0], spot.x, 0.1);
            assertEquals(BLUE_WEST[i][1], spot.y, 0.1);
            assertHeading(Math.toRadians(BLUE_WEST[i][2]), spot.heading, Math.toRadians(1));
            assertTrue("spot " + spot.nominalIn + ": " + spot.reason, spot.valid);
            assertEquals("", spot.reason);
        }
    }

    @Test
    public void blueEastHitsTheWallFrom60() {
        ShotTunerLane.SIDE_DEG = 0;
        List<ShotTunerLane.Spot> spots = ShotTunerLane.spots(Alliance.BLUE);
        assertEquals(11, spots.size());
        assertEquals(126.6, spots.get(5).x, 0.1);
        assertEquals(121.0, spots.get(5).y, 0.1);
        for (ShotTunerLane.Spot spot : spots) {
            if (spot.nominalIn <= 55) {
                assertTrue("spot " + spot.nominalIn + ": " + spot.reason, spot.valid);
            } else {
                assertFalse("spot " + spot.nominalIn, spot.valid);
                assertEquals("wall", spot.reason);
            }
        }
    }

    @Test
    public void redIsBlueRotated180() {
        for (double side : new double[] {180, 0}) {
            ShotTunerLane.SIDE_DEG = side;
            List<ShotTunerLane.Spot> blue = ShotTunerLane.spots(Alliance.BLUE);
            List<ShotTunerLane.Spot> red = ShotTunerLane.spots(Alliance.RED);
            assertEquals(blue.size(), red.size());
            for (int i = 0; i < blue.size(); i++) {
                ShotTunerLane.Spot b = blue.get(i);
                ShotTunerLane.Spot r = red.get(i);
                assertEquals(b.nominalIn, r.nominalIn, 0.0);
                assertEquals(Field.SIZE_IN - b.x, r.x, EPS);
                assertEquals(Field.SIZE_IN - b.y, r.y, EPS);
                assertHeading(b.heading + Math.PI, r.heading, EPS);
                assertEquals(b.valid, r.valid);
                assertEquals(b.reason, r.reason);
            }
        }
    }

    @Test
    public void everyValidSpotIsItsNominalDistanceFromCellAAndFacesIt() {
        for (double side : new double[] {180, 0}) {
            ShotTunerLane.SIDE_DEG = side;
            for (Alliance alliance : Alliance.values()) {
                Pose cell = ShotTunerLane.cellA(alliance);
                for (ShotTunerLane.Spot spot : ShotTunerLane.spots(alliance)) {
                    if (!spot.valid) {
                        continue;
                    }
                    assertEquals(spot.nominalIn, Math.hypot(spot.x - cell.x(), spot.y - cell.y()), EPS);
                    assertHeading(Math.atan2(cell.y() - spot.y, cell.x() - spot.x), spot.heading, EPS);
                }
            }
        }
    }

    @Test
    public void lanePointsAndPoses() {
        Pose blueCell = ShotTunerLane.cellA(Alliance.BLUE);
        assertEquals(84.2, blueCell.x(), EPS);
        assertEquals(86.0, blueCell.y(), EPS);
        Pose redCell = ShotTunerLane.cellA(Alliance.RED);
        assertEquals(59.8, redCell.x(), EPS);
        assertEquals(58.0, redCell.y(), EPS);
        Pose blueTurn = ShotTunerLane.turnPoint(Alliance.BLUE);
        assertEquals(84.2, blueTurn.x(), EPS);
        assertEquals(121.0, blueTurn.y(), EPS);
        assertHeading(0.0, blueTurn.heading(), EPS);
        Pose redTurn = ShotTunerLane.turnPoint(Alliance.RED);
        assertEquals(59.8, redTurn.x(), EPS);
        assertEquals(23.0, redTurn.y(), EPS);
        assertHeading(0.0, redTurn.heading(), EPS);
        ShotTunerLane.Spot spot = ShotTunerLane.spots(Alliance.RED).get(4);
        Pose pose = ShotTunerLane.pose(spot);
        assertEquals(spot.x, pose.x(), EPS);
        assertEquals(spot.y, pose.y(), EPS);
        assertHeading(spot.heading, pose.heading(), EPS);
    }

    @Test
    public void nominalsAreRoundedToATenthAndIncludeTheEnd() {
        // Back-leg distances, so the geometry is trivial. (0.7 - 0.1) / 0.1 is 5.999999999999999, which needs
        // the 1e-6 tolerance to give 7 spots; 0.1 + 2 * 0.1 is 0.30000000000000004, which needs the rounding.
        ShotTunerLane.START_IN = 0.1;
        ShotTunerLane.END_IN = 0.7;
        ShotTunerLane.STEP_IN = 0.1;
        List<ShotTunerLane.Spot> spots = ShotTunerLane.spots(Alliance.BLUE);
        assertEquals(7, spots.size());
        assertEquals(0.3, spots.get(2).nominalIn, 0.0);
        assertEquals(0.7, spots.get(6).nominalIn, 0.0);
    }

    @Test
    public void aStepThatIsNotPositiveGivesTheStartAlone() {
        ShotTunerLane.STEP_IN = 0;
        List<ShotTunerLane.Spot> spots = ShotTunerLane.spots(Alliance.BLUE);
        assertEquals(1, spots.size());
        assertEquals(30.0, spots.get(0).nominalIn, 0.0);
    }

    @Test
    public void aTinyStepIsCappedSoEditingPanelsCannotHang() {
        ShotTunerLane.STEP_IN = 1e-9;
        assertEquals(1000, ShotTunerLane.spots(Alliance.BLUE).size());
        ShotTunerLane.STEP_IN = 5;
        ShotTunerLane.END_IN = Double.POSITIVE_INFINITY;
        assertEquals(1000, ShotTunerLane.spots(Alliance.BLUE).size());
    }

    @Test
    public void tooCloseHitsTheHiveFrame() {
        ShotTunerLane.START_IN = 20;
        ShotTunerLane.Spot spot = ShotTunerLane.spots(Alliance.BLUE).get(0);
        assertEquals(20.0, spot.nominalIn, 0.0);
        assertFalse(spot.valid);
        assertEquals("hive frame", spot.reason);
    }

    @Test
    public void aDistanceTheSideLegCannotReachIsOffLane() {
        ShotTunerLane.TURN_IN = -100;
        for (ShotTunerLane.Spot spot : ShotTunerLane.spots(Alliance.BLUE)) {
            assertFalse(spot.valid);
            assertEquals("off lane", spot.reason);
            assertEquals(84.2, spot.x, EPS);
            assertEquals(-14.0, spot.y, EPS);
        }
    }
}
