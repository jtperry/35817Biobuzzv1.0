package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.junit.Before;
import org.junit.Test;

public class FieldTest {

    private static final double EPS = 1e-9;

    @Before
    public void setUp() {
        Field.BLUE_CELL_A_X = 84.2;
        Field.BLUE_CELL_A_Y = 86.0;
        Field.SIDE_SWITCH_DEADBAND_IN = 2.0;
        Field.FRAME_MIN_X = 52.5;
        Field.FRAME_MAX_X = 91.5;
        Field.FRAME_MIN_Y = 47.3;
        Field.FRAME_MAX_Y = 96.7;
    }

    private static void assertHeading(double expected, double actual) {
        assertEquals(0.0, MathUtil.wrapRadians(expected - actual), EPS);
    }

    @Test
    public void cellsFollowTheMirrorRules() {
        assertEquals(84.2, Field.cellX(Alliance.BLUE, HiveCell.A), EPS);
        assertEquals(86.0, Field.cellY(Alliance.BLUE, HiveCell.A), EPS);
        assertEquals(84.2, Field.cellX(Alliance.BLUE, HiveCell.B), EPS);
        assertEquals(58.0, Field.cellY(Alliance.BLUE, HiveCell.B), EPS);
        assertEquals(59.8, Field.cellX(Alliance.RED, HiveCell.A), EPS);
        assertEquals(58.0, Field.cellY(Alliance.RED, HiveCell.A), EPS);
        assertEquals(59.8, Field.cellX(Alliance.RED, HiveCell.B), EPS);
        assertEquals(86.0, Field.cellY(Alliance.RED, HiveCell.B), EPS);
    }

    @Test
    public void redIsBlueRotated180() {
        Pose red = Field.rotate180(new Pose(10, 20, 0));
        assertEquals(134.0, red.x(), EPS);
        assertEquals(124.0, red.y(), EPS);
        assertHeading(Math.PI, red.heading());
        Pose back = Field.rotate180(red);
        assertEquals(10.0, back.x(), EPS);
        assertEquals(20.0, back.y(), EPS);
        assertHeading(0.0, back.heading());
    }

    @Test
    public void startPoseUsesCadEdges() {
        Pose blue = Field.startPose(Alliance.BLUE);
        assertEquals(135.94, blue.x(), EPS);
        assertEquals(137.13, blue.y(), EPS);
        assertHeading(-Math.PI / 2, blue.heading());
        Pose red = Field.startPose(Alliance.RED);
        assertEquals(8.06, red.x(), EPS);
        assertEquals(6.87, red.y(), EPS);
        assertHeading(Math.PI / 2, red.heading());
    }

    @Test
    public void cellOnSidePicksTheCellOnTheRobotsHalf() {
        assertEquals(HiveCell.A, Field.cellOnSide(Alliance.BLUE, 100, HiveCell.B));
        assertEquals(HiveCell.B, Field.cellOnSide(Alliance.BLUE, 40, HiveCell.A));
        assertEquals(HiveCell.B, Field.cellOnSide(Alliance.BLUE, 73, HiveCell.B));
        assertEquals(HiveCell.A, Field.cellOnSide(Alliance.BLUE, 71, HiveCell.A));
        assertEquals(HiveCell.B, Field.cellOnSide(Alliance.RED, 100, HiveCell.A));
        assertEquals(HiveCell.A, Field.cellOnSide(Alliance.RED, 40, HiveCell.B));
    }

    @Test
    public void footprintCornersAtHeadingZero() {
        double[] c = new double[8];
        Field.footprintCorners(72, 72, 0, c);
        assertEquals(81.70, c[0], EPS);
        assertEquals(80.06, c[1], EPS);
        assertEquals(81.70, c[2], EPS);
        assertEquals(63.94, c[3], EPS);
        assertEquals(65.13, c[4], EPS);
        assertEquals(63.94, c[5], EPS);
        assertEquals(65.13, c[6], EPS);
        assertEquals(80.06, c[7], EPS);
    }

    @Test
    public void wallAndFrameChecks() {
        assertTrue(Field.insideWalls(72, 120, -Math.PI / 2, 1.0));
        assertFalse(Field.insideWalls(140, 120, -Math.PI / 2, 1.0));
        assertTrue(Field.clearOfFrame(84.2, 116, -Math.PI / 2, 2.0));
        assertFalse(Field.clearOfFrame(84.2, 100, -Math.PI / 2, 2.0));
    }
}
