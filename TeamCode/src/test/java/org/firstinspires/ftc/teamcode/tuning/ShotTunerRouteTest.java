package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class ShotTunerRouteTest {

    private static final double EPS = 1e-9;
    private static final String VIA = "via the turn point";
    private static final String TO_TURN_BLOCKED = "leg to the turn point hits the hive or a wall";

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

    // ---- helpers ----

    private static ShotTunerLane.Spot spot(Alliance alliance, double nominalIn) {
        for (ShotTunerLane.Spot spot : ShotTunerLane.spots(alliance)) {
            if (spot.nominalIn == nominalIn) {
                return spot;
            }
        }
        fail("no spot " + nominalIn);
        return null;
    }

    private static Pose target(Alliance alliance, double nominalIn) {
        return ShotTunerLane.pose(spot(alliance, nominalIn));
    }

    /** A blue pose (degrees), rotated 180 for red. */
    private static Pose pose(Alliance alliance, double x, double y, double deg) {
        return Field.forAlliance(alliance, new Pose(x, y, Math.toRadians(deg)));
    }

    private static Pose turnFacing(Alliance alliance) {
        Pose turn = ShotTunerLane.turnPoint(alliance);
        Pose cell = ShotTunerLane.cellA(alliance);
        return new Pose(turn.x(), turn.y(), Math.atan2(cell.y() - turn.y(), cell.x() - turn.x()));
    }

    private static double length(Pose from, Pose to) {
        return Math.hypot(to.x() - from.x(), to.y() - from.y());
    }

    private static void assertSamePose(String message, Pose expected, Pose actual) {
        assertEquals(message + " x", expected.x(), actual.x(), EPS);
        assertEquals(message + " y", expected.y(), actual.y(), EPS);
        assertEquals(message + " heading", 0.0, MathUtil.wrapRadians(expected.heading() - actual.heading()), EPS);
    }

    private static boolean legClear(ShotTunerRoute.Leg leg, Pose cell, Pose start) {
        return leg.facingCell ? ShotTunerLane.facingLegClear(leg.from, leg.to, cell, start)
                : ShotTunerLane.linearLegClear(leg.from, leg.to, start);
    }

    /**
     * Plans and checks what every plan in this file must hold. Not blocked: the legs connect from the start pose to
     * the target pose, every leg passes its own ShotTunerLane check (so no in-place leg has a failing sweep), an
     * in-place leg is the only leg and linear, and lengthIn is the summed translation. Blocked: no legs, a reason.
     */
    private static ShotTunerRoute.Plan plan(Alliance alliance, Pose start, ShotTunerLane.Spot held,
                                            ShotTunerLane.Spot target, boolean firstMove) {
        Pose cell = ShotTunerLane.cellA(alliance);
        ShotTunerRoute.Plan plan = ShotTunerRoute.plan(start, held, target, cell, ShotTunerLane.turnPoint(alliance),
                firstMove);
        if (plan.blocked) {
            assertTrue(plan.legs.isEmpty());
            assertFalse(plan.reason.isEmpty());
            assertEquals("blocked", plan.summary);
            assertEquals(0.0, plan.lengthIn, 0.0);
            return plan;
        }
        assertEquals("", plan.reason);
        Pose at = start;
        double length = 0;
        for (int i = 0; i < plan.legs.size(); i++) {
            ShotTunerRoute.Leg leg = plan.legs.get(i);
            assertSamePose("leg " + i + " from", at, leg.from);
            if (leg.inPlace()) {
                assertEquals(1, plan.legs.size());
                assertFalse(leg.facingCell);
            }
            assertTrue(alliance + " " + plan.summary + " leg " + i + " clear", legClear(leg, cell, start));
            length += length(leg.from, leg.to);
            at = leg.to;
        }
        if (!plan.legs.isEmpty()) {
            assertSamePose("last leg to", ShotTunerLane.pose(target), at);
        }
        assertEquals(length, plan.lengthIn, EPS);
        return plan;
    }

    /** Leg 1 linear from start to the turn point facing cell A, leg 2 facing cell A to the target. */
    private static void assertViaTurnPoint(Alliance alliance, ShotTunerRoute.Plan plan, Pose start, Pose target) {
        assertFalse(alliance + " " + plan.reason, plan.blocked);
        assertEquals(VIA, plan.summary);
        assertEquals(2, plan.legs.size());
        ShotTunerRoute.Leg toTurn = plan.legs.get(0);
        ShotTunerRoute.Leg fromTurn = plan.legs.get(1);
        assertSamePose("leg 1 from", start, toTurn.from);
        assertSamePose("leg 1 to", turnFacing(alliance), toTurn.to);
        assertFalse(toTurn.facingCell);
        assertSamePose("leg 2 from", turnFacing(alliance), fromTurn.from);
        assertSamePose("leg 2 to", target, fromTurn.to);
        assertTrue(fromTurn.facingCell);
        assertEquals(length(start, turnFacing(alliance)) + length(turnFacing(alliance), target), plan.lengthIn, EPS);
    }

    /** One leg start -> target. */
    private static void assertOneLeg(ShotTunerRoute.Plan plan, String summary, boolean facingCell, Pose start,
                                     Pose target) {
        assertFalse(plan.reason, plan.blocked);
        assertEquals(summary, plan.summary);
        assertEquals(1, plan.legs.size());
        ShotTunerRoute.Leg leg = plan.legs.get(0);
        assertSamePose("from", start, leg.from);
        assertSamePose("to", target, leg.to);
        assertEquals(facingCell, leg.facingCell);
    }

    // ---- first move ----

    @Test
    public void theFirstMoveFromSlot0GoesViaTheTurnPoint() {
        for (Alliance alliance : Alliance.values()) {
            Pose start = Field.restartPose(alliance, 0);
            ShotTunerRoute.Plan plan = plan(alliance, start, null, spot(alliance, 30), true);
            assertViaTurnPoint(alliance, plan, start, target(alliance, 30));
            assertEquals(Math.hypot(135.94 - 84.2, 137.13 - 121) + 5, plan.lengthIn, 1e-6);
            try {
                plan.legs.clear();
                fail("legs must be unmodifiable");
            } catch (UnsupportedOperationException expected) {
                // the plan is read-only
            }
        }
    }

    @Test
    public void slot0RotatedAHairPastAWallStillPlans() {
        for (Alliance alliance : Alliance.values()) {
            Pose slot = Field.restartPose(alliance, 0);
            for (double deg : new double[] {0.1, -0.1}) {
                Pose start = new Pose(slot.x(), slot.y(), slot.heading() + Math.toRadians(deg));
                ShotTunerRoute.Plan plan = plan(alliance, start, null, spot(alliance, 30), true);
                assertViaTurnPoint(alliance, plan, start, target(alliance, 30));
            }
        }
    }

    /**
     * With the default lane: slots 0 and 1 plan via the turn point. Slot 2 is blocked by the hive frame: its
     * footprint's bounding box (the frame check) overlaps the frame margin mid-turn. Slots 3 and 4 are blocked by
     * the hive (the straight line to the turn point crosses it). Slot 5 is blocked by the hive too: its footprint
     * starts 0.06 in past the south wall, inside START_PAST_WALL_MAX_IN, so that wall does not block it.
     */
    @Test
    public void theFirstMoveFromEveryRestartSlot() {
        for (Alliance alliance : Alliance.values()) {
            Pose turnFacing = turnFacing(alliance);
            for (int slot = 0; slot < Field.RESTART_SLOTS; slot++) {
                Pose start = Field.restartPose(alliance, slot);
                ShotTunerRoute.Plan plan = plan(alliance, start, null, spot(alliance, 30), true);
                String name = alliance + " slot " + slot;
                if (slot <= 1) {
                    assertViaTurnPoint(alliance, plan, start, target(alliance, 30));
                } else {
                    assertTrue(name, plan.blocked);
                    assertEquals(name, TO_TURN_BLOCKED, plan.reason);
                    assertEquals(name, "hive frame", ShotTunerLane.linearLegBlock(start, turnFacing, start));
                }
            }
            assertFalse(ShotTunerLane.pastWall(Field.restartPose(alliance, 5)));
        }
    }

    // ---- along the lane ----

    @Test
    public void heldOnTheSameLegDrivesOneFacingLeg() {
        for (Alliance alliance : Alliance.values()) {
            Pose at40 = target(alliance, 40);
            Pose at45 = target(alliance, 45);
            assertOneLeg(plan(alliance, at40, spot(alliance, 40), spot(alliance, 45), false),
                    "along the lane", true, at40, at45);
            assertOneLeg(plan(alliance, at45, spot(alliance, 45), spot(alliance, 40), false),
                    "along the lane", true, at45, at40);
        }
    }

    @Test
    public void heldOnTheOtherLegGoesViaTheTurnPoint() {
        for (Alliance alliance : Alliance.values()) {
            Pose at30 = target(alliance, 30);
            ShotTunerRoute.Plan plan = plan(alliance, at30, spot(alliance, 30), spot(alliance, 40), false);
            assertViaTurnPoint(alliance, plan, at30, target(alliance, 40));
        }
    }

    @Test
    public void sideToBackSkipsTheLegFromTheTurnPoint() {
        for (Alliance alliance : Alliance.values()) {
            Pose at40 = target(alliance, 40);
            Pose at35 = target(alliance, 35);
            ShotTunerRoute.Plan plan = plan(alliance, at40, spot(alliance, 40), spot(alliance, 35), false);
            assertOneLeg(plan, VIA, false, at40, at35);
            assertTrue(ShotTunerLane.linearLegClear(at40, at35, at40));
        }
    }

    // ---- after a nudge (held null, not the first move) ----

    @Test
    public void aNudgeNearTheLaneDrivesStraightBack() {
        ShotTunerLane.Spot blue = spot(Alliance.BLUE, 60);
        for (Alliance alliance : Alliance.values()) {
            // 3 in off the side leg, toward the hive, turned 20 deg.
            Pose start = pose(alliance, blue.x + 2, blue.y - 3, Math.toDegrees(blue.heading) + 20);
            ShotTunerRoute.Plan plan = plan(alliance, start, null, spot(alliance, 60), false);
            assertOneLeg(plan, "direct", false, start, target(alliance, 60));
            assertFalse(plan.legs.get(0).inPlace());
        }
    }

    @Test
    public void aNudgeWhoseDirectLegIsBlockedFallsBackToTheTurnPoint() {
        for (Alliance alliance : Alliance.values()) {
            // 11.2 in off the back leg: the direct leg's turn sweeps the footprint into the frame margin.
            Pose start = pose(alliance, 73, 107, 180);
            Pose target = target(alliance, 30);
            assertFalse(ShotTunerLane.linearLegClear(start, target, start));
            assertViaTurnPoint(alliance, plan(alliance, start, null, spot(alliance, 30), false), start, target);
            // 11 in off the side leg: the direct leg's turn sweeps a corner into the west wall.
            start = pose(alliance, 9, 110, -90);
            target = target(alliance, 80);
            assertFalse(ShotTunerLane.linearLegClear(start, target, start));
            assertViaTurnPoint(alliance, plan(alliance, start, null, spot(alliance, 80), false), start, target);
        }
    }

    @Test
    public void aNudgeBeyondTheTargetOnTheSideLegStillCountsAsNearTheLane() {
        for (Alliance alliance : Alliance.values()) {
            Pose at80 = target(alliance, 80);
            assertOneLeg(plan(alliance, at80, null, spot(alliance, 65), false), "direct", false, at80,
                    target(alliance, 65));
        }
    }

    @Test
    public void farFromTheLaneGoesViaTheTurnPoint() {
        for (Alliance alliance : Alliance.values()) {
            // 30 in west of the back leg, with the 30 in spot's heading.
            Pose start = pose(alliance, 54.2, 116, -90);
            assertViaTurnPoint(alliance, plan(alliance, start, null, spot(alliance, 30), false), start,
                    target(alliance, 30));
        }
    }

    @Test
    public void aSkippedTranslationTurnsInPlaceOrIsAlreadyThere() {
        ShotTunerLane.Spot blue = spot(Alliance.BLUE, 60);
        for (Alliance alliance : Alliance.values()) {
            Pose target = target(alliance, 60);
            Pose start = pose(alliance, blue.x + 0.3, blue.y, Math.toDegrees(blue.heading) + 40);
            ShotTunerRoute.Plan plan = plan(alliance, start, null, spot(alliance, 60), false);
            assertOneLeg(plan, "direct", false, start, target);
            assertTrue(plan.legs.get(0).inPlace());
            assertTrue(ShotTunerLane.linearLegClear(start, target, start));

            start = pose(alliance, blue.x + 0.3, blue.y, Math.toDegrees(blue.heading) + 1);
            plan = plan(alliance, start, null, spot(alliance, 60), false);
            assertFalse(plan.blocked);
            assertTrue(plan.legs.isEmpty());
            assertEquals("already there", plan.summary);
            assertEquals(0.0, plan.lengthIn, 0.0);
        }
    }

    /**
     * Plan finding F3: the 80 in spot turned by hand to -170 deg. The shortest turn back to about -26 deg passes
     * -140 deg, where the front-right corner reaches x = -0.3, so the in-place sweep fails on the west wall and the
     * plan goes via the turn point. (plan() checks that no plan in this file has an in-place leg that fails.)
     */
    @Test
    public void anInPlaceTurnIntoTheWallGoesViaTheTurnPoint() {
        for (Alliance alliance : Alliance.values()) {
            Pose start = pose(alliance, 12.3, 121, -170);
            Pose target = target(alliance, 80);
            assertTrue(length(start, target) < ShotTunerRoute.MIN_LEG_IN);
            assertEquals("wall", ShotTunerLane.linearLegBlock(start, target, start));
            assertViaTurnPoint(alliance, plan(alliance, start, null, spot(alliance, 80), false), start, target);
        }
    }

    // ---- blocked ----

    @Test
    public void aBlockedTurnPointBlocksEveryRouteThroughIt() {
        // The turn point's footprint crosses the north wall while the 30-50 in spots stay valid.
        ShotTunerLane.TURN_IN = 52;
        for (Alliance alliance : Alliance.values()) {
            assertFalse(ShotTunerLane.poseClear(turnFacing(alliance)));
            for (double nominal = 30; nominal <= 50; nominal += 5) {
                assertTrue(alliance + " " + nominal, spot(alliance, nominal).valid);
            }
            for (int slot = 0; slot <= 1; slot++) {
                ShotTunerRoute.Plan plan = plan(alliance, Field.restartPose(alliance, slot), null,
                        spot(alliance, 30), true);
                assertTrue(plan.blocked);
                assertEquals("turn point blocked", plan.reason);
            }
            ShotTunerRoute.Plan far = plan(alliance, pose(alliance, 54.2, 116, -90), null, spot(alliance, 30),
                    false);
            assertTrue(far.blocked);
            assertEquals("turn point blocked", far.reason);
        }
    }

    @Test
    public void unplannableInputsAreBlocked() {
        for (Alliance alliance : Alliance.values()) {
            Pose slot = Field.restartPose(alliance, 0);
            ShotTunerLane.Spot target = spot(alliance, 30);
            assertEquals("pose not finite",
                    plan(alliance, new Pose(Double.NaN, 100, 0), null, target, false).reason);
            assertEquals("pose not finite", ShotTunerRoute.plan(slot, null, target,
                    new Pose(Double.POSITIVE_INFINITY, 86, 0), ShotTunerLane.turnPoint(alliance), true).reason);
            assertEquals("spot not valid", plan(alliance, slot, null, null, true).reason);
            ShotTunerLane.Spot invalid = new ShotTunerLane.Spot(20, target.x, target.y, target.heading, false,
                    "hive frame");
            assertEquals("spot not valid", plan(alliance, slot, null, invalid, true).reason);
            // Slot 0 is flush on the blue (east for blue) wall: 1 in farther is 1 in past it.
            Pose past = Field.forAlliance(alliance, new Pose(Field.SIZE_IN - Field.ROBOT_SIDE_IN + 1,
                    Field.SIZE_IN - Field.ROBOT_BACK_IN, Math.toRadians(-90)));
            assertTrue(ShotTunerLane.pastWall(past));
            ShotTunerRoute.Plan plan = plan(alliance, past, null, target, true);
            assertTrue(plan.blocked);
            assertEquals("pose past a wall", plan.reason);
        }
    }
}
