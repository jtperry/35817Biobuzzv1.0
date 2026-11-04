package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Shot Tuner route planner (decision 29): the legs the robot drives by itself to a spot, each checked with the
 * ShotTunerLane clearance checks. Pure: the OpMode turns the legs into Pedro paths and ends with a hold at the
 * target. Rules, first match wins:
 * 1. blocked if a pose is not finite, the spot is not valid (or its pose is not finite), or the start is past a wall
 *    (ShotTunerLane.pastWall);
 * 2. already there: no legs;
 * 3. along the lane: held at a spot on the target's lane leg, one leg to the target;
 * 4. direct: after a nudge or an interrupted move (not the first move), within LANE_TOLERANCE_IN of the target's
 *    lane leg, one linear leg to the target;
 * 5. via the turn point (facing cell A): the fallback; its failure is the blocked reason.
 */
public final class ShotTunerRoute {

    /** Shorter translations are not driven (with the defaults the 35 in spot is the turn point). */
    public static final double MIN_LEG_IN = 1.0;
    /** Smaller heading changes are not turned; also the tolerance for "already faces cell A". */
    public static final double MIN_TURN_DEG = 2.0;
    /** How close the robot must be to the target's lane leg for a direct leg, inches. */
    public static final double LANE_TOLERANCE_IN = 12.0;

    private static final double MIN_TURN_RAD = Math.toRadians(MIN_TURN_DEG);
    /** Same test as the spots' back leg: the turn point itself is on the back leg. */
    private static final double SIDE_LEG_EPS_IN = 1e-6;

    private static final String ALREADY_THERE = "already there";
    private static final String ALONG_THE_LANE = "along the lane";
    private static final String DIRECT = "direct";
    private static final String VIA_TURN_POINT = "via the turn point";

    /** One straight leg. */
    public static final class Leg {

        public final Pose from;
        public final Pose to;

        /**
         * True: the heading faces cell A all along (Pedro facingPoint(cell)). False: it turns linearly the shortest
         * way from from's heading to to's (Pedro linear(from, to)).
         */
        public final boolean facingCell;

        private Leg(Pose from, Pose to, boolean facingCell) {
            this.from = from;
            this.to = to;
            this.facingCell = facingCell;
        }

        /**
         * Translation below MIN_LEG_IN: a turn in place (the only leg of its plan). No path is driven for it; the
         * final hold at the target covers the small translation and the turn. The hold's translation and heading
         * controllers converge independently, so the turn can be made anywhere between the two positions: an
         * in-place leg is checked along the diagonal and with the whole turn swept at both the start and the target
         * positions.
         */
        public boolean inPlace() {
            return length(from, to) < MIN_LEG_IN;
        }
    }

    /** A route: legs in order from the start pose to the target pose, or blocked with a reason. */
    public static final class Plan {

        /** Unmodifiable; empty when already there or blocked. */
        public final List<Leg> legs;
        public final boolean blocked;
        /** "" unless blocked, else short Driver Station text such as "turn point blocked". */
        public final String reason;
        /** "already there", "along the lane", "direct", "via the turn point" or "blocked". */
        public final String summary;
        /** Summed translation of the legs, inches. */
        public final double lengthIn;

        private Plan(List<Leg> legs, boolean blocked, String reason, String summary) {
            this.legs = Collections.unmodifiableList(legs);
            this.blocked = blocked;
            this.reason = reason;
            this.summary = summary;
            double length = 0;
            for (Leg leg : legs) {
                length += length(leg.from, leg.to);
            }
            this.lengthIn = length;
        }
    }

    private ShotTunerRoute() {
    }

    /**
     * Plans the move to target. start = the robot pose now; held = the spot the robot is holding at after arriving
     * there (null after a stick cancel, an interrupted or refused move, and before the first move); cell, turn = the
     * frozen lane's ShotTunerLane.cellA and turnPoint (the ones the spots were solved from); firstMove = no automatic
     * move has run yet in this OpMode. start is the wall-margin reference for every leg's check.
     */
    public static Plan plan(Pose start, ShotTunerLane.Spot held, ShotTunerLane.Spot target, Pose cell, Pose turn,
                            boolean firstMove) {
        // Rule 1: not plannable.
        if (!finite(start) || !finite(cell) || !finite(turn)) {
            return blocked("pose not finite");
        }
        if (target == null || !target.valid || !Double.isFinite(target.x) || !Double.isFinite(target.y)
                || !Double.isFinite(target.heading)) {
            return blocked("spot not valid");
        }
        if (ShotTunerLane.pastWall(start)) {
            return blocked("pose past a wall");
        }
        Pose goal = ShotTunerLane.pose(target);
        // Rule 2: already there.
        if (length(start, goal) < MIN_LEG_IN && Math.abs(MathUtil.wrapRadians(goal.heading() - start.heading()))
                < MIN_TURN_RAD) {
            return new Plan(new ArrayList<Leg>(), false, "", ALREADY_THERE);
        }
        // Rule 3: along the lane (failed checks fall through to rule 5).
        if (held != null && onSideLeg(held, cell, turn) == onSideLeg(target, cell, turn)) {
            Leg leg = straight(start, goal, cell);
            if (clear(leg, cell, start)) {
                return route(ALONG_THE_LANE, leg);
            }
        }
        // Rule 4: direct, back to the lane after a nudge or an interrupted move.
        if (!firstMove && held == null && nearLane(start, target, cell, turn)) {
            Leg leg = new Leg(start, goal, false);
            if (clear(leg, cell, start)) {
                return route(DIRECT, leg);
            }
        }
        return viaTurnPoint(start, goal, cell, turn);
    }

    /** Rule 5: linear to the turn point facing cell A, then facing cell A to the target; short legs merge. */
    private static Plan viaTurnPoint(Pose start, Pose goal, Pose cell, Pose turn) {
        Pose turnFacing = new Pose(turn.x(), turn.y(), Math.atan2(cell.y() - turn.y(), cell.x() - turn.x()));
        if (!ShotTunerLane.poseClear(turnFacing)) {
            return blocked("turn point blocked");
        }
        boolean skipToTurn = length(start, turn) < MIN_LEG_IN;
        boolean skipFromTurn = length(turn, goal) < MIN_LEG_IN;
        if (!skipToTurn && !skipFromTurn) {
            Leg toTurn = new Leg(start, turnFacing, false);
            if (!clear(toTurn, cell, start)) {
                return blocked("leg to the turn point hits the hive or a wall");
            }
            Leg fromTurn = new Leg(turnFacing, goal, true);
            if (!clear(fromTurn, cell, start)) {
                return blocked("leg from the turn point hits the hive or a wall");
            }
            return route(VIA_TURN_POINT, toTurn, fromTurn);
        }
        // One leg start -> target. Leg 2 skipped: leg 1 (linear) ends at the target. Leg 1 skipped: leg 2 starts at
        // the start pose, facing cell A only if the robot already faces it.
        Leg merged = skipToTurn ? straight(start, goal, cell) : new Leg(start, goal, false);
        if (merged.inPlace()) {
            String block = linearBlock(start, goal, start);
            if (!block.isEmpty()) {
                return blocked(ShotTunerLane.HIVE_FRAME.equals(block) ? "turn in place hits the hive"
                        : "turn in place hits a wall");
            }
        } else if (!clear(merged, cell, start)) {
            return blocked(skipToTurn ? "leg from the turn point hits the hive or a wall"
                    : "leg to the turn point hits the hive or a wall");
        }
        return route(VIA_TURN_POINT, merged);
    }

    /** One leg start -> goal: in place below MIN_LEG_IN, facing cell A if the start already faces it, else linear. */
    private static Leg straight(Pose start, Pose goal, Pose cell) {
        boolean facing = length(start, goal) >= MIN_LEG_IN && Math.abs(MathUtil.wrapRadians(
                Math.atan2(cell.y() - start.y(), cell.x() - start.x()) - start.heading())) < MIN_TURN_RAD;
        return new Leg(start, goal, facing);
    }

    /** The leg's own check; start is the move's wall-margin reference. In-place legs are linear (swept by angle). */
    private static boolean clear(Leg leg, Pose cell, Pose start) {
        return leg.facingCell ? ShotTunerLane.facingLegClear(leg.from, leg.to, cell, start)
                : linearBlock(leg.from, leg.to, start).isEmpty();
    }

    /**
     * ShotTunerLane.linearLegBlock, plus, for a turn of MIN_TURN_DEG or more, the whole turn swept at to's position:
     * Pedro's heading follows the path's closest point, so on a short leg with a big turn the robot stops
     * translating first and makes the rest of the turn at the end. For an in-place leg (translation below MIN_LEG_IN,
     * left to Pedro's hold) also the whole turn swept at from's position: the hold can finish the turn before the
     * translation. "" when clear, else the first cause.
     */
    private static String linearBlock(Pose from, Pose to, Pose start) {
        String block = ShotTunerLane.linearLegBlock(from, to, start);
        if (block.isEmpty() && Math.abs(MathUtil.wrapRadians(to.heading() - from.heading())) >= MIN_TURN_RAD) {
            block = ShotTunerLane.linearLegBlock(new Pose(to.x(), to.y(), from.heading()), to, start);
        }
        if (block.isEmpty() && length(from, to) < MIN_LEG_IN) {
            block = ShotTunerLane.linearLegBlock(from, new Pose(from.x(), from.y(), to.heading()), start);
        }
        return block;
    }

    /** Farther from cell A than the turn point (the turn point itself is on the back leg). */
    private static boolean onSideLeg(ShotTunerLane.Spot spot, Pose cell, Pose turn) {
        return spot.nominalIn > length(cell, turn) + SIDE_LEG_EPS_IN;
    }

    /**
     * Within LANE_TOLERANCE_IN of the target's lane leg. Back leg: the segment cell A -> turn point. Side leg: the
     * half-line from the turn point through the target (frozen poses only, so a robot beyond the target counts).
     */
    private static boolean nearLane(Pose start, ShotTunerLane.Spot target, Pose cell, Pose turn) {
        double distance = onSideLeg(target, cell, turn)
                ? distanceToLine(start, turn.x(), turn.y(), target.x, target.y, false)
                : distanceToLine(start, cell.x(), cell.y(), turn.x(), turn.y(), true);
        return distance <= LANE_TOLERANCE_IN;
    }

    /** Distance from p to the segment a-b (segment true) or the half-line from a through b (segment false). */
    private static double distanceToLine(Pose p, double ax, double ay, double bx, double by, boolean segment) {
        double dx = bx - ax;
        double dy = by - ay;
        double lengthSq = dx * dx + dy * dy;
        double t = lengthSq > 0 ? ((p.x() - ax) * dx + (p.y() - ay) * dy) / lengthSq : 0;
        t = segment ? MathUtil.clamp(t, 0, 1) : Math.max(0, t);
        return Math.hypot(p.x() - (ax + t * dx), p.y() - (ay + t * dy));
    }

    private static Plan route(String summary, Leg... legs) {
        List<Leg> list = new ArrayList<>();
        Collections.addAll(list, legs);
        return new Plan(list, false, "", summary);
    }

    private static Plan blocked(String reason) {
        return new Plan(new ArrayList<Leg>(), true, reason, "blocked");
    }

    private static boolean finite(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static double length(Pose from, Pose to) {
        return Math.hypot(to.x() - from.x(), to.y() - from.y());
    }
}
