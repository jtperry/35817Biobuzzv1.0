package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * Shot Tuner spots (spec 7.3): one every STEP_IN from START_IN to END_IN inches from cell A, on a two-leg lane.
 * Back leg: from cell A along BACK_DEG up to TURN_IN. Side leg: from the turn point along SIDE_DEG.
 * Each spot is the lane point exactly that far from cell A, facing it. Angles are blue; red = blue rotated 180.
 */
@Configurable
public final class ShotTunerLane {

    public static double START_IN = 30;
    public static double END_IN = 80;
    public static double STEP_IN = 5;

    /** Back leg direction from cell A, degrees, blue (90 = away from the hive). */
    public static double BACK_DEG = 90;

    /** Back leg length, inches. */
    public static double TURN_IN = 35;

    /** Side leg direction from the turn point, degrees, blue (180 = west, 0 = east). */
    public static double SIDE_DEG = 180;

    /**
     * A valid spot's footprint stays this far inside the walls and clear of the hive frame, inches. A negative or
     * NaN value fails closed: every spot is invalid ("wall") and every pose and leg check fails.
     */
    public static double WALL_MARGIN_IN = 1;
    public static double FRAME_MARGIN_IN = 2;

    /**
     * A leg's start pose may be this far past a wall (pose error, a restart slot measured a hair off) and still
     * plan; farther past and every leg check fails (see pastWall). Fixed, not a tunable.
     */
    public static final double START_PAST_WALL_MAX_IN = 0.5;

    /** Cap on the spot count, so a tiny STEP_IN or a huge END_IN typed into Panels cannot hang the OpMode. */
    private static final int MAX_SPOTS = 1000;

    /** Leg samples: at most this far apart (inches) and this much turn apart (radians); at most MAX_SAMPLES. */
    private static final double SAMPLE_IN = 0.5;
    private static final double SAMPLE_RAD = Math.toRadians(2);
    private static final int MAX_SAMPLES = 2000;
    /** Rounding slack for a start flush on a wall (restart slot 0 is in the corner). */
    private static final double WALL_SLACK_IN = 1e-6;

    /** Spot and leg block reasons. */
    static final String WALL = "wall";
    static final String HIVE_FRAME = "hive frame";

    public static final class Spot {

        /** Distance from cell A this spot was solved for, inches, rounded to 0.1 (the CSV key). */
        public final double nominalIn;
        public final double x;
        public final double y;

        /** Radians, facing cell A, in [-PI, PI). */
        public final double heading;
        public final boolean valid;

        /** "" when valid, else "wall", "hive frame" or "off lane". */
        public final String reason;

        public Spot(double nominalIn, double x, double y, double heading, boolean valid, String reason) {
            this.nominalIn = nominalIn;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.valid = valid;
            this.reason = reason;
        }
    }

    private ShotTunerLane() {
    }

    /**
     * Spots in lane order, START_IN to END_IN inclusive. A step that is not positive (0, negative, NaN) gives the
     * START_IN spot alone; END_IN below START_IN gives none; at most MAX_SPOTS. Validity uses the full margins.
     */
    public static List<Spot> spots(Alliance alliance) {
        double steps = STEP_IN > 0 ? Math.floor((END_IN - START_IN) / STEP_IN + 1e-6) + 1 : 1;
        int count = (int) Math.min(MAX_SPOTS, steps);
        List<Spot> spots = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            // The first nominal is START_IN itself: START_IN + 0 * STEP_IN is NaN for a non-finite step.
            double nominal = i == 0 ? START_IN : START_IN + i * STEP_IN;
            spots.add(spot(alliance, Math.round(nominal * 10) / 10.0));
        }
        return spots;
    }

    public static Pose cellA(Alliance alliance) {
        return new Pose(Field.cellX(alliance, HiveCell.A), Field.cellY(alliance, HiveCell.A), 0);
    }

    /** End of the back leg, heading 0. */
    public static Pose turnPoint(Alliance alliance) {
        double back = Math.toRadians(BACK_DEG);
        Pose turn = Field.forAlliance(alliance, new Pose(
                Field.cellX(Alliance.BLUE, HiveCell.A) + TURN_IN * Math.cos(back),
                Field.cellY(Alliance.BLUE, HiveCell.A) + TURN_IN * Math.sin(back), 0));
        return new Pose(turn.x(), turn.y(), 0);
    }

    public static Pose pose(Spot spot) {
        return new Pose(spot.x, spot.y, spot.heading);
    }

    /**
     * True if the footprint stays inside the walls and clear of the hive frame along a straight leg whose heading
     * turns linearly the shortest way from from's heading to to's (Pedro linear()). start = where the move began, the
     * wall-margin reference: per wall, the margin is WALL_MARGIN_IN, or start's own clearance to that wall when it is
     * smaller (closer than WALL_MARGIN_IN, or a hair past), so a leg never gets closer to that wall than the start
     * and never farther past it; the other walls keep the full WALL_MARGIN_IN. False if a pose is not finite, the
     * margins are not valid (negative or NaN), or start is past a wall by more than START_PAST_WALL_MAX_IN.
     */
    public static boolean linearLegClear(Pose from, Pose to, Pose start) {
        return linearLegBlock(from, to, start).isEmpty();
    }

    /**
     * True if the footprint at this pose keeps the full spot margins (no start allowance): for the turn point. False
     * if the pose is not finite or the margins are not valid (negative or NaN).
     */
    public static boolean poseClear(Pose pose) {
        return marginsValid() && finite(pose)
                && Field.insideWalls(pose.x(), pose.y(), pose.heading(), WALL_MARGIN_IN)
                && Field.clearOfFrame(pose.x(), pose.y(), pose.heading(), FRAME_MARGIN_IN);
    }

    /** As linearLegClear (same margins), but the heading faces the point all along the leg (Pedro facingPoint()). */
    public static boolean facingLegClear(Pose from, Pose to, Pose facing, Pose start) {
        return legBlock(from, to, facing, 0, start).isEmpty();
    }

    /**
     * True if the pose is finite and a footprint corner lies more than START_PAST_WALL_MAX_IN past a wall. Every leg
     * check fails from such a start.
     */
    public static boolean pastWall(Pose pose) {
        if (!finite(pose)) {
            return false;
        }
        for (double clearance : wallClearances(pose.x(), pose.y(), pose.heading())) {
            if (clearance < -START_PAST_WALL_MAX_IN) {
                return true;
            }
        }
        return false;
    }

    /**
     * linearLegClear with the cause: "" when clear, else WALL or HIVE_FRAME (the first sample that fails). A
     * non-finite pose, invalid margins or a start past a wall give WALL. For ShotTunerRoute's in-place reasons.
     */
    static String linearLegBlock(Pose from, Pose to, Pose start) {
        return legBlock(from, to, null, MathUtil.wrapRadians(to.heading() - from.heading()), start);
    }

    private static String legBlock(Pose from, Pose to, Pose facing, double turn, Pose start) {
        if (!marginsValid() || !finite(from) || !finite(to) || !finite(start) || (facing != null && !finite(facing))
                || pastWall(start)) {
            return WALL;
        }
        // Per wall: the full margin, or start's own clearance to that wall when it is smaller (no floor).
        double[] wallMargins = wallClearances(start.x(), start.y(), start.heading());
        for (int wall = 0; wall < wallMargins.length; wall++) {
            wallMargins[wall] = Math.min(WALL_MARGIN_IN, wallMargins[wall]) - WALL_SLACK_IN;
        }
        double length = Math.hypot(to.x() - from.x(), to.y() - from.y());
        double steps = Math.ceil(Math.max(length / SAMPLE_IN, Math.abs(turn) / SAMPLE_RAD));
        int count = (int) Math.max(1, Math.min(MAX_SAMPLES, steps));
        for (int i = 0; i <= count; i++) {
            double t = (double) i / count;
            double x = from.x() + t * (to.x() - from.x());
            double y = from.y() + t * (to.y() - from.y());
            double heading = facing == null ? from.heading() + t * turn : Math.atan2(facing.y() - y, facing.x() - x);
            double[] clearances = wallClearances(x, y, heading);
            for (int wall = 0; wall < clearances.length; wall++) {
                if (clearances[wall] < wallMargins[wall]) {
                    return WALL;
                }
            }
            if (!Field.clearOfFrame(x, y, heading, FRAME_MARGIN_IN)) {
                return HIVE_FRAME;
            }
        }
        return "";
    }

    /** Footprint clearance to each wall: west, east, south, north, inches (negative = past it). */
    private static double[] wallClearances(double x, double y, double heading) {
        double[] corners = new double[8];
        Field.footprintCorners(x, y, heading, corners);
        double[] clearances = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        for (int i = 0; i < corners.length; i += 2) {
            clearances[0] = Math.min(clearances[0], corners[i]);
            clearances[1] = Math.min(clearances[1], Field.SIZE_IN - corners[i]);
            clearances[2] = Math.min(clearances[2], corners[i + 1]);
            clearances[3] = Math.min(clearances[3], Field.SIZE_IN - corners[i + 1]);
        }
        return clearances;
    }

    /** False for a negative or NaN margin: the checks then fail closed. */
    private static boolean marginsValid() {
        return WALL_MARGIN_IN >= 0 && FRAME_MARGIN_IN >= 0;
    }

    private static boolean finite(Pose pose) {
        return Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    private static Spot spot(Alliance alliance, double nominalIn) {
        double back = Math.toRadians(BACK_DEG);
        double turnX = TURN_IN * Math.cos(back);
        double turnY = TURN_IN * Math.sin(back);
        double dx;
        double dy;
        boolean onLane = true;
        if (nominalIn <= TURN_IN) {
            dx = nominalIn * Math.cos(back);
            dy = nominalIn * Math.sin(back);
        } else {
            // |turn + t * side| = nominal (relative to cell A): t^2 + 2bt + TURN^2 - nominal^2 = 0, larger root.
            double side = Math.toRadians(SIDE_DEG);
            double ux = Math.cos(side);
            double uy = Math.sin(side);
            double b = turnX * ux + turnY * uy;
            double disc = b * b - TURN_IN * TURN_IN + nominalIn * nominalIn;
            double t = disc >= 0 ? -b + Math.sqrt(disc) : -1;
            onLane = t >= 0;
            dx = onLane ? turnX + t * ux : turnX;
            dy = onLane ? turnY + t * uy : turnY;
        }
        Pose pose = Field.forAlliance(alliance, new Pose(
                Field.cellX(Alliance.BLUE, HiveCell.A) + dx,
                Field.cellY(Alliance.BLUE, HiveCell.A) + dy, Math.atan2(-dy, -dx)));
        double heading = MathUtil.wrapRadians(pose.heading());
        String reason;
        if (!onLane) {
            reason = "off lane";
        } else if (!marginsValid() || !Field.insideWalls(pose.x(), pose.y(), heading, WALL_MARGIN_IN)) {
            reason = WALL;
        } else if (!Field.clearOfFrame(pose.x(), pose.y(), heading, FRAME_MARGIN_IN)) {
            reason = HIVE_FRAME;
        } else {
            reason = "";
        }
        return new Spot(nominalIn, pose.x(), pose.y(), heading, reason.isEmpty(), reason);
    }
}
