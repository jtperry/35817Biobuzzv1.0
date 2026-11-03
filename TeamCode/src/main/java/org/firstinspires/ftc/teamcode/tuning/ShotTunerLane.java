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

    /** A valid spot's footprint stays this far inside the walls and clear of the hive frame, inches. */
    public static double WALL_MARGIN_IN = 1;
    public static double FRAME_MARGIN_IN = 2;

    /** Cap on the spot count, so a tiny STEP_IN or a huge END_IN typed into Panels cannot hang the OpMode. */
    private static final int MAX_SPOTS = 1000;

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

    /** Spots in lane order. A step that is not positive gives START_IN alone; at most MAX_SPOTS. */
    public static List<Spot> spots(Alliance alliance) {
        double steps = STEP_IN > 0 ? Math.floor((END_IN - START_IN) / STEP_IN + 1e-6) + 1 : 1;
        int count = (int) Math.min(MAX_SPOTS, steps);
        List<Spot> spots = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            spots.add(spot(alliance, Math.round((START_IN + i * STEP_IN) * 10) / 10.0));
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
        } else if (!Field.insideWalls(pose.x(), pose.y(), heading, WALL_MARGIN_IN)) {
            reason = "wall";
        } else if (!Field.clearOfFrame(pose.x(), pose.y(), heading, FRAME_MARGIN_IN)) {
            reason = "hive frame";
        } else {
            reason = "";
        }
        return new Spot(nominalIn, pose.x(), pose.y(), heading, reason.isEmpty(), reason);
    }
}
