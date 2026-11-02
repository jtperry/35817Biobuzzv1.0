package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Field geometry in Pedro coordinates: inches, 0-144 on both axes, origin at the red wall / audience corner,
 * +x toward the blue wall, +y away from the audience, heading 0 = +x, counterclockwise positive.
 * Red is always blue rotated 180 degrees about (72, 72). Poses are the robot center (drivetrain center).
 */
@Configurable
public final class Field {

    public static final double SIZE_IN = 144;

    /** Blue cell A (the one that starts up). The only measured cell; the other three are derived. */
    public static double BLUE_CELL_A_X = 84.2;
    public static double BLUE_CELL_A_Y = 86.0;

    /** Teleop cell pick: the robot must cross the hive midline by this much to switch cells. */
    public static double SIDE_SWITCH_DEADBAND_IN = 2.0;

    /** Robot center to each edge, inches (CAD, spec 4.6). Front = intake bumper. */
    public static final double ROBOT_FRONT_IN = 9.70;
    public static final double ROBOT_BACK_IN = 6.87;
    public static final double ROBOT_SIDE_IN = 8.06;

    /** Hive frame footprint, approximate (manual 9.6.1). Confirm on the field. */
    public static double FRAME_MIN_X = 52.5;
    public static double FRAME_MAX_X = 91.5;
    public static double FRAME_MIN_Y = 47.3;
    public static double FRAME_MAX_Y = 96.7;

    public static final int RESTART_SLOTS = 6;
    public static final String[] RESTART_SLOT_NAMES = {
            "right", "right-flower", "left-flower", "top-flower", "bottom-flower", "left"
    };

    /**
     * Blue restart poses. Slot 0 = corner from the CAD edges: left side on the blue wall, back on the far wall.
     * Slots 1-5 are v1 measurements; re-measure them with the new robot.
     */
    private static final Pose[] BLUE_RESTART_POSES = {
            new Pose(SIZE_IN - ROBOT_SIDE_IN, SIZE_IN - ROBOT_BACK_IN, Math.toRadians(-90)),
            new Pose(133, 105.6, Math.toRadians(180)),
            new Pose(133, 84, Math.toRadians(180)),
            new Pose(84, 8, Math.toRadians(90)),
            new Pose(105.4, 8, Math.toRadians(90)),
            new Pose(133, 8, Math.toRadians(0)),
    };

    private Field() {
    }

    /** Both cells of a hive share x. Red = 144 - blue. */
    public static double cellX(Alliance alliance, HiveCell cell) {
        return alliance == Alliance.RED ? SIZE_IN - BLUE_CELL_A_X : BLUE_CELL_A_X;
    }

    /** Cell B = cell A mirrored across y = 72. Red = 144 - blue. */
    public static double cellY(Alliance alliance, HiveCell cell) {
        double blueY = cell == HiveCell.A ? BLUE_CELL_A_Y : SIZE_IN - BLUE_CELL_A_Y;
        return alliance == Alliance.RED ? SIZE_IN - blueY : blueY;
    }

    /** Rotate 180 degrees about the field center. */
    public static Pose rotate180(Pose pose) {
        return new Pose(SIZE_IN - pose.x(), SIZE_IN - pose.y(), pose.heading() + Math.PI);
    }

    /** Blue pose as given, red rotated 180 degrees. */
    public static Pose forAlliance(Alliance alliance, Pose blue) {
        return alliance == Alliance.RED ? rotate180(blue) : blue;
    }

    public static Pose restartPose(Alliance alliance, int slot) {
        return forAlliance(alliance, BLUE_RESTART_POSES[Math.floorMod(slot, RESTART_SLOTS)]);
    }

    public static Pose startPose(Alliance alliance) {
        return restartPose(alliance, 0);
    }

    /** The cell on the robot's side of the hive midline (y = 72). Inside the deadband: keep current. */
    public static HiveCell cellOnSide(Alliance alliance, double robotY, HiveCell current) {
        double mid = SIZE_IN / 2;
        HiveCell upper = cellY(alliance, HiveCell.A) > mid ? HiveCell.A : HiveCell.B;
        if (robotY > mid + SIDE_SWITCH_DEADBAND_IN) {
            return upper;
        }
        if (robotY < mid - SIDE_SWITCH_DEADBAND_IN) {
            return upper.other();
        }
        return current;
    }

    /** Footprint corners at a pose into out: front-left, front-right, back-right, back-left as x, y pairs. */
    public static void footprintCorners(double x, double y, double heading, double[] out) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        corner(out, 0, x, y, cos, sin, ROBOT_FRONT_IN, ROBOT_SIDE_IN);
        corner(out, 2, x, y, cos, sin, ROBOT_FRONT_IN, -ROBOT_SIDE_IN);
        corner(out, 4, x, y, cos, sin, -ROBOT_BACK_IN, -ROBOT_SIDE_IN);
        corner(out, 6, x, y, cos, sin, -ROBOT_BACK_IN, ROBOT_SIDE_IN);
    }

    private static void corner(double[] out, int index, double x, double y, double cos, double sin,
                               double forward, double left) {
        out[index] = x + forward * cos - left * sin;
        out[index + 1] = y + forward * sin + left * cos;
    }

    /** True if every footprint corner is at least margin inside the walls. For tools, not the loop. */
    public static boolean insideWalls(double x, double y, double heading, double margin) {
        double[] corners = new double[8];
        footprintCorners(x, y, heading, corners);
        for (double value : corners) {
            if (value < margin || value > SIZE_IN - margin) {
                return false;
            }
        }
        return true;
    }

    /** True if the footprint's bounding box stays margin clear of the hive frame. For tools, not the loop. */
    public static boolean clearOfFrame(double x, double y, double heading, double margin) {
        double[] corners = new double[8];
        footprintCorners(x, y, heading, corners);
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (int i = 0; i < 8; i += 2) {
            minX = Math.min(minX, corners[i]);
            maxX = Math.max(maxX, corners[i]);
            minY = Math.min(minY, corners[i + 1]);
            maxY = Math.max(maxY, corners[i + 1]);
        }
        return maxX < FRAME_MIN_X - margin || minX > FRAME_MAX_X + margin
                || maxY < FRAME_MIN_Y - margin || minY > FRAME_MAX_Y + margin;
    }
}
