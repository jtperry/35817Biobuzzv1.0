package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

/** Draws on the Panels field in Pedro coordinates. Call from a telemetry frame (10 Hz), not every loop. */
public final class FieldDrawing {

    public static final double ROBOT_RADIUS_IN = 9;

    private static final FieldManager FIELD = PanelsField.INSTANCE.getField();
    private static final Style ROBOT_STYLE = new Style("", "#3F51B5", 0.75);
    private static final double LINE_WIDTH = 0.75;

    private FieldDrawing() {
    }

    /**
     * Robot circle + line to its front, then send. A null or non-finite pose still sends the shapes queued by line() and
     * circle(), without the robot, so they never pile up into a later frame (or the next OpMode's).
     */
    public static void robot(Pose pose) {
        // Pedro coordinates. Set every frame: update() resets them.
        FIELD.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) {
            FIELD.update();
            return;
        }
        FIELD.setStyle(ROBOT_STYLE);
        FIELD.moveCursor(pose.x(), pose.y());
        FIELD.circle(ROBOT_RADIUS_IN);
        FIELD.moveCursor(pose.x(), pose.y());
        FIELD.line(pose.x() + ROBOT_RADIUS_IN * Math.cos(pose.heading()),
                pose.y() + ROBOT_RADIUS_IN * Math.sin(pose.heading()));
        FIELD.update();
    }

    /** Line in Pedro coordinates. Sent by the next robot() call, so draw it before robot() in the same frame. */
    public static void line(double x1, double y1, double x2, double y2, String color) {
        // A NaN lane tunable typed into Panels must not reach the Panels JSON.
        if (!Double.isFinite(x1) || !Double.isFinite(y1) || !Double.isFinite(x2) || !Double.isFinite(y2)) {
            return;
        }
        FIELD.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        FIELD.setStyle("", color, LINE_WIDTH);
        FIELD.moveCursor(x1, y1);
        FIELD.line(x2, y2);
    }

    /** Circle outline in Pedro coordinates (a small one marks a point). Sent by the next robot() call. */
    public static void circle(double x, double y, double radius, String color) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(radius)) {
            return;
        }
        FIELD.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        FIELD.setStyle("", color, LINE_WIDTH);
        FIELD.moveCursor(x, y);
        FIELD.circle(radius);
    }
}
