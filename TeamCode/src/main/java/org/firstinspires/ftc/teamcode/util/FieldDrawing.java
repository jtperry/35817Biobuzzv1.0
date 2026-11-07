package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

public final class FieldDrawing {

    public static final double ROBOT_RADIUS_IN = 9;

    private static final FieldManager FIELD = PanelsField.INSTANCE.getField();
    private static final Style ROBOT_STYLE = new Style("", "#3F51B5", 0.75);
    private static final double LINE_WIDTH = 0.75;

    private FieldDrawing() {
    }

    public static void robot(Pose pose) {
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

    public static void line(double x1, double y1, double x2, double y2, String color) {
        if (!Double.isFinite(x1) || !Double.isFinite(y1) || !Double.isFinite(x2) || !Double.isFinite(y2)) {
            return;
        }
        FIELD.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        FIELD.setStyle("", color, LINE_WIDTH);
        FIELD.moveCursor(x1, y1);
        FIELD.line(x2, y2);
    }

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
