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

    private FieldDrawing() {
    }

    /** Robot circle + line to its front, then send. */
    public static void robot(Pose pose) {
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            return;
        }
        // Pedro coordinates. Set every frame: update() resets them.
        FIELD.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        FIELD.setStyle(ROBOT_STYLE);
        FIELD.moveCursor(pose.x(), pose.y());
        FIELD.circle(ROBOT_RADIUS_IN);
        FIELD.moveCursor(pose.x(), pose.y());
        FIELD.line(pose.x() + ROBOT_RADIUS_IN * Math.cos(pose.heading()),
                pose.y() + ROBOT_RADIUS_IN * Math.sin(pose.heading()));
        FIELD.update();
    }
}
