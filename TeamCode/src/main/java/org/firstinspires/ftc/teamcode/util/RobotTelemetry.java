package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Telemetry for one OpMode: Driver Station + Panels at HZ, with a loop-time line on top.
 * Nothing is registered globally, so nothing piles up across OpMode runs (spec 4.4).
 *
 * Each loop:
 *   if (tele.beginFrame()) { tele.add("x", x); tele.detail("y", y); tele.endFrame(); }
 */
@Configurable
public final class RobotTelemetry {

    /** Frames per second. Panels sends at most every 75 ms, so keep this at 13 or less. */
    public static double HZ = 10;

    /** False = hide detail() lines. */
    public static boolean VERBOSE = true;

    private final Telemetry driverStation;
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final LoopTimer loopTimer = new LoopTimer(2.0);
    private final Every frame = new Every(1000.0 / HZ);
    private boolean inFrame = false;

    public RobotTelemetry(Telemetry driverStation) {
        this.driverStation = driverStation;
    }

    /** Call exactly once per loop. True = write this frame now: add lines, then endFrame(). */
    public boolean beginFrame() {
        loopTimer.tick(System.nanoTime());
        frame.setPeriodMs(1000.0 / Math.max(HZ, 1.0));
        if (!frame.ready()) {
            return false;
        }
        inFrame = true;
        loopTimer.compute();
        add("loop", loopTimer.summary());
        return true;
    }

    /** Always shown. */
    public void add(String caption, Object value) {
        line(caption + ": " + value);
    }

    /** Shown only when VERBOSE. */
    public void detail(String caption, Object value) {
        if (VERBOSE) {
            add(caption, value);
        }
    }

    public void line(String text) {
        panels.addLine(text);
        driverStation.addLine(text);
    }

    /** Send the frame to Panels and the Driver Station. */
    public void endFrame() {
        if (!inFrame) {
            return;
        }
        inFrame = false;
        panels.update();
        driverStation.update();
    }
}
