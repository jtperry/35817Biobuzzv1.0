package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import java.util.Locale;

/**
 * Shows the detected type, every axis and every button of both pads, and rumbles them. A plain SDK OpMode:
 * no Robot, no NextFTC hooks, no hardware, never drives. It reads the raw gamepad fields, so it works in INIT too.
 * Each pad rumbles itself: A = both motors 500 ms, B = 3 blips, X = stop.
 */
@TeleOp(name = "Gamepad Test", group = "test")
public class GamepadTest extends OpMode {

    private static final int RUMBLE_MS = 500;

    private RobotTelemetry tele;
    private final RumbleTest rumble1 = new RumbleTest();
    private final RumbleTest rumble2 = new RumbleTest();

    @Override
    public void init() {
        tele = new RobotTelemetry(telemetry);
    }

    @Override
    public void init_loop() {
        step();
    }

    @Override
    public void loop() {
        step();
    }

    @Override
    public void stop() {
        gamepad1.stopRumble();
        gamepad2.stopRumble();
    }

    private void step() {
        rumble1.update(gamepad1);
        rumble2.update(gamepad2);

        if (!tele.beginFrame()) {
            return;
        }
        show(1, gamepad1, rumble1);
        show(2, gamepad2, rumble2);
        tele.line("Rumble (each pad on itself): A = both motors 500 ms, B = 3 blips, X = stop.");
        tele.line("Xbox One / Series pad: the type is the Driver Station's guess. If buttons or rumble look wrong, set"
                + " Driver Station > Settings > Gamepad Type Overrides to Xbox 360 for that pad, then run this again.");
        tele.line("Stick y is negative when pushed up. guide / ps may never arrive (the OS can capture it).");
        tele.endFrame();
    }

    private void show(int number, Gamepad pad, RumbleTest rumble) {
        String name = "pad " + number;
        boolean assigned = pad.getGamepadId() != Gamepad.ID_UNASSOCIATED;
        tele.add(name + " type", pad.type() + (assigned ? "" : "   (no pad: Start + A / Start + B on the Driver Station)"));
        tele.add(name + " sticks", String.format(Locale.US, "L (%.2f, %.2f)   R (%.2f, %.2f)",
                pad.left_stick_x, pad.left_stick_y, pad.right_stick_x, pad.right_stick_y));
        tele.add(name + " triggers", String.format(Locale.US, "L %.2f   R %.2f", pad.left_trigger, pad.right_trigger));
        tele.add(name + " buttons", pressed(pad));
        if (hasTouchpad(pad)) {
            tele.add(name + " touchpad", String.format(Locale.US, "finger 1 %s (%.2f, %.2f)   finger 2 %s (%.2f, %.2f)",
                    pad.touchpad_finger_1 ? "down" : "up", pad.touchpad_finger_1_x, pad.touchpad_finger_1_y,
                    pad.touchpad_finger_2 ? "down" : "up", pad.touchpad_finger_2_x, pad.touchpad_finger_2_y));
        }
        tele.add(name + " rumble", rumble.last + (pad.isRumbling() ? "   (rumbling)" : ""));
    }

    /** The touchpad line is for PS4 pads; any other pad shows it only if it reports a touch. */
    private static boolean hasTouchpad(Gamepad pad) {
        return pad.type() == Gamepad.Type.SONY_PS4 || pad.type() == Gamepad.Type.SONY_PS4_SUPPORTED_BY_KERNEL
                || pad.touchpad || pad.touchpad_finger_1 || pad.touchpad_finger_2;
    }

    /** Names of the pressed buttons. back / start / guide are the same buttons as share / options / ps. */
    private static String pressed(Gamepad pad) {
        StringBuilder out = new StringBuilder();
        append(out, pad.a, "A");
        append(out, pad.b, "B");
        append(out, pad.x, "X");
        append(out, pad.y, "Y");
        append(out, pad.dpad_up, "dpad up");
        append(out, pad.dpad_down, "dpad down");
        append(out, pad.dpad_left, "dpad left");
        append(out, pad.dpad_right, "dpad right");
        append(out, pad.left_bumper, "LB");
        append(out, pad.right_bumper, "RB");
        append(out, pad.left_stick_button, "left stick");
        append(out, pad.right_stick_button, "right stick");
        append(out, pad.back, "back/share");
        append(out, pad.start, "start/options");
        append(out, pad.guide, "guide/ps");
        append(out, pad.touchpad, "touchpad");
        return out.length() == 0 ? "none" : out.toString();
    }

    private static void append(StringBuilder out, boolean down, String name) {
        if (down) {
            if (out.length() > 0) {
                out.append(", ");
            }
            out.append(name);
        }
    }

    /** Rumble on the raw A / B / X fields: one rumble per press, not one per loop. */
    private static final class RumbleTest {
        private boolean wasA;
        private boolean wasB;
        private boolean wasX;
        String last = "none yet";

        void update(Gamepad pad) {
            if (pad.a && !wasA) {
                pad.rumble(1.0, 1.0, RUMBLE_MS);
                last = "both motors, " + RUMBLE_MS + " ms";
            }
            if (pad.b && !wasB) {
                pad.rumbleBlips(3);
                last = "3 blips";
            }
            if (pad.x && !wasX) {
                pad.stopRumble();
                last = "stopped";
            }
            wasA = pad.a;
            wasB = pad.b;
            wasX = pad.x;
        }
    }
}
