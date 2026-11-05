package org.firstinspires.ftc.teamcode.tuning;

import java.util.function.BooleanSupplier;

/**
 * One gamepad button that acts only after it has been seen released since PLAY: a button held through PLAY does
 * nothing until it is released and pressed again. Draining the edge flags at PLAY does not work: the SDK resets edge
 * detection before start() runs and every Driver Station resend latches a fresh press of a held button, and a NextFTC
 * binding reads a button pressed during INIT as a rising edge on its first poll after PLAY. Pure: the readers are
 * given, so it runs on the laptop.
 */
public final class ButtonArming {

    /** The press edge (reading it consumes it), or null when a binding acts on the button. */
    private final BooleanSupplier edge;
    /** The raw button field. */
    private final BooleanSupplier down;
    /** Seen released since start(); false until start(). */
    private boolean armed = false;

    /** For a button the loop acts on: edge = its wasPressed() reader, down = its raw field. */
    public ButtonArming(BooleanSupplier edge, BooleanSupplier down) {
        this.edge = edge;
        this.down = down;
    }

    /** For a button a binding acts on: poll() only arms it, and the binding checks armed(). */
    public ButtonArming(BooleanSupplier down) {
        this(null, down);
    }

    /** At PLAY: armed only if the button is up now. */
    public void start() {
        armed = !down.getAsBoolean();
    }

    /**
     * Once per loop, in this order: read (and so consume) the edge; if the button was not armed at the start of this
     * loop, drop the edge and arm it if the raw field reads up; return whether to act on a press this loop. Always
     * false for a binding's button.
     */
    public boolean poll() {
        boolean pressed = edge != null && edge.getAsBoolean();
        if (!armed) {
            armed = !down.getAsBoolean();
            return false;
        }
        return pressed;
    }

    public boolean armed() {
        return armed;
    }
}
