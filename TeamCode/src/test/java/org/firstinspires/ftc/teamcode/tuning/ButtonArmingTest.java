package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ButtonArmingTest {

    /** A gamepad button: a press latches an edge until wasPressed() reads it, as the SDK does. */
    private static final class Button {

        boolean down = false;
        boolean latched = false;
        int edgeReads = 0;

        boolean wasPressed() {
            edgeReads++;
            boolean pressed = latched;
            latched = false;
            return pressed;
        }

        /** Pressed and still down; also a Driver Station resend of a held button latches a press. */
        void press() {
            down = true;
            latched = true;
        }

        void release() {
            down = false;
        }

        ButtonArming arming() {
            return new ButtonArming(this::wasPressed, () -> down);
        }
    }

    @Test
    public void aButtonHeldThroughPlayDropsItsEdgesUntilReleased() {
        Button button = new Button();
        button.press();
        ButtonArming arming = button.arming();
        arming.start();
        for (int loop = 0; loop < 3; loop++) {
            button.press();
            assertFalse("loop " + loop, arming.poll());
            assertFalse(button.latched);
        }
        button.release();
        assertFalse(arming.poll());
        assertTrue(arming.armed());
    }

    @Test
    public void aPressAfterTheReleaseActs() {
        Button button = new Button();
        button.press();
        ButtonArming arming = button.arming();
        arming.start();
        assertFalse(arming.poll());
        button.release();
        assertFalse(arming.poll());
        button.press();
        assertTrue(arming.poll());
        assertFalse(arming.poll());
        button.release();
        assertFalse(arming.poll());
        button.press();
        assertTrue(arming.poll());
    }

    @Test
    public void anEdgeSeenInTheSameLoopAsTheReleaseIsDropped() {
        Button button = new Button();
        button.press();
        ButtonArming arming = button.arming();
        arming.start();
        button.press();
        button.release();
        assertFalse(arming.poll());
        assertFalse(button.latched);
        assertTrue(arming.armed());
        assertFalse(arming.poll());
    }

    @Test
    public void aNeverHeldButtonActsOnItsFirstPress() {
        Button button = new Button();
        ButtonArming arming = button.arming();
        arming.start();
        assertTrue(arming.armed());
        button.press();
        assertTrue(arming.poll());
    }

    @Test
    public void theEdgeIsReadEveryLoopEvenWhileNotArmed() {
        Button button = new Button();
        button.press();
        ButtonArming arming = button.arming();
        arming.start();
        arming.poll();
        arming.poll();
        assertEquals(2, button.edgeReads);
    }

    @Test
    public void nothingIsArmedBeforeStart() {
        Button button = new Button();
        ButtonArming arming = button.arming();
        assertFalse(arming.armed());
        button.press();
        assertFalse(arming.poll());
    }

    /** A binding's button: poll() never acts, armed() turns true once the button is seen released. */
    @Test
    public void aBindingButtonIsArmedOnceReleased() {
        Button button = new Button();
        button.press();
        ButtonArming arming = new ButtonArming(() -> button.down);
        arming.start();
        assertFalse(arming.armed());
        assertFalse(arming.poll());
        assertFalse(arming.armed());
        button.release();
        assertFalse(arming.poll());
        assertTrue(arming.armed());
        button.press();
        assertFalse(arming.poll());
        assertTrue(arming.armed());
        assertTrue(button.latched);
    }
}
