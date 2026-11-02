package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class EveryTest {

    private static final long MS = 1_000_000L;
    private final long[] clock = {0};

    @Test
    public void firstCallIsReadyThenOncePerPeriod() {
        Every every = new Every(100, () -> clock[0]);
        assertTrue(every.ready());
        clock[0] = 50 * MS;
        assertFalse(every.ready());
        clock[0] = 100 * MS;
        assertTrue(every.ready());
        clock[0] = 150 * MS;
        assertFalse(every.ready());
    }

    @Test
    public void missedPeriodsDoNotPileUp() {
        Every every = new Every(100, () -> clock[0]);
        assertTrue(every.ready());
        clock[0] = 1000 * MS;
        assertTrue(every.ready());
        assertFalse(every.ready());
        clock[0] = 1099 * MS;
        assertFalse(every.ready());
    }

    @Test
    public void resetMakesNextCallReady() {
        Every every = new Every(100, () -> clock[0]);
        assertTrue(every.ready());
        every.reset();
        assertTrue(every.ready());
    }
}
