package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class LoopTimerTest {

    private static final long MS = 1_000_000L;

    @Test
    public void steadyLoopsGiveSteadyStats() {
        LoopTimer timer = new LoopTimer(2.0);
        long now = 0;
        for (int i = 0; i <= 20; i++) {
            timer.tick(now);
            now += 10 * MS;
        }
        timer.compute();
        assertEquals(20, timer.count());
        assertEquals(10.0, timer.meanMs(), 1e-9);
        assertEquals(10.0, timer.p95Ms(), 1e-9);
        assertEquals(10.0, timer.maxMs(), 1e-9);
    }

    @Test
    public void oneSpikeShowsInMaxNotP95() {
        LoopTimer timer = new LoopTimer(2.0);
        long now = 0;
        timer.tick(now);
        for (int i = 0; i < 19; i++) {
            now += 10 * MS;
            timer.tick(now);
        }
        now += 50 * MS;
        timer.tick(now);
        timer.compute();
        assertEquals(20, timer.count());
        assertEquals(50.0, timer.maxMs(), 1e-9);
        assertEquals(10.0, timer.p95Ms(), 1e-9);
        assertEquals(12.0, timer.meanMs(), 1e-9);
    }

    @Test
    public void onlyLoopsInsideTheWindowCount() {
        LoopTimer timer = new LoopTimer(0.5);
        long now = 0;
        for (int i = 0; i <= 100; i++) {
            timer.tick(now);
            now += 10 * MS;
        }
        timer.compute();
        assertEquals(51, timer.count());
    }

    @Test
    public void noLoopsGivesZeros() {
        LoopTimer timer = new LoopTimer(2.0);
        timer.compute();
        assertEquals(0, timer.count());
        assertEquals(0.0, timer.maxMs(), 1e-9);
    }
}
