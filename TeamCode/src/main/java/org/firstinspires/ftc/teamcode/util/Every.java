package org.firstinspires.ftc.teamcode.util;

import java.util.function.LongSupplier;

/** True at most once per period. For telemetry, drawing and slow sensor reads. */
public final class Every {

    private final LongSupplier clockNanos;
    private long periodNanos;
    private long nextNanos = 0;
    private boolean started = false;

    public Every(double periodMs) {
        this(periodMs, System::nanoTime);
    }

    /** clockNanos is swappable for tests. */
    public Every(double periodMs, LongSupplier clockNanos) {
        this.clockNanos = clockNanos;
        setPeriodMs(periodMs);
    }

    public void setPeriodMs(double periodMs) {
        periodNanos = (long) (periodMs * 1e6);
    }

    /** True on the first call, then once each period. Missed periods do not pile up. */
    public boolean ready() {
        long now = clockNanos.getAsLong();
        if (!started || now >= nextNanos) {
            started = true;
            nextNanos = now + periodNanos;
            return true;
        }
        return false;
    }

    /** Make the next ready() true. */
    public void reset() {
        started = false;
    }
}
