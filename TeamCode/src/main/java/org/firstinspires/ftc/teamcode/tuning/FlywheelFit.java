package org.firstinspires.ftc.teamcode.tuning;

/**
 * Least squares line volts = kS + kV * rpm for the flywheel feedforward (Flywheel Tuner, spec 5.1). Pure math:
 * the gains come out in the units of Flywheel.KS_VOLTS and Flywheel.KV_VOLTS_PER_RPM.
 */
public final class FlywheelFit {

    /** NaN gains when the points cannot fix a line (fewer than 2 distinct rpm values). */
    public static final class Result {
        public final double ksVolts;
        public final double kvVoltsPerRpm;
        /** 1.0 = every point on the line. NaN when every volts value is the same. */
        public final double rSquared;
        public final int samples;

        private Result(double ksVolts, double kvVoltsPerRpm, double rSquared, int samples) {
            this.ksVolts = ksVolts;
            this.kvVoltsPerRpm = kvVoltsPerRpm;
            this.rSquared = rSquared;
            this.samples = samples;
        }
    }

    private FlywheelFit() {
    }

    /** Fits the first count points of rpm and volts. */
    public static Result fit(double[] rpm, double[] volts, int count) {
        if (!hasTwoRpms(rpm, count)) {
            return new Result(Double.NaN, Double.NaN, Double.NaN, count);
        }
        double meanRpm = 0;
        double meanVolts = 0;
        for (int i = 0; i < count; i++) {
            meanRpm += rpm[i];
            meanVolts += volts[i];
        }
        meanRpm /= count;
        meanVolts /= count;

        // Centered sums: RPM is in the thousands, and raw sums of squares would cancel away digits.
        double sxx = 0;
        double sxy = 0;
        for (int i = 0; i < count; i++) {
            double dx = rpm[i] - meanRpm;
            sxx += dx * dx;
            sxy += dx * (volts[i] - meanVolts);
        }
        double kv = sxy / sxx;
        double ks = meanVolts - kv * meanRpm;

        double ssRes = 0;
        double ssTot = 0;
        for (int i = 0; i < count; i++) {
            double residual = volts[i] - (ks + kv * rpm[i]);
            double dy = volts[i] - meanVolts;
            ssRes += residual * residual;
            ssTot += dy * dy;
        }
        double rSquared = allEqual(volts, count) ? Double.NaN : 1 - ssRes / ssTot;
        return new Result(ks, kv, rSquared, count);
    }

    private static boolean hasTwoRpms(double[] rpm, int count) {
        return !allEqual(rpm, count);
    }

    /** Exact comparison: the mean of equal values can differ from them by rounding, so ssTot > 0 would not do. */
    private static boolean allEqual(double[] values, int count) {
        for (int i = 1; i < count; i++) {
            if (values[i] != values[0]) {
                return false;
            }
        }
        return true;
    }
}
